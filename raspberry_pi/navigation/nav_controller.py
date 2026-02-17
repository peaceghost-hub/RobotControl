"""
Pi-Side Autonomous Navigation Controller
==========================================

Runs the full navigation state machine on the Raspberry Pi where the
compass sensor lives (zero-latency heading reads).  Sends simple drive
commands to the Arduino Mega over I2C — the Mega acts as a motor driver
only, no navigation math.

State machine:
    IDLE → ACQUIRING_HEADING → NAVIGATING → WAYPOINT_REACHED → (next) → COMPLETE
                ↑                    |
          OBSTACLE_AVOID  ←  OBSTACLE_DETECTED

GOLDEN RULE: NO FREEZING.  Every step is non-blocking.
"""

import math
import time
import logging
import threading
from enum import Enum, auto
from typing import Optional, Dict, List, Any

logger = logging.getLogger("nav_controller")


class NavState(Enum):
    IDLE               = auto()
    ACQUIRING_HEADING   = auto()
    HEADING_ACQUIRED    = auto()  # brief transition — shown on dashboard
    NAVIGATING          = auto()
    OBSTACLE_DETECTED   = auto()
    OBSTACLE_AVOID      = auto()
    WAYPOINT_REACHED    = auto()
    COMPLETE            = auto()
    PAUSED              = auto()


class NavController:
    """Pi-side autonomous navigation engine.

    Uses compass (on Pi I2C) and GPS (SIM7600E on Pi) for all decisions.
    Sends only motor drive commands to Mega via the existing I2C link.
    """

    # ---- Tuning ----
    WAYPOINT_RADIUS      = 3.0    # meters — "reached"
    HEADING_DEADBAND     = 5.0    # degrees — close enough → go straight
    ACQUIRE_SLOW_THRESH  = 15.0   # degrees — switch to slow rotation
    ROTATION_SPEED_FAST  = 150    # PWM for big heading error
    ROTATION_SPEED_SLOW  = 100    # PWM for small heading error
    DRIVE_SPEED          = 120    # PWM forward
    OBSTACLE_TURN_TIME   = 1.0    # seconds — rotate away from obstacle
    OBSTACLE_CHECK_PAUSE = 0.3    # seconds — pause after stop before turning
    COURSE_DRIFT_THRESH  = 12.0   # degrees — re-acquire heading while driving
    HEADING_ACQUIRE_TIMEOUT = 30.0  # seconds — give up if can't acquire
    NAV_LOOP_HZ         = 10     # control loop frequency

    def __init__(self, compass, robot_link, gps_provider, config: dict = None):
        """
        Args:
            compass:       Compass instance (raspberry_pi.sensors.compass.Compass)
            robot_link:    I2CComm instance for sending drive commands to Mega
            gps_provider:  callable() → dict with 'latitude', 'longitude', 'speed'
                           OR an object with .get_position() returning same
            config:        optional config dict overrides
        """
        self.compass = compass
        self.robot_link = robot_link
        self._gps_provider = gps_provider

        # Waypoints
        self._waypoints: List[Dict[str, Any]] = []
        self._current_wp_index = 0

        # State
        self._state = NavState.IDLE
        self._state_entry_time = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Obstacle sub-state
        self._obstacle_phase = 'stop'  # 'stop' → 'turn' → 'recheck'
        self._obstacle_phase_start = 0.0
        self._avoid_direction = 'left'  # alternate each attempt
        self._avoid_attempts = 0

        # Bearing / heading cache
        self._current_heading = 0.0
        self._target_bearing = 0.0
        self._last_heading_error = 0.0

        # Stats for dashboard
        self._distance_to_wp = 0.0
        self._completed_waypoints: List[int] = []

        # Motor trim (compensate hardware mismatch)
        cfg = config or {}
        nav_cfg = cfg.get('navigation', {})
        self._motor_trim_left  = int(nav_cfg.get('motor_trim_left', 0))
        self._motor_trim_right = int(nav_cfg.get('motor_trim_right', 0))
        self.DRIVE_SPEED = int(nav_cfg.get('drive_speed', self.DRIVE_SPEED))
        self.WAYPOINT_RADIUS = float(nav_cfg.get('waypoint_radius', self.WAYPOINT_RADIUS))

        logger.info("NavController created (trim L=%+d R=%+d, speed=%d, radius=%.1fm)",
                     self._motor_trim_left, self._motor_trim_right,
                     self.DRIVE_SPEED, self.WAYPOINT_RADIUS)

    # ================================================================
    #  PUBLIC API
    # ================================================================

    @property
    def state(self) -> NavState:
        with self._lock:
            return self._state

    @property
    def is_active(self) -> bool:
        return self._running and self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED)

    def get_status(self) -> dict:
        """Snapshot for dashboard display."""
        # Read live compass heading even when nav loop is not running
        heading = self._current_heading
        if not self._running and self.compass:
            try:
                h = self.compass.read_heading()
                if h is not None:
                    heading = h
            except Exception:
                pass
        with self._lock:
            return {
                'state': self._state.name,
                'current_heading': round(heading, 1),
                'target_bearing': round(self._target_bearing, 1) if self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED) else None,
                'heading_error': round(self._last_heading_error, 1) if self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED) else None,
                'distance_to_wp': round(self._distance_to_wp, 1) if self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED) else None,
                'current_wp_index': self._current_wp_index,
                'total_waypoints': len(self._waypoints),
                'completed_waypoints': list(self._completed_waypoints),
                'avoid_attempts': self._avoid_attempts,
            }

    def set_waypoints(self, waypoints: List[Dict[str, Any]]):
        """Load waypoints from dashboard.  Each: {latitude, longitude, id, description}."""
        with self._lock:
            self._waypoints = list(waypoints)
            self._current_wp_index = 0
            self._completed_waypoints = []
        logger.info("Loaded %d waypoints", len(waypoints))

    def start(self):
        """Begin autonomous navigation."""
        with self._lock:
            if not self._waypoints:
                logger.warning("Cannot start nav — no waypoints")
                return False
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._avoid_attempts = 0
            self._avoid_direction = 'left'
        self._running = True
        self._enter_state(NavState.ACQUIRING_HEADING)

        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        logger.info("Navigation STARTED (%d waypoints)", len(self._waypoints))
        return True

    def stop(self):
        """Stop navigation completely."""
        self._running = False
        self._send_stop()
        self._enter_state(NavState.IDLE)
        logger.info("Navigation STOPPED")

    def pause(self):
        """Pause — stop motors but remember position."""
        self._send_stop()
        self._enter_state(NavState.PAUSED)
        logger.info("Navigation PAUSED")

    def resume(self):
        """Resume from pause."""
        if self._state != NavState.PAUSED:
            return
        self._running = True
        self._enter_state(NavState.ACQUIRING_HEADING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        logger.info("Navigation RESUMED")

    def accept_heading(self):
        """Manually accept current heading as good enough.

        Called from dashboard when the operator sees the robot is already on
        the right path but the heading error is larger than HEADING_DEADBAND.
        Stops rotation immediately and transitions to HEADING_ACQUIRED → NAVIGATING.
        """
        with self._lock:
            if self._state not in (NavState.ACQUIRING_HEADING,):
                logger.info("accept_heading ignored — state is %s", self._state.name)
                return False
        self._send_stop()
        logger.info("Heading manually accepted (error=%.1f°)", self._last_heading_error)
        self._enter_state(NavState.HEADING_ACQUIRED)
        return True

    # ================================================================
    #  MAIN CONTROL LOOP (runs in its own thread)
    # ================================================================

    def _nav_loop(self):
        """Non-blocking control loop running at NAV_LOOP_HZ."""
        interval = 1.0 / self.NAV_LOOP_HZ
        logger.info("Nav loop started @ %d Hz", self.NAV_LOOP_HZ)

        while self._running:
            try:
                self._tick()
            except Exception as e:
                logger.error("Nav tick error: %s", e, exc_info=True)
                # Safety: stop motors on any error
                self._send_stop()

            time.sleep(interval)

        logger.info("Nav loop exited")

    def _tick(self):
        """Single iteration of the navigation state machine."""
        state = self.state

        if state == NavState.IDLE or state == NavState.COMPLETE or state == NavState.PAUSED:
            return

        # Read sensors every tick
        self._current_heading = self._read_heading()
        gps = self._get_gps()
        if not gps:
            return  # no GPS fix — wait

        # Compute bearing and distance to current waypoint
        with self._lock:
            if self._current_wp_index >= len(self._waypoints):
                self._enter_state(NavState.COMPLETE)
                self._send_stop()
                return
            wp = self._waypoints[self._current_wp_index]

        wp_lat = float(wp['latitude'])
        wp_lon = float(wp['longitude'])
        cur_lat = gps['latitude']
        cur_lon = gps['longitude']

        self._distance_to_wp = self._haversine(cur_lat, cur_lon, wp_lat, wp_lon)
        self._target_bearing = self._bearing(cur_lat, cur_lon, wp_lat, wp_lon)
        self._last_heading_error = self._normalize_error(self._target_bearing - self._current_heading)

        # ---- Dispatch to state handler ----
        if state == NavState.ACQUIRING_HEADING:
            self._handle_acquiring_heading()
        elif state == NavState.HEADING_ACQUIRED:
            self._handle_heading_acquired()
        elif state == NavState.NAVIGATING:
            self._handle_navigating()
        elif state == NavState.OBSTACLE_DETECTED:
            self._handle_obstacle_detected()
        elif state == NavState.OBSTACLE_AVOID:
            self._handle_obstacle_avoid()
        elif state == NavState.WAYPOINT_REACHED:
            self._handle_waypoint_reached()

    # ================================================================
    #  STATE HANDLERS
    # ================================================================

    def _handle_acquiring_heading(self):
        """Rotate robot until heading matches target bearing (within deadband)."""
        error = self._last_heading_error
        abs_error = abs(error)

        # Timeout check
        elapsed = time.time() - self._state_entry_time
        if elapsed > self.HEADING_ACQUIRE_TIMEOUT:
            logger.warning("Heading acquire timeout — proceeding anyway")
            self._enter_state(NavState.NAVIGATING)
            return

        if abs_error <= self.HEADING_DEADBAND:
            # Heading acquired!
            self._send_stop()
            self._enter_state(NavState.HEADING_ACQUIRED)
            return

        # Rotate toward target
        if error > 0:
            # Target is to the RIGHT → rotate right
            speed = self.ROTATION_SPEED_SLOW if abs_error < self.ACQUIRE_SLOW_THRESH else self.ROTATION_SPEED_FAST
            self._send_rotate_right(speed)
        else:
            # Target is to the LEFT → rotate left
            speed = self.ROTATION_SPEED_SLOW if abs_error < self.ACQUIRE_SLOW_THRESH else self.ROTATION_SPEED_FAST
            self._send_rotate_left(speed)

    def _handle_heading_acquired(self):
        """Brief pause state for dashboard display, then start navigating."""
        elapsed = time.time() - self._state_entry_time
        if elapsed >= 0.5:  # show "HEADING ACQUIRED" for 0.5s
            self._enter_state(NavState.NAVIGATING)

    def _handle_navigating(self):
        """Drive forward. Monitor: waypoint reached, obstacle, course drift."""
        # Check waypoint reached
        if self._distance_to_wp <= self.WAYPOINT_RADIUS:
            self._send_stop()
            self._enter_state(NavState.WAYPOINT_REACHED)
            return

        # Check obstacle (poll Mega)
        if self._check_obstacle():
            self._send_stop()
            self._enter_state(NavState.OBSTACLE_DETECTED)
            return

        # Check course drift — if heading drifted too far, re-acquire
        if abs(self._last_heading_error) > self.COURSE_DRIFT_THRESH:
            logger.info("Course drift %.1f° — re-acquiring heading", self._last_heading_error)
            self._send_stop()
            self._enter_state(NavState.ACQUIRING_HEADING)
            return

        # All clear — drive forward
        self._send_forward(self.DRIVE_SPEED)

    def _handle_obstacle_detected(self):
        """Brief stop, then enter avoidance."""
        elapsed = time.time() - self._state_entry_time
        if elapsed >= self.OBSTACLE_CHECK_PAUSE:
            self._avoid_attempts += 1
            # Alternate turn direction each attempt
            self._avoid_direction = 'left' if self._avoid_attempts % 2 == 1 else 'right'
            self._obstacle_phase = 'turn'
            self._obstacle_phase_start = time.time()
            self._enter_state(NavState.OBSTACLE_AVOID)

    def _handle_obstacle_avoid(self):
        """Multi-phase: turn away → check clear → re-acquire heading."""
        elapsed = time.time() - self._obstacle_phase_start

        if self._obstacle_phase == 'turn':
            # Rotate away from obstacle
            if elapsed < self.OBSTACLE_TURN_TIME:
                if self._avoid_direction == 'left':
                    self._send_rotate_left(self.ROTATION_SPEED_FAST)
                else:
                    self._send_rotate_right(self.ROTATION_SPEED_FAST)
            else:
                # Turn complete — stop and recheck
                self._send_stop()
                self._obstacle_phase = 'recheck'
                self._obstacle_phase_start = time.time()

        elif self._obstacle_phase == 'recheck':
            if elapsed >= self.OBSTACLE_CHECK_PAUSE:
                if self._check_obstacle():
                    # Still blocked — turn more
                    self._obstacle_phase = 'turn'
                    self._obstacle_phase_start = time.time()
                    if self._avoid_attempts > 5:
                        logger.warning("Too many obstacle attempts — skipping waypoint")
                        self._advance_waypoint()
                else:
                    # Clear — re-acquire heading then navigate
                    logger.info("Obstacle cleared — re-acquiring heading")
                    self._enter_state(NavState.ACQUIRING_HEADING)

    def _handle_waypoint_reached(self):
        """Mark waypoint complete, advance to next."""
        elapsed = time.time() - self._state_entry_time
        if elapsed >= 0.5:  # brief pause
            self._advance_waypoint()

    # ================================================================
    #  MOTOR COMMANDS (sent to Mega via I2C)
    # ================================================================

    def _send_forward(self, speed: int):
        """Drive both motors forward at `speed` PWM."""
        left = speed + self._motor_trim_left
        right = speed + self._motor_trim_right
        self._send_motor_command(left, right)

    def _send_rotate_left(self, speed: int):
        """Rotate left: left motor backward, right motor forward."""
        self._send_motor_command(-speed, speed)

    def _send_rotate_right(self, speed: int):
        """Rotate right: left motor forward, right motor backward."""
        self._send_motor_command(speed, -speed)

    def _send_stop(self):
        """Stop both motors."""
        self._send_motor_command(0, 0)

    def _send_motor_command(self, left: int, right: int):
        """Send motor command to Mega using CMD_MANUAL_OVERRIDE."""
        try:
            if self.robot_link and hasattr(self.robot_link, 'send_manual_control'):
                self.robot_link.send_manual_control(left, right, joystick_active=True)
        except Exception as e:
            logger.debug("Motor command failed: %s", e)

    # ================================================================
    #  SENSOR READS
    # ================================================================

    def _read_heading(self) -> float:
        """Read compass heading from Pi-local compass."""
        try:
            return self.compass.read_heading()
        except Exception:
            return self._current_heading  # return last known

    def _get_gps(self) -> Optional[dict]:
        """Get current GPS position."""
        try:
            if callable(self._gps_provider):
                return self._gps_provider()
            elif hasattr(self._gps_provider, 'get_position'):
                return self._gps_provider.get_position()
            return None
        except Exception:
            return None

    def _check_obstacle(self) -> bool:
        """Poll Mega for obstacle status."""
        try:
            if self.robot_link and hasattr(self.robot_link, 'request_obstacle_status'):
                result = self.robot_link.request_obstacle_status()
                if result:
                    return result.get('obstacle', False)
        except Exception:
            pass
        return False

    # ================================================================
    #  HELPERS
    # ================================================================

    def _enter_state(self, new_state: NavState):
        with self._lock:
            old = self._state
            self._state = new_state
            self._state_entry_time = time.time()
        if old != new_state:
            logger.info("NAV: %s → %s", old.name, new_state.name)

    def _advance_waypoint(self):
        """Move to next waypoint or complete."""
        with self._lock:
            if self._current_wp_index < len(self._waypoints):
                wp = self._waypoints[self._current_wp_index]
                wp_id = wp.get('id', self._current_wp_index)
                self._completed_waypoints.append(wp_id)
                logger.info("Waypoint %d reached! (%d/%d)",
                            wp_id, self._current_wp_index + 1, len(self._waypoints))
                self._current_wp_index += 1

            if self._current_wp_index >= len(self._waypoints):
                self._send_stop()
                self._enter_state(NavState.COMPLETE)
                self._running = False
                logger.info("ALL WAYPOINTS COMPLETE!")
                return

        # Reset per-waypoint counters
        self._avoid_attempts = 0
        self._enter_state(NavState.ACQUIRING_HEADING)

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Distance in meters between two GPS coordinates."""
        R = 6371000.0  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Bearing in degrees (0–360) from point 1 to point 2."""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
        bearing = math.degrees(math.atan2(x, y))
        return bearing % 360.0

    @staticmethod
    def _normalize_error(error: float) -> float:
        """Normalize angle difference to -180..+180."""
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error
