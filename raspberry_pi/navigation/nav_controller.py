"""
Pi-Side Autonomous Navigation Controller
==========================================

Runs the full navigation state machine on the Raspberry Pi where the
compass sensor lives (zero-latency heading reads).  Sends simple drive
commands to the Arduino Mega over I2C — the Mega acts as a motor driver
only, no navigation math.

Geodesic calculations use GeographicLib (Karney's algorithm) on the
WGS-84 ellipsoid for sub-millimetre bearing & distance accuracy.
Falls back to spherical haversine if the library is unavailable.

State machine:
    IDLE → PREPARING (3s) → ACQUIRING_HEADING → HEADING_ACQUIRED (10s)
         → NAVIGATING → WAYPOINT_REACHED (3s) → PREPARING → ...
              ↑                    |
        OBSTACLE_AVOID  ←  OBSTACLE_DETECTED → PREPARING → ...

GOLDEN RULE: NO FREEZING.  Every step is non-blocking.
"""

import math
import time
import logging
import threading
from enum import Enum, auto
from typing import Optional, Dict, List, Any

# Karney's WGS-84 geodesic — gold-standard bearing & distance
try:
    from geographiclib.geodesic import Geodesic
    _geod = Geodesic.WGS84
    _HAS_GEOGRAPHICLIB = True
except ImportError:
    _geod = None
    _HAS_GEOGRAPHICLIB = False

logger = logging.getLogger("nav_controller")
if _HAS_GEOGRAPHICLIB:
    logger.info("GeographicLib loaded — using Karney WGS-84 geodesic (sub-mm precision)")
else:
    logger.warning("geographiclib not installed — falling back to spherical haversine")


class NavState(Enum):
    IDLE               = auto()
    PREPARING           = auto()  # 3s pause before acquiring — user can review
    ACQUIRING_HEADING   = auto()
    HEADING_ACQUIRED    = auto()  # 10s countdown before forward drive
    NAVIGATING          = auto()
    OBSTACLE_DETECTED   = auto()
    OBSTACLE_AVOID      = auto()
    WAYPOINT_REACHED    = auto()  # 3s hold — notification shown
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
    ACQUIRE_SETTLE_TIME  = 0.35   # seconds — stay inside deadband before accept
    ACQUIRE_MIN_TURN_TIME = 0.45  # seconds — avoid instant left/right reversal
    ACQUIRE_REVERSE_HYST = 10.0   # degrees — error required before turn reversal
    HEADING_FILTER_ALPHA = 0.35   # low-pass smoothing for compass heading
    ACQUIRE_BEARING_SAMPLES = 6   # number of live bearing samples to average
    ACQUIRE_TIMEOUT_RETRIES = 2   # retries before pausing for manual review
    DRIVE_SPEED          = 150    # PWM forward
    SAFE_AVOID_DISTANCE_CM = 40   # reverse back to at least this distance before avoiding
    SAFE_REVERSE_TIMEOUT = 3.0    # seconds — never reverse indefinitely
    OBSTACLE_TURN_TIME   = 1.0    # seconds — rotate away from obstacle
    OBSTACLE_PASS_TIME   = 1.5    # seconds — drive forward to get past obstacle before re-acquire
    OBSTACLE_CHECK_PAUSE = 0.3    # seconds — pause after stop before turning
    COURSE_DRIFT_THRESH  = 12.0   # degrees — re-acquire heading while driving
    NAV_GRACE_PERIOD     = 2.0    # seconds — skip drift check at start of NAVIGATING
    HEADING_ACQUIRE_TIMEOUT = 30.0  # seconds — give up if can't acquire
    HEADING_HOLD_TIME    = 10.0   # seconds — countdown before forward drive
    AI_ADVICE_TIMEOUT    = 10.0   # seconds — wait for AI obstacle advice before fallback
    PREPARE_TIME         = 3.0    # seconds — pause before acquiring heading
    WAYPOINT_HOLD_TIME   = 3.0    # seconds — pause after waypoint reached
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
        self._mega_gps_provider = None  # set externally for Neo-6M fallback

        # Dashboard notification (consumed once per read)
        self._notification = None   # e.g. {'level': 'info', 'msg': '...'}
        self._gps_wait_logged = False  # GPS-wait throttle flag

        # Neo-6M satellite count (polled from Mega)
        self._neo_satellites = 0

        # Waypoints
        self._waypoints: List[Dict[str, Any]] = []
        self._current_wp_index = 0
        self._heading_lock_active = False
        self._heading_lock_bearing: Optional[float] = None
        self._heading_lock_home: Optional[Dict[str, Any]] = None
        self._heading_lock_outbound_bearing: Optional[float] = None

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
        self._last_obstacle_distance_cm = -1
        self._clearance_notice_phase = 0.0

        # Bearing / heading cache
        self._current_heading = 0.0
        self._current_heading_magnetic = 0.0  # raw magnetic for dashboard
        self._filtered_heading: Optional[float] = None
        self._target_bearing = 0.0
        self._live_target_bearing: Optional[float] = None
        self._acquire_target_bearing: Optional[float] = None
        self._acquire_deadband_since = None
        self._acquire_turn_direction = 0   # -1 left, +1 right
        self._acquire_turn_since = 0.0
        self._bearing_samples: List[float] = []
        self._last_heading_error = 0.0
        self._acquire_timeout_retries = 0

        # AI obstacle advice (received from AI Vision via main.py)
        # When obstacle is detected, NavController waits for AI to advise
        # on avoidance direction.  If no advice within AI_ADVICE_TIMEOUT,
        # falls back to traditional turn-away avoidance.
        self._ai_advice_event = threading.Event()
        self._ai_advice_direction = None   # 'FORWARD'|'LEFT'|'RIGHT'|'STOP'
        self._ai_advice_safety = None      # 'SAFE'|'CAUTION'|'DANGER'
        self._ai_analysis_triggered = False  # True if AI was asked for this obstacle

        # Heading confirmation (dashboard can accept/reject before forward drive)
        self._heading_confirmed = threading.Event()
        self._heading_confirm_result = None  # True=accept, False=reject, None=pending
        self._event_callback = None  # set externally: fn(event_type, payload)

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
        self._adaptive_speed = bool(nav_cfg.get('adaptive_speed', False))
        self.SAFE_AVOID_DISTANCE_CM = max(20, min(60, int(nav_cfg.get(
            'safe_avoid_distance_cm', self.SAFE_AVOID_DISTANCE_CM
        ))))
        self.SAFE_REVERSE_TIMEOUT = max(1.0, min(6.0, float(nav_cfg.get(
            'safe_reverse_timeout', self.SAFE_REVERSE_TIMEOUT
        ))))
        self.OBSTACLE_PASS_TIME = max(0.5, min(4.0, float(nav_cfg.get(
            'obstacle_pass_time', self.OBSTACLE_PASS_TIME
        ))))
        self._invert_rotation_direction = bool(nav_cfg.get('invert_rotation_direction', False))

        logger.info("NavController created (trim L=%+d R=%+d, speed=%d, radius=%.1fm, adaptive=%s, safe_avoid=%dcm, invert_rotation=%s)",
                     self._motor_trim_left, self._motor_trim_right,
                     self.DRIVE_SPEED, self.WAYPOINT_RADIUS,
                     self._adaptive_speed, self.SAFE_AVOID_DISTANCE_CM,
                     self._invert_rotation_direction)

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
        magnetic = self._current_heading_magnetic
        if not self._running and self.compass:
            try:
                h = self.compass.read_heading()
                if h is not None:
                    heading = h
                m = self.compass.read_heading_magnetic() if hasattr(self.compass, 'read_heading_magnetic') else h
                if m is not None:
                    magnetic = m
            except Exception:
                pass
        with self._lock:
            # Compute countdown remaining for timed states
            countdown = None
            if self._state == NavState.HEADING_ACQUIRED:
                remaining = self.HEADING_HOLD_TIME - (time.time() - self._state_entry_time)
                countdown = max(0, round(remaining))
            elif self._state == NavState.PREPARING:
                remaining = self.PREPARE_TIME - (time.time() - self._state_entry_time)
                countdown = max(0, round(remaining))
            elif self._state == NavState.WAYPOINT_REACHED:
                remaining = self.WAYPOINT_HOLD_TIME - (time.time() - self._state_entry_time)
                countdown = max(0, round(remaining))

            # Pop one-shot notification
            notif = self._notification
            self._notification = None

            return {
                'state': self._state.name,
                'nav_mode': 'heading_lock' if self._heading_lock_active else 'waypoint',
                'current_heading': round(heading, 1),
                'magnetic_heading': round(magnetic, 1),
                'target_bearing': round(self._target_bearing, 1) if self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED) else None,
                'heading_error': round(self._last_heading_error, 1) if self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED) else None,
                'distance_to_wp': round(self._distance_to_wp, 1) if self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED) else None,
                'current_wp_index': self._current_wp_index,
                'total_waypoints': len(self._waypoints),
                'completed_waypoints': list(self._completed_waypoints),
                'avoid_attempts': self._avoid_attempts,
                'countdown': countdown,
                'notification': notif,
                'neo_satellites': self._neo_satellites,
                'adaptive_speed': self._adaptive_speed,
                'base_drive_speed': int(self.DRIVE_SPEED),
                'heading_lock_bearing': round(self._heading_lock_bearing, 1) if self._heading_lock_active and self._heading_lock_bearing is not None else None,
                'heading_lock_home_available': self._heading_lock_home is not None,
                'safe_avoid_distance_cm': self.SAFE_AVOID_DISTANCE_CM,
                'last_obstacle_distance_cm': self._last_obstacle_distance_cm if self._last_obstacle_distance_cm > 0 else None,
            }

    def return_home(self):
        """Reverse the completed + remaining waypoints and navigate back to start.

        Takes the full waypoint list (in original order), reverses it,
        and begins navigation so the robot retraces its path.
        """
        with self._lock:
            heading_lock_home_available = self._heading_lock_home is not None and (
                self._heading_lock_active or len(self._waypoints) <= 1
            )
        if heading_lock_home_available:
            return self.return_to_heading_lock_home()

        with self._lock:
            if not self._waypoints:
                logger.warning("Cannot return home — no waypoints")
                return False
            self._heading_lock_active = False
            self._heading_lock_bearing = None
            # Reverse the FULL waypoint list (already visited + remaining)
            reversed_wps = list(reversed(self._waypoints))
            self._waypoints = reversed_wps
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._avoid_attempts = 0
            self._avoid_direction = 'left'
        self._running = True
        self._send_stop()
        self._notification = {'level': 'info', 'msg': 'Returning home — preparing...'}
        self._enter_state(NavState.PREPARING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        logger.info("RETURN HOME started (%d waypoints, reversed)", len(reversed_wps))
        return True

    def set_waypoints(self, waypoints: List[Dict[str, Any]]):
        """Load waypoints from dashboard.  Each: {latitude, longitude, id, description}."""
        with self._lock:
            self._waypoints = list(waypoints)
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._heading_lock_active = False
            self._heading_lock_bearing = None
            self._heading_lock_home = None
            self._heading_lock_outbound_bearing = None
        logger.info("Loaded %d waypoints", len(waypoints))

    def start_heading_lock_target(self, latitude: float, longitude: float):
        """Navigate to a single target while preserving the current heading.

        This supports the dashboard's manual "forward-to-coordinate" mode:
        the operator physically aligns the robot first, then the Pi keeps
        re-acquiring that same heading after obstacle avoidance.
        """
        heading = self._read_heading()
        if heading is None:
            heading = self._current_heading

        try:
            lat = float(latitude)
            lon = float(longitude)
        except (TypeError, ValueError):
            logger.warning("Heading-lock target rejected — invalid coordinates")
            return False

        home_wp = None
        gps = self._get_gps()
        if gps and gps.get('latitude') is not None and gps.get('longitude') is not None:
            try:
                home_wp = {
                    'latitude': float(gps['latitude']),
                    'longitude': float(gps['longitude']),
                    'id': 'heading_lock_home',
                    'description': 'Heading-lock departure point',
                }
            except (TypeError, ValueError):
                home_wp = None

        outbound_bearing = float(heading or 0.0) % 360.0

        with self._lock:
            self._waypoints = [{
                'latitude': lat,
                'longitude': lon,
                'id': 1,
                'description': 'Heading-lock target',
            }]
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._avoid_attempts = 0
            self._avoid_direction = 'left'
            self._heading_lock_active = True
            self._heading_lock_bearing = outbound_bearing
            self._heading_lock_outbound_bearing = outbound_bearing
            self._heading_lock_home = home_wp
            self._acquire_timeout_retries = 0

        self._running = True
        self._send_stop()
        self._notification = {
            'level': 'info',
            'msg': f'Heading-lock target started — holding {self._heading_lock_bearing:.1f}°'
                   + (' (home saved)' if home_wp else ' (home unavailable)'),
        }
        self._enter_state(NavState.NAVIGATING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        logger.info("Heading-lock navigation started to (%.6f, %.6f) on %.1f°",
                    lat, lon, self._heading_lock_bearing)
        return True

    def return_to_heading_lock_home(self):
        """Return to the saved departure point for the current heading-lock run."""
        with self._lock:
            if not self._heading_lock_home:
                logger.warning("Cannot return to heading-lock home — no saved departure point")
                return False
            home_wp = dict(self._heading_lock_home)
            outbound_bearing = self._heading_lock_outbound_bearing
            return_bearing = ((outbound_bearing or 0.0) + 180.0) % 360.0 if outbound_bearing is not None else None

            self._waypoints = [home_wp]
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._avoid_attempts = 0
            self._avoid_direction = 'left'
            self._heading_lock_active = return_bearing is not None
            self._heading_lock_bearing = return_bearing
            self._acquire_timeout_retries = 0

        self._running = True
        self._send_stop()
        if return_bearing is not None:
            self._notification = {
                'level': 'info',
                'msg': f'Returning to saved departure point — acquiring {return_bearing:.1f}°',
            }
        else:
            self._notification = {
                'level': 'info',
                'msg': 'Returning to saved departure point — preparing...',
            }
        self._enter_state(NavState.PREPARING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        logger.info("Heading-lock return-home started to (%.6f, %.6f)%s",
                    home_wp['latitude'], home_wp['longitude'],
                    f" on {return_bearing:.1f}°" if return_bearing is not None else "")
        return True

    def set_drive_speed(self, speed: int) -> bool:
        self.DRIVE_SPEED = max(60, min(255, int(speed)))
        logger.info("Nav drive speed -> %d", self.DRIVE_SPEED)
        return True

    def set_adaptive_speed(self, enabled: bool) -> bool:
        self._adaptive_speed = bool(enabled)
        logger.info("Adaptive auto speed -> %s", self._adaptive_speed)
        return True

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
            self._heading_lock_active = False
            self._heading_lock_bearing = None
            self._heading_lock_home = None
            self._heading_lock_outbound_bearing = None
            self._acquire_timeout_retries = 0
        self._running = True
        self._send_stop()
        self._notification = {'level': 'info', 'msg': 'Navigation started — preparing...'}
        self._enter_state(NavState.PREPARING)

        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        logger.info("Navigation STARTED (%d waypoints)", len(self._waypoints))
        return True

    def stop(self):
        """Stop navigation completely."""
        self._running = False
        self._send_stop()
        self._heading_lock_active = False
        self._heading_lock_bearing = None
        self._acquire_timeout_retries = 0
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
        self._acquire_timeout_retries = 0
        self._send_stop()
        self._notification = {'level': 'info', 'msg': 'Navigation resumed — preparing...'}
        self._enter_state(NavState.PREPARING)
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
            if self._state not in (NavState.PREPARING, NavState.ACQUIRING_HEADING):
                logger.info("accept_heading ignored — state is %s", self._state.name)
                return False
        # Send stop multiple times to guarantee Mega processes it
        self._send_stop()
        self._send_stop()
        logger.info("Heading manually accepted (error=%.1f°) — motors stopped, %.0fs countdown",
                     self._last_heading_error, self.HEADING_HOLD_TIME)
        self._heading_acquired_emitted = False  # reset for _handle_heading_acquired
        self._enter_state(NavState.HEADING_ACQUIRED)
        return True

    def confirm_heading(self, accepted: bool = True):
        """Dashboard confirms or rejects the acquired heading.

        Called when the user presses Confirm or Reject on the heading
        confirmation dialog.  If accepted, the robot proceeds to
        NAVIGATING immediately.  If rejected, it re-enters ACQUIRING.
        """
        self._heading_confirm_result = accepted
        self._heading_confirmed.set()
        logger.info("Heading %s by user", "CONFIRMED" if accepted else "REJECTED")
        return True

    def set_event_callback(self, callback):
        """Register a callback fn(event_type: str, payload: dict)
        for sending events to the dashboard (e.g. heading_acquired)."""
        self._event_callback = callback

    def set_ai_override(self, enabled: bool):
        """Deprecated — kept as no-op for backward compatibility."""
        pass

    def receive_ai_advice(self, direction: str, safety: str, reason: str = ''):
        """Called by main.py when AI Vision provides obstacle avoidance advice.

        This unblocks _handle_obstacle_detected() which is waiting for
        AI advice before falling back to traditional avoidance."""
        self._ai_advice_direction = direction.upper() if direction else 'STOP'
        self._ai_advice_safety = safety.upper() if safety else 'DANGER'
        self._ai_advice_event.set()
        logger.info("AI advice received: %s (safety=%s) — %s",
                    self._ai_advice_direction, self._ai_advice_safety, reason)

    def notify_ai_triggered(self):
        """Called by main.py when AI analysis has been triggered for
        the current obstacle.  NavController will wait up to
        AI_ADVICE_TIMEOUT for the result."""
        self._ai_analysis_triggered = True

    def handle_ai_proactive_stop(self, direction: str, safety: str, reason: str = ''):
        """Called by main.py when AI Vision detects DANGER / requests STOP
        while NavController is in NAVIGATING state (no ultrasonic obstacle yet).

        This allows the AI camera to proactively halt the robot before
        the ultrasonic sensor detects the obstacle — e.g. drop-offs, glass
        walls, or objects the ultrasonic cone misses.
        """
        logger.warning("AI proactive stop during NAVIGATING — %s %s: %s",
                        direction, safety, reason)
        self._send_stop()
        # Pre-load AI advice so _handle_obstacle_detected sees it immediately
        self._ai_advice_direction = direction.upper() if direction else 'STOP'
        self._ai_advice_safety = safety.upper() if safety else 'DANGER'
        self._ai_analysis_triggered = True
        self._ai_advice_event.set()
        # Transition into obstacle-detected so the normal handler takes over
        self._enter_obstacle_detected(skip_clearance=True)

    def _emit_event(self, event_type: str, payload: dict = None):
        """Fire an event to the dashboard via the registered callback."""
        if self._event_callback:
            try:
                self._event_callback(event_type, payload or {})
            except Exception as e:
                logger.debug("Event callback error: %s", e)

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

        # Read sensors every tick. Use a light circular low-pass filter so
        # compass jitter does not cause left/right dithering during acquire.
        raw_heading = self._read_heading()
        if self._filtered_heading is None:
            self._filtered_heading = raw_heading
        else:
            self._filtered_heading = self._blend_angle(
                self._filtered_heading,
                raw_heading,
                self.HEADING_FILTER_ALPHA,
            )
        self._current_heading = self._filtered_heading

        # PREPARING state: motors stopped, waiting for timer — no GPS needed
        if state == NavState.PREPARING:
            self._handle_preparing()
            return

        gps = self._get_gps()
        if not gps:
            # Surface the GPS wait to the user — this is commonly why
            # ACQUIRING_HEADING appears stuck with no motor activity.
            if not getattr(self, '_gps_wait_logged', False):
                self._gps_wait_logged = True
                self._notification = {
                    'level': 'warning',
                    'msg': 'Waiting for GPS fix — motors paused until position is known',
                }
                logger.warning("Nav tick: no GPS fix — waiting (state=%s)", state.name)
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

        self._gps_wait_logged = False  # GPS is available again
        self._distance_to_wp = self._haversine(cur_lat, cur_lon, wp_lat, wp_lon)
        if self._heading_lock_active and self._heading_lock_bearing is not None:
            live_target_bearing = self._heading_lock_bearing
        else:
            live_target_bearing = self._bearing(cur_lat, cur_lon, wp_lat, wp_lon)

        self._live_target_bearing = live_target_bearing
        self._record_bearing_sample(live_target_bearing)

        if state in (NavState.ACQUIRING_HEADING, NavState.HEADING_ACQUIRED) and self._acquire_target_bearing is None:
            self._acquire_target_bearing = self._lock_acquire_target_bearing()

        if state in (NavState.ACQUIRING_HEADING, NavState.HEADING_ACQUIRED) and self._acquire_target_bearing is not None:
            self._target_bearing = self._acquire_target_bearing
        else:
            self._target_bearing = live_target_bearing

        self._last_heading_error = self._normalize_error(self._target_bearing - self._current_heading)

        # Debug log bearing calculation periodically (every ~2s at 10Hz)
        if int(time.time()) % 2 == 0 and int(time.time() * self.NAV_LOOP_HZ) % self.NAV_LOOP_HZ == 0:
            logger.debug("NAV bearing: cur=(%.6f,%.6f) wp=(%.6f,%.6f) → bearing=%.1f° heading=%.1f° error=%.1f° dist=%.1fm",
                         cur_lat, cur_lon, wp_lat, wp_lon,
                         self._target_bearing, self._current_heading,
                         self._last_heading_error, self._distance_to_wp)

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

    def _handle_preparing(self):
        """3-second hold before heading acquisition.

        Motors stay stopped.  Dashboard shows 'PREPARING' with countdown.
        User can click Accept Heading during this window if already aligned.
        """
        try:
            gps = self._get_gps()
        except Exception:
            gps = None

        if gps:
            with self._lock:
                wp = self._waypoints[self._current_wp_index] if self._current_wp_index < len(self._waypoints) else None
            if wp:
                try:
                    wp_lat = float(wp['latitude'])
                    wp_lon = float(wp['longitude'])
                    cur_lat = float(gps['latitude'])
                    cur_lon = float(gps['longitude'])
                    self._distance_to_wp = self._haversine(cur_lat, cur_lon, wp_lat, wp_lon)
                    if self._heading_lock_active and self._heading_lock_bearing is not None:
                        live_target_bearing = self._heading_lock_bearing
                    else:
                        live_target_bearing = self._bearing(cur_lat, cur_lon, wp_lat, wp_lon)
                    self._live_target_bearing = live_target_bearing
                    self._target_bearing = live_target_bearing
                    self._record_bearing_sample(live_target_bearing)
                    self._last_heading_error = self._normalize_error(self._target_bearing - self._current_heading)
                except Exception:
                    pass

        elapsed = time.time() - self._state_entry_time
        # Reinforce motor stop every second
        if int(elapsed) != getattr(self, '_prep_last_sec', -1):
            self._prep_last_sec = int(elapsed)
            self._send_stop()
        if elapsed >= self.PREPARE_TIME:
            logger.info("Prepare phase complete — acquiring heading")
            self._enter_state(NavState.ACQUIRING_HEADING)

    def _handle_acquiring_heading(self):
        """Rotate robot until heading matches target bearing (within deadband)."""
        error = self._last_heading_error
        abs_error = abs(error)
        now = time.time()

        # Timeout check
        elapsed = now - self._state_entry_time
        if elapsed > self.HEADING_ACQUIRE_TIMEOUT:
            self._send_stop()
            self._acquire_timeout_retries += 1
            if self._acquire_timeout_retries <= self.ACQUIRE_TIMEOUT_RETRIES:
                self._notification = {
                    'level': 'warning',
                    'msg': f'Heading acquire timeout — retrying ({self._acquire_timeout_retries}/{self.ACQUIRE_TIMEOUT_RETRIES})',
                }
                logger.warning("Heading acquire timeout — retrying (%d/%d)",
                               self._acquire_timeout_retries,
                               self.ACQUIRE_TIMEOUT_RETRIES)
                self._enter_state(NavState.PREPARING)
            else:
                self._notification = {
                    'level': 'error',
                    'msg': 'Heading acquire failed repeatedly — navigation paused for manual review',
                }
                logger.error("Heading acquire failed repeatedly — pausing navigation for manual review")
                self.pause()
            return

        if abs_error <= self.HEADING_DEADBAND:
            self._send_stop()
            if self._acquire_deadband_since is None:
                self._acquire_deadband_since = now
                logger.debug("Heading inside deadband — settling (error=%.1f°)", abs_error)
                return
            if (now - self._acquire_deadband_since) >= self.ACQUIRE_SETTLE_TIME:
                self._send_stop()  # double-send for reliability
                self._heading_acquired_emitted = False  # reset for _handle_heading_acquired
                logger.info("Heading auto-acquired (error=%.1f°) — %.0fs countdown",
                             abs_error, self.HEADING_HOLD_TIME)
                self._enter_state(NavState.HEADING_ACQUIRED)
            return
        self._acquire_deadband_since = None

        desired_dir = 1 if error > 0 else -1
        if self._acquire_turn_direction == 0:
            self._acquire_turn_direction = desired_dir
            self._acquire_turn_since = now
        elif desired_dir != self._acquire_turn_direction:
            committed_long_enough = (now - self._acquire_turn_since) >= self.ACQUIRE_MIN_TURN_TIME
            if committed_long_enough and abs_error >= self.ACQUIRE_REVERSE_HYST:
                self._acquire_turn_direction = desired_dir
                self._acquire_turn_since = now
                logger.debug("Acquire turn reversed — error now %.1f°", error)

        speed = self._get_acquire_turn_speed(abs_error)
        if self._acquire_turn_direction > 0:
            self._send_rotate_right(speed)
        else:
            self._send_rotate_left(speed)

    def _handle_heading_acquired(self):
        """Hold state with countdown — motors stay stopped.

        Emits 'heading_acquired' event to dashboard for confirmation.
        Dashboard can confirm or reject.  Auto-accepts after
        HEADING_HOLD_TIME seconds (non-blocking via Event.wait with timeout).

        Dashboard shows 'Starting navigation in N…' countdown.
        After confirmation or timeout, transitions to NAVIGATING.
        """
        elapsed = time.time() - self._state_entry_time

        # On first entry, emit heading_acquired event and reset confirmation
        if not getattr(self, '_heading_acquired_emitted', False):
            self._heading_acquired_emitted = True
            self._heading_confirmed.clear()
            self._heading_confirm_result = None
            self._emit_event('heading_acquired', {
                'heading': round(self._current_heading, 1),
                'magnetic_heading': round(self._current_heading_magnetic, 1),
                'target_bearing': round(self._target_bearing, 1),
                'heading_error': round(self._last_heading_error, 1),
            })

        # Reinforce stop every ~1s (at 10Hz, every 10th tick) as safety
        tick_num = int(elapsed * self.NAV_LOOP_HZ)
        if tick_num % self.NAV_LOOP_HZ == 0:
            self._send_stop()

        # Check if user confirmed/rejected
        if self._heading_confirmed.is_set():
            self._heading_acquired_emitted = False
            if self._heading_confirm_result is False:
                # Rejected — re-acquire
                logger.info("Heading rejected by user — re-acquiring")
                self._enter_state(NavState.ACQUIRING_HEADING)
                return
            else:
                # Confirmed (or True by default)
                logger.info("Heading confirmed by user — starting NAVIGATING")
                self._enter_state(NavState.NAVIGATING)
                return

        # Auto-accept after HEADING_HOLD_TIME
        if elapsed >= self.HEADING_HOLD_TIME:
            self._heading_acquired_emitted = False
            logger.info("Countdown complete (auto-accept) — starting NAVIGATING")
            self._enter_state(NavState.NAVIGATING)

    def _handle_navigating(self):
        """Drive forward. Monitor: waypoint reached, obstacle, course drift."""
        # Check waypoint reached
        if self._distance_to_wp <= self.WAYPOINT_RADIUS:
            self._send_stop()
            self._send_stop()
            wp_num = self._current_wp_index + 1
            wp_total = len(self._waypoints)
            self._notification = {
                'level': 'success',
                'msg': f'Waypoint {wp_num}/{wp_total} reached! Holding...'
            }
            self._enter_state(NavState.WAYPOINT_REACHED)
            return

        # Check obstacle (poll Mega)
        obstacle = self._read_obstacle()
        if obstacle and obstacle.get('obstacle'):
            self._send_stop()
            # Clear previous AI advice results so we wait for fresh advice
            # for THIS obstacle.  But do NOT reset _ai_analysis_triggered —
            # obstacle_loop may have already called notify_ai_triggered()
            # for this same obstacle (race between the two polling loops).
            self._ai_advice_event.clear()
            self._ai_advice_direction = None
            self._ai_advice_safety = None
            self._enter_obstacle_detected(skip_clearance=False)
            return

        # Check course drift — if heading drifted too far, re-acquire
        # Grace period: skip drift check for first NAV_GRACE_PERIOD seconds
        # so the robot actually drives forward after heading countdown.
        nav_elapsed = time.time() - self._state_entry_time
        if nav_elapsed >= self.NAV_GRACE_PERIOD:
            if abs(self._last_heading_error) > self.COURSE_DRIFT_THRESH:
                logger.info("Course drift %.1f° — re-acquiring heading", self._last_heading_error)
                self._send_stop()
                self._enter_state(NavState.ACQUIRING_HEADING)
                return

        # All clear — drive forward
        self._send_forward(self._get_cruise_speed())

    def _handle_obstacle_detected(self):
        """Stop and wait for AI Vision advice on obstacle avoidance.

        Flow:
        1. Brief safety pause (OBSTACLE_CHECK_PAUSE = 0.3s)
        2. If AI advice arrives → execute it
        3. If AI was triggered but no advice within AI_ADVICE_TIMEOUT → fallback
        4. If AI was NOT triggered → immediate traditional avoidance
        """
        if self._obstacle_phase == 'safety':
            if self._run_safety_clearance():
                return
            self._obstacle_phase = 'decision'
            self._obstacle_phase_start = time.time()
            self._send_stop()
            return

        elapsed = time.time() - self._obstacle_phase_start

        # Brief safety pause before any action
        if elapsed < self.OBSTACLE_CHECK_PAUSE:
            self._send_stop()
            return

        # Check if AI advice has arrived
        if self._ai_advice_event.is_set():
            direction = self._ai_advice_direction or 'STOP'
            safety = self._ai_advice_safety or 'DANGER'
            logger.info("Executing AI advice: %s (safety=%s)", direction, safety)

            if direction == 'FORWARD' and safety in ('SAFE', 'CAUTION'):
                # AI says path is clear — re-acquire heading and continue
                logger.info("AI: path clear — re-acquiring heading")
                self._enter_state(NavState.ACQUIRING_HEADING)
                return

            if direction in ('LEFT', 'RIGHT') and safety != 'DANGER':
                # AI says turn to avoid — execute as obstacle avoidance
                self._avoid_attempts += 1
                self._avoid_direction = direction.lower()
                self._obstacle_phase = 'turn'
                self._obstacle_phase_start = time.time()
                self._enter_state(NavState.OBSTACLE_AVOID)
                return

            # AI says STOP or DANGER — fall through to traditional avoidance
            logger.info("AI: STOP/DANGER — traditional avoidance")
            self._enter_traditional_avoidance()
            return

        # AI analysis was triggered — wait up to AI_ADVICE_TIMEOUT
        if self._ai_analysis_triggered:
            if elapsed < self.AI_ADVICE_TIMEOUT:
                self._send_stop()  # keep motors stopped while waiting
                return  # still waiting for AI
            # Timeout — AI didn't respond in time
            logger.warning("AI advice timeout (%.1fs) — traditional avoidance",
                           self.AI_ADVICE_TIMEOUT)
            self._enter_traditional_avoidance()
            return

        # AI was NOT triggered (model not loaded/enabled) — immediate fallback
        self._enter_traditional_avoidance()

    def _enter_traditional_avoidance(self):
        """Enter traditional turn-away obstacle avoidance."""
        self._avoid_attempts += 1
        self._avoid_direction = 'left' if self._avoid_attempts % 2 == 1 else 'right'
        self._obstacle_phase = 'turn'
        self._obstacle_phase_start = time.time()
        self._enter_state(NavState.OBSTACLE_AVOID)

    def _handle_obstacle_avoid(self):
        """Multi-phase: safety reverse → turn away → drive past → recheck."""
        elapsed = time.time() - self._obstacle_phase_start

        if self._obstacle_phase == 'safety':
            if self._run_safety_clearance():
                return
            self._obstacle_phase = 'turn'
            self._obstacle_phase_start = time.time()
            return

        if self._obstacle_phase == 'turn':
            # Rotate away from obstacle
            if elapsed < self.OBSTACLE_TURN_TIME:
                if self._avoid_direction == 'left':
                    self._send_rotate_left(self._get_turn_speed(fast=True))
                else:
                    self._send_rotate_right(self._get_turn_speed(fast=True))
            else:
                # Turn complete — start moving past the obstacle
                self._send_stop()
                self._obstacle_phase = 'forward'
                self._obstacle_phase_start = time.time()

        elif self._obstacle_phase == 'forward':
            obstacle = self._read_obstacle()
            dist_cm = self._extract_obstacle_distance(obstacle)
            if 0 < dist_cm < self.SAFE_AVOID_DISTANCE_CM:
                logger.info("Avoid-forward interrupted at %d cm — backing to safe distance", dist_cm)
                self._send_stop()
                self._obstacle_phase = 'safety'
                self._obstacle_phase_start = time.time()
                return

            if elapsed < self.OBSTACLE_PASS_TIME:
                self._send_forward(self._get_avoid_forward_speed())
            else:
                self._send_stop()
                self._obstacle_phase = 'recheck'
                self._obstacle_phase_start = time.time()

        elif self._obstacle_phase == 'recheck':
            if elapsed >= self.OBSTACLE_CHECK_PAUSE:
                obstacle = self._read_obstacle()
                dist_cm = self._extract_obstacle_distance(obstacle)
                if obstacle and obstacle.get('obstacle'):
                    if 0 < dist_cm < self.SAFE_AVOID_DISTANCE_CM:
                        self._obstacle_phase = 'safety'
                    else:
                        self._obstacle_phase = 'turn'
                    self._obstacle_phase_start = time.time()
                    if self._avoid_attempts > 5:
                        logger.warning("Too many obstacle attempts — skipping waypoint")
                        self._advance_waypoint()
                else:
                    # Clear — pause before re-acquiring heading
                    logger.info("Obstacle cleared — preparing before re-acquire")
                    self._send_stop()
                    self._notification = {'level': 'info', 'msg': 'Obstacle cleared — preparing...'}
                    self._enter_state(NavState.PREPARING)

    def _handle_waypoint_reached(self):
        """Hold for WAYPOINT_HOLD_TIME, show notification, then advance."""
        elapsed = time.time() - self._state_entry_time
        # Send stop reinforcement
        if int(elapsed) != getattr(self, '_wr_last_sec', -1):
            self._wr_last_sec = int(elapsed)
            self._send_stop()
        if elapsed >= self.WAYPOINT_HOLD_TIME:
            self._advance_waypoint()

    # ================================================================
    #  MOTOR COMMANDS (sent to Mega via I2C)
    # ================================================================

    def _send_forward(self, speed: int):
        """Drive both motors forward at `speed` PWM."""
        left = speed + self._motor_trim_left
        right = speed + self._motor_trim_right
        self._send_motor_command(left, right)

    def _send_reverse(self, speed: int):
        """Reverse both motors slowly to regain safe obstacle clearance."""
        left = -(speed + self._motor_trim_left)
        right = -(speed + self._motor_trim_right)
        self._send_motor_command(left, right)

    def _send_rotate_left(self, speed: int):
        """Rotate left: left motor backward, right motor forward."""
        if self._invert_rotation_direction:
            self._send_motor_command(speed, -speed)
        else:
            self._send_motor_command(-speed, speed)

    def _send_rotate_right(self, speed: int):
        """Rotate right: left motor forward, right motor backward."""
        if self._invert_rotation_direction:
            self._send_motor_command(-speed, speed)
        else:
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
        """Read compass heading from Pi-local compass (true/corrected)."""
        try:
            # Also cache the raw magnetic heading for the dashboard
            if hasattr(self.compass, 'read_heading_magnetic'):
                self._current_heading_magnetic = self.compass.read_heading_magnetic()
            return self.compass.read_heading()
        except Exception:
            return self._current_heading  # return last known

    def _get_gps(self) -> Optional[dict]:
        """Get current GPS position.

        Primary: SIM7600E on Pi (via gps_provider callback).
        Fallback: Neo-6M on Mega (via mega_gps_provider callback) if primary
        returns None — also updates _neo_satellites.
        """
        # Primary GPS
        try:
            if callable(self._gps_provider):
                result = self._gps_provider()
            elif hasattr(self._gps_provider, 'get_position'):
                result = self._gps_provider.get_position()
            else:
                result = None
        except Exception:
            result = None

        if result:
            return result

        # Fallback: poll Mega Neo-6M
        if self._mega_gps_provider:
            try:
                mega = self._mega_gps_provider()
                if mega and mega.get('valid') and mega.get('latitude') and mega.get('longitude'):
                    self._neo_satellites = mega.get('satellites', 0)
                    logger.debug("Using Neo-6M GPS fallback (sats=%d)", self._neo_satellites)
                    return {
                        'latitude': mega['latitude'],
                        'longitude': mega['longitude'],
                        'speed': mega.get('speed', 0),
                    }
                elif mega:
                    self._neo_satellites = mega.get('satellites', 0)
            except Exception:
                pass

        return None

    def _check_obstacle(self) -> bool:
        """Poll Mega for obstacle status."""
        obstacle = self._read_obstacle()
        return bool(obstacle and obstacle.get('obstacle'))

    def _read_obstacle(self) -> Optional[dict]:
        """Poll Mega for obstacle flag + distance and cache the latest reading."""
        try:
            if self.robot_link and hasattr(self.robot_link, 'request_obstacle_status'):
                result = self.robot_link.request_obstacle_status()
                if result:
                    dist_cm = self._extract_obstacle_distance(result)
                    if dist_cm > 0:
                        self._last_obstacle_distance_cm = dist_cm
                    return result
        except Exception:
            pass
        return None

    def _extract_obstacle_distance(self, obstacle: Optional[dict]) -> int:
        if not obstacle:
            return -1
        try:
            dist_cm = int(obstacle.get('distance_cm', -1))
        except (TypeError, ValueError):
            dist_cm = -1
        return dist_cm if dist_cm > 0 else -1

    def _get_cruise_speed(self) -> int:
        base = max(60, min(255, int(self.DRIVE_SPEED)))
        if not self._adaptive_speed:
            return base
        if self._distance_to_wp <= max(self.WAYPOINT_RADIUS + 1.0, 5.0):
            return max(90, min(base, base - 10))
        return max(95, min(170, base))

    def _get_avoid_forward_speed(self) -> int:
        base = int(self.DRIVE_SPEED)
        if not self._adaptive_speed:
            return max(85, min(base, 140))
        return max(90, min(135, base - 10))

    def _get_reverse_speed(self) -> int:
        base = int(self.DRIVE_SPEED)
        if not self._adaptive_speed:
            return 90
        return max(80, min(110, base - 25))

    def _get_turn_speed(self, fast: bool) -> int:
        if not self._adaptive_speed:
            return self.ROTATION_SPEED_FAST if fast else self.ROTATION_SPEED_SLOW
        base = int(self.DRIVE_SPEED)
        if fast:
            return max(140, min(180, base + 20))
        return max(110, min(140, base))

    def _get_acquire_turn_speed(self, abs_error: float) -> int:
        """Gentler turn profile for heading acquisition to reduce overshoot."""
        if abs_error >= 45.0:
            return self._get_turn_speed(fast=True)
        if abs_error >= self.ACQUIRE_SLOW_THRESH:
            if not self._adaptive_speed:
                return max(105, min(self.ROTATION_SPEED_FAST - 20, 130))
            base = int(self.DRIVE_SPEED)
            return max(115, min(145, base + 5))
        if not self._adaptive_speed:
            return max(85, min(self.ROTATION_SPEED_SLOW, 100))
        base = int(self.DRIVE_SPEED)
        return max(95, min(120, base - 5))

    def _run_safety_clearance(self) -> bool:
        """Reverse slowly until the obstacle is at a safer distance."""
        obstacle = self._read_obstacle()
        dist_cm = self._extract_obstacle_distance(obstacle)
        elapsed = time.time() - self._obstacle_phase_start

        if 0 < dist_cm < self.SAFE_AVOID_DISTANCE_CM and elapsed < self.SAFE_REVERSE_TIMEOUT:
            self._send_reverse(self._get_reverse_speed())
            if self._clearance_notice_phase != self._obstacle_phase_start:
                self._notification = {
                    'level': 'warning',
                    'msg': f'Obstacle too close ({dist_cm} cm) — backing to {self.SAFE_AVOID_DISTANCE_CM} cm',
                }
                self._clearance_notice_phase = self._obstacle_phase_start
            return True

        if 0 < dist_cm < self.SAFE_AVOID_DISTANCE_CM and elapsed >= self.SAFE_REVERSE_TIMEOUT:
            logger.warning("Safe reverse timeout at %d cm — continuing with stop", dist_cm)

        self._send_stop()
        return False

    def _enter_obstacle_detected(self, skip_clearance: bool = False):
        self._obstacle_phase = 'decision' if skip_clearance else 'safety'
        self._obstacle_phase_start = time.time()
        self._enter_state(NavState.OBSTACLE_DETECTED)

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
        if new_state == NavState.ACQUIRING_HEADING:
            self._acquire_deadband_since = None
            self._acquire_turn_direction = 0
            self._acquire_turn_since = 0.0
            allow_cached_target = old != NavState.PREPARING or bool(self._bearing_samples) or self._live_target_bearing is not None
            self._acquire_target_bearing = self._lock_acquire_target_bearing(
                allow_cached_target=allow_cached_target
            )
            if self._acquire_target_bearing is not None:
                self._target_bearing = self._acquire_target_bearing
                self._last_heading_error = self._normalize_error(
                    self._target_bearing - self._current_heading
                )
        elif new_state == NavState.HEADING_ACQUIRED:
            self._acquire_deadband_since = None
            self._acquire_turn_direction = 0
            self._acquire_turn_since = 0.0
        elif new_state == NavState.PREPARING:
            self._acquire_target_bearing = None
            self._acquire_deadband_since = None
            self._acquire_turn_direction = 0
            self._acquire_turn_since = 0.0
            self._bearing_samples = []
            self._live_target_bearing = None
        elif new_state not in (NavState.ACQUIRING_HEADING, NavState.HEADING_ACQUIRED):
            self._acquire_target_bearing = None
            self._acquire_deadband_since = None
            self._acquire_turn_direction = 0
            self._acquire_turn_since = 0.0
        if new_state in (NavState.NAVIGATING, NavState.WAYPOINT_REACHED, NavState.COMPLETE):
            self._acquire_timeout_retries = 0
        # When leaving obstacle handling, reset AI advice flags so the
        # next obstacle encounter starts with a clean slate.
        if new_state not in (NavState.OBSTACLE_DETECTED, NavState.OBSTACLE_AVOID):
            self._ai_analysis_triggered = False

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
                self._heading_lock_active = False
                self._heading_lock_bearing = None
                self._notification = {
                    'level': 'success',
                    'msg': 'All waypoints complete! Navigation finished.'
                }
                self._enter_state(NavState.COMPLETE)
                self._running = False
                logger.info("ALL WAYPOINTS COMPLETE!")
                return

        # Reset per-waypoint counters
        self._avoid_attempts = 0
        self._send_stop()
        self._notification = {
            'level': 'info',
            'msg': f'Waypoint {self._current_wp_index}/{len(self._waypoints)} — preparing next...'
        }
        self._enter_state(NavState.PREPARING)

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Distance in meters between two GPS coordinates.

        Primary:  Karney's geodesic on WGS-84 ellipsoid (< 0.5 mm error).
        Fallback: Haversine on sphere (if geographiclib unavailable).
        """
        if _HAS_GEOGRAPHICLIB:
            result = _geod.Inverse(lat1, lon1, lat2, lon2)
            return result['s12']  # distance in metres
        # Spherical fallback
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Initial bearing (azimuth) in degrees (0–360) from point 1 to point 2.

        Primary:  Karney's geodesic on WGS-84 ellipsoid (< 0.5 mm error).
        Fallback: Spherical great-circle bearing (if geographiclib unavailable).
        """
        if _HAS_GEOGRAPHICLIB:
            result = _geod.Inverse(lat1, lon1, lat2, lon2)
            azi = result['azi1']  # initial azimuth, -180..+180
            return azi % 360.0
        # Spherical fallback
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

    @classmethod
    def _blend_angle(cls, previous: float, current: float, alpha: float) -> float:
        """Circular low-pass blend between two headings."""
        if previous is None:
            return current
        alpha = max(0.0, min(1.0, float(alpha)))
        delta = cls._normalize_error(current - previous)
        return (previous + alpha * delta) % 360.0

    @classmethod
    def _circular_mean_deg(cls, values: List[float]) -> Optional[float]:
        """Mean of compass bearings in degrees, accounting for wraparound."""
        if not values:
            return None
        sin_sum = 0.0
        cos_sum = 0.0
        for deg in values:
            rad = math.radians(deg)
            sin_sum += math.sin(rad)
            cos_sum += math.cos(rad)
        if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
            return values[-1] % 360.0
        return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0

    def _record_bearing_sample(self, bearing: float) -> None:
        if bearing is None:
            return
        self._bearing_samples.append(float(bearing) % 360.0)
        if len(self._bearing_samples) > self.ACQUIRE_BEARING_SAMPLES:
            self._bearing_samples.pop(0)

    def _lock_acquire_target_bearing(self, allow_cached_target: bool = True) -> Optional[float]:
        """Freeze a stable target bearing for acquire/countdown phases."""
        if self._heading_lock_active and self._heading_lock_bearing is not None:
            latched = float(self._heading_lock_bearing) % 360.0
        else:
            latched = self._circular_mean_deg(self._bearing_samples)
            if latched is None and self._live_target_bearing is not None:
                latched = float(self._live_target_bearing) % 360.0
            elif latched is None and allow_cached_target and self._target_bearing is not None:
                latched = float(self._target_bearing) % 360.0
        if latched is None:
            return None
        logger.info("Acquire bearing locked at %.1f°", latched)
        return latched
