"""
Main Control Loop for Raspberry Pi
Environmental Monitoring Robot
"""

import time
import json
import logging
import sys
import os
import signal
from datetime import datetime
from threading import Thread, Event
from threading import Lock
from typing import Optional

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import custom modules
from raspberry_pi.sensors.sensor_manager import SensorManager
from raspberry_pi.sensors.compass import Compass
from raspberry_pi.communication.gsm_module import GSMModule
from raspberry_pi.communication.sim7600e_gps import SIM7600EGPS
from raspberry_pi.communication.serial_comm import ArduinoComm
from raspberry_pi.communication.i2c_comm import I2CComm
from raspberry_pi.communication.cloud_uploader import CloudUploader
from raspberry_pi.communication.api_client import DashboardAPI
from raspberry_pi.camera.camera_stream import CameraStream
from raspberry_pi.camera.mjpeg_server import MJPEGServer, update_frame as mjpeg_update_frame
from raspberry_pi.utils.logger import setup_logger
from raspberry_pi.utils.data_formatter import DataFormatter
from raspberry_pi.utils.qmi_health import get_monitor as get_qmi_monitor
from raspberry_pi.navigation.nav_controller import NavController

# Configuration
def _load_config() -> tuple[dict, str]:
    """Load robot configuration.

    Priority:
      1) $ROBOT_CONFIG_PATH (if set)
      2) config.json next to this file (raspberry_pi/config.json)
      3) ./config.json in current working directory
    """
    env_path = os.environ.get('ROBOT_CONFIG_PATH')
    candidates = []
    if env_path:
        candidates.append(env_path)
    candidates.append(os.path.join(os.path.dirname(__file__), 'config.json'))
    candidates.append(os.path.join(os.getcwd(), 'config.json'))

    for path in candidates:
        try:
            with open(path, 'r') as f:
                return json.load(f), path
        except FileNotFoundError:
            continue

    return {
            "device_id": "robot_01",
            "update_interval": 1,
            "gps_update_interval": 2,
            "status_update_interval": 10,
            "waypoint_check_interval": 5,
            "dashboard_api": {
                "base_url": "http://localhost:5000"
            },
            "gsm": {
                "port": "/dev/ttyUSB0",
                "baudrate": 9600,
                "apn": "your-apn",
                "module_type": "SIM7600E"
            },
            "sensors": {
                "dht": {
                    "pin": 4,
                    "type": "DHT22"
                },
                "mq_sensors": {
                    "mq7": {"channel": 1, "enabled": True},
                    "mq135": {"channel": 0, "enabled": True}
                },
                "adc": {
                    "type": "ADS1115",
                    "address": 72,
                    "gain": 1
                }
            },
            "camera": {
                "enabled": False,
                "resolution": [640, 480],
                "fps": 5,
                "quality": 75,
                # optional max frame bytes allowed for outbound frames (safety)
                "max_frame_size": 2 * 1024 * 1024
            },
            "log_level": "INFO",
            "i2c": {
                "bus": 1,
                "mega_address": 8
            }
        }, "<default>"


CONFIG, _CONFIG_PATH = _load_config()

# Setup logging
logger = setup_logger('main', CONFIG.get('log_level', 'INFO'))
logger.info("Loaded config from %s", _CONFIG_PATH)

# Global shutdown event
shutdown_event = Event()


class RobotController:
    """Main robot controller class"""
    
    def __init__(self):
        logger.info("Initializing Robot Controller...")
        
        self.device_id = CONFIG.get('device_id', 'robot_01')
        self.update_interval = CONFIG.get('update_interval', 5)
        self.gps_interval = CONFIG.get('gps_update_interval', 2)
        self.status_interval = CONFIG.get('status_update_interval', 10)
        self.waypoint_interval = CONFIG.get('waypoint_check_interval', 5)
        
        # Initialize components
        # Sensor Manager (optional)
        self.sensor_manager = None
        try:
            self.sensor_manager = SensorManager(CONFIG['sensors'])
            logger.info("Sensor Manager initialized")
        except Exception as e:
            logger.warning(f"Sensor Manager not available: {e}")
        
        # Compass (HMC5883L on I2C) — pass full config for calibration
        self.compass = None
        try:
            compass_cal = CONFIG.get('compass', {})
            decl = compass_cal.get('declination_deg', -0.5)
            self.compass = Compass(declination_deg=decl, config=CONFIG)
            logger.info("Compass initialized")
        except Exception as e:
            logger.warning(f"Compass not available: {e}")
        
        # GSM Module (optional - legacy) and/or SIM7600E (preferred)
        # Support older configs that put SIM7600E settings under "gsm" by using module_type.
        self.gsm = None
        self.sim7600e = None
        self.gps_from_gsm = False

        gsm_cfg = CONFIG.get('gsm') or {}
        sim_cfg = CONFIG.get('sim7600e')
        gsm_module_type = str(gsm_cfg.get('module_type', '')).strip().upper()

        # If SIM7600E is configured explicitly, use it.
        # If module_type indicates SIM7600E, treat the gsm config as sim7600e config.
        if not sim_cfg and gsm_cfg and gsm_module_type == 'SIM7600E':
            sim_cfg = gsm_cfg

        # SIM7600E with GPS (optional) - takes priority over Neo-6M
        if sim_cfg:
            try:
                self.sim7600e = SIM7600EGPS(sim_cfg)
                self.gps_from_gsm = True
                logger.info("SIM7600E module initialized")
            except Exception as e:
                logger.warning(f"SIM7600E module not available: {e}")

        # Legacy GSM module only if enabled and not SIM7600E
        try:
            gsm_enabled = bool(gsm_cfg.get('enabled', True))
            if gsm_enabled and gsm_cfg and not self.sim7600e:
                self.gsm = GSMModule(gsm_cfg)
                logger.info("GSM Module initialized (legacy)")
        except Exception as e:
            logger.warning(f"GSM Module not available: {e}")
        
        # Arduino Mega communication (optional)
        self.comm_mode = None
        self.robot_link = None

        # Latched manual driving (dashboard arrows)
        self._manual_drive_lock = Lock()
        self._manual_drive_active = False
        self._manual_drive_direction = 'stop'
        self._manual_drive_speed = 180
        self._manual_drive_thread = None
        self._analog_drive_lock = Lock()
        self._analog_drive_throttle = 0
        self._analog_drive_steer = 0
        self._analog_drive_active = False
        self._analog_drive_thread = None
        self._analog_drive_last_input_mono = 0.0
        self._analog_drive_last_send_mono = 0.0
        joystick_cfg = CONFIG.get('joystick', {})
        self._analog_drive_interval = max(0.03, float(joystick_cfg.get('stream_interval_s', 0.05)))
        self._analog_bus_priority_hold = max(
            self._analog_drive_interval * 2.0,
            float(joystick_cfg.get('bus_priority_hold_s', 0.35))
        )
        self._manual_assist_lock = Lock()
        self._manual_assist_active = False
        self._manual_assist_thread = None
        self._manual_assist_cancel = Event()
        self._manual_assist_resume_heading = None
        self._manual_assist_resume_speed = 180
        self._manual_assist_resume_direction = 'forward'
        self._manual_assist_last_turn = 'right'
        self.wireless_backup_active = False  # Track wireless backup control engagement
        try:
            if CONFIG.get('i2c'):
                self.robot_link = I2CComm(CONFIG['i2c'])
                self.comm_mode = 'i2c'
                if getattr(self.robot_link, 'bus', None):
                    logger.info("Configured I2C communication with Mega")
                else:
                    logger.warning("I2C communication not available (bus not initialized)")
            elif CONFIG.get('arduino'):
                self.robot_link = ArduinoComm(CONFIG['arduino'])
                self.comm_mode = 'serial'
                logger.info("Configured serial communication with Mega")
            else:
                logger.warning("No communication configuration found for Mega controller - running without navigation")
        except Exception as e:
            logger.warning(f"Arduino Mega communication not available: {e}")

        nav_cfg = CONFIG.get('navigation', {})
        self._invert_rotation_direction = bool(nav_cfg.get('invert_rotation_direction', False))
        
        # Dashboard API (required)
        try:
            self.api_client = DashboardAPI(CONFIG['dashboard_api'], self.device_id)
            # Wire QMI health into API client so HTTP successes/failures
            # update the network state tracker.
            from raspberry_pi.communication.api_client import set_qmi_health
            set_qmi_health(get_qmi_monitor())
            logger.info("Dashboard API Client initialized")
        except Exception as e:
            logger.error(f"Failed to initialize Dashboard API: {e}")
            raise
        
        # Cloud uploader (optional)
        try:
            self.cloud_uploader = CloudUploader(CONFIG.get('cloud_integrations'))
            logger.info("Cloud uploader configured")
        except Exception as e:
            logger.warning(f"Cloud uploader not available: {e}")
            self.cloud_uploader = None

        # Camera (optional)
        self.camera = None
        self.mjpeg_server = None
        if CONFIG.get('camera', {}).get('enabled', False):
            try:
                self.camera = CameraStream(CONFIG['camera'])
                if self.camera and self.camera.enabled:
                    logger.info("Camera Stream initialized")
                    # Start MJPEG direct-stream server so browsers can connect
                    # directly to the Pi for lowest-latency video.
                    mjpeg_port = int(CONFIG.get('camera', {}).get('mjpeg_port', 8081))
                    self.mjpeg_server = MJPEGServer(host='0.0.0.0', port=mjpeg_port)
                    self.mjpeg_server.start()
                else:
                    logger.warning("Camera Stream unavailable after init")
            except Exception as e:
                logger.warning(f"Camera not available: {e}")
        
        self.command_poll_interval = CONFIG.get('command_poll_interval', 2)

        # Pi-side Navigation Controller (replaces Mega-side nav)
        # GPS provider: returns latest SIM7600E position
        self._latest_gps = {}  # thread-safe via GIL for dict reads
        self._latest_gps_lock = Lock()
        self.nav_controller = None
        if self.compass and self.robot_link:
            try:
                self.nav_controller = NavController(
                    compass=self.compass,
                    robot_link=self.robot_link,
                    gps_provider=self._get_nav_gps,
                    config=CONFIG
                )
                # Wire up Neo-6M GPS fallback (polls Mega over I2C)
                if self.robot_link:
                    self.nav_controller._mega_gps_provider = self._get_mega_gps
                # Wire up event callback so NavController can emit heading_acquired etc.
                self.nav_controller.set_event_callback(self._nav_event_callback)
                logger.info("Pi-side NavController initialized")
            except Exception as e:
                logger.warning(f"NavController init failed: {e}")

        # QMI network health monitor — application-layer connectivity check
        qmi_cfg = CONFIG.get('qmi_health', {})
        self.qmi_health = get_qmi_monitor(enabled=qmi_cfg.get('enabled', True))

        # AI auto-drive: epoch counter to cancel stale delayed-resume threads.
        # Bumped every time the user explicitly sends NAV_PAUSE / NAV_STOP /
        # NAV_RESUME / NAV_START.  A delayed-resume thread captures the epoch
        # at spawn time and only acts if the epoch hasn't changed.
        self._ai_resume_epoch = 0

        # Thread management
        self.threads = []
        
    def start(self):
        """Start all robot systems"""
        logger.info("Starting Robot Controller...")
        
        try:
            # Connect SIM7600E (optional - for GPS + LTE)
            if self.sim7600e:
                try:
                    if self.sim7600e.connect():
                        logger.info("SIM7600E module connected")
                        try:
                            if self.sim7600e.check_internet():
                                logger.info("SIM7600E network registered (internet path available)")
                            else:
                                logger.warning("SIM7600E not registered / internet not ready")
                        except Exception as exc:
                            logger.debug("SIM7600E internet check failed: %s", exc)
                        # Wait for GPS lock
                        logger.info("Waiting for GPS lock...")
                        for attempt in range(5):
                            gps_data = self.sim7600e.get_gps_data()
                            if gps_data:
                                logger.info(f"GPS locked! Position: {gps_data['latitude']:.6f}, {gps_data['longitude']:.6f}")
                                break
                            time.sleep(3)
                    else:
                        logger.warning("SIM7600E not connected - GPS from Neo-6M only")
                except Exception as e:
                    logger.warning(f"SIM7600E initialization failed: {e} - using Neo-6M")
            
            # Connect GSM (optional - gracefully handle if hardware not available)
            if self.gsm and not self.sim7600e:
                try:
                    if self.gsm.connect():
                        logger.info("GSM module connected")
                        # Wait for internet connection
                        logger.info("Waiting for internet connection...")
                        for _ in range(3):  # Try 3 times, 5 seconds each
                            if self.gsm.check_internet():
                                logger.info("Internet connected via GSM!")
                                break
                            time.sleep(5)
                    else:
                        logger.warning("GSM module not connected - continuing without it")
                except Exception as e:
                    logger.warning(f"GSM initialization failed: {e} - continuing without it")
            
            # Start navigation communication link
            if self.robot_link:
                try:
                    if self.comm_mode == 'serial':
                        self.robot_link.start()
                    elif self.comm_mode == 'i2c':
                        # Only attempt I2C ping when the I2C bus is actually available.
                        if not getattr(self.robot_link, 'bus', None):
                            logger.warning("Skipping I2C ping (I2C bus not available)")
                        else:
                            # Mega may be busy in Arduino setup() at boot (e.g., buzzer tone),
                            # and it processes I2C commands in its main loop. Retry briefly.
                            ping_ok = False
                            for attempt in range(1, 11):  # ~5s total
                                if self.robot_link.ping():
                                    ping_ok = True
                                    break
                                time.sleep(0.5)
                            if not ping_ok:
                                logger.warning("Mega did not acknowledge I2C ping")
                except Exception as e:
                    logger.warning(f"Failed to start navigation link: {e}")
            
            # Start camera streaming
            if self.camera and self.camera.enabled:
                camera_thread = Thread(target=self.camera_loop, daemon=True)
                camera_thread.start()
                self.threads.append(camera_thread)
            
            # Start sensor reading loop
            if self.sensor_manager:
                sensor_thread = Thread(target=self.sensor_loop, daemon=True)
                sensor_thread.start()
                self.threads.append(sensor_thread)
            
            # Start GPS update loop
            gps_thread = Thread(target=self.gps_loop, daemon=True)
            gps_thread.start()
            self.threads.append(gps_thread)
            
            # Start status update loop
            status_thread = Thread(target=self.status_loop, daemon=True)
            status_thread.start()
            self.threads.append(status_thread)

            # Start obstacle alert loop (continuous warnings while obstacle present)
            if self.robot_link and hasattr(self.robot_link, 'request_obstacle_status'):
                obstacle_thread = Thread(target=self.obstacle_loop, daemon=True)
                obstacle_thread.start()
                self.threads.append(obstacle_thread)
            
            # Start command polling loop
            command_thread = Thread(target=self.command_loop, daemon=True)
            command_thread.start()
            self.threads.append(command_thread)

            # Start high-frequency instant command loop (manual drive fast-lane)
            instant_cmd_thread = Thread(target=self.instant_command_loop, daemon=True)
            instant_cmd_thread.start()
            self.threads.append(instant_cmd_thread)

            analog_drive_thread = Thread(target=self._analog_drive_loop, daemon=True)
            analog_drive_thread.start()
            self.threads.append(analog_drive_thread)

            # Start Pi-side navigation status broadcast loop (always runs —
            # sends full nav status, compass-only, or heartbeat depending on
            # which components are available)
            nav_status_thread = Thread(target=self.nav_status_loop, daemon=True)
            nav_status_thread.start()
            self.threads.append(nav_status_thread)

            # Start QMI network health monitor (background connectivity checks)
            self.qmi_health.start()
            
            logger.info("All systems started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start robot: {e}")
            return False
    
    def sensor_loop(self):
        """Read and transmit sensor data"""
        logger.info("Starting sensor loop...")
        
        while not shutdown_event.is_set():
            try:
                # Read sensors when available but do not skip the loop entirely if they are missing
                sensor_data = {}
                if self.sensor_manager:
                    try:
                        sensor_data = self.sensor_manager.read_all()
                    except Exception as sensor_err:
                        logger.debug(f"Sensor manager read failed: {sensor_err}")
                else:
                    logger.debug("Sensor manager not available; sending compass-only payload")
                
                # Read compass heading for dashboard data
                # (Heading is sent to Mega at 200ms rate by compass_heading_loop)
                if self.compass:
                    try:
                        heading = self.compass.read_heading()
                        sensor_data['heading'] = heading
                    except Exception as e:
                        logger.debug(f"Compass read failed: {e}")
                        sensor_data['heading'] = 0
                
                # Add timestamp and device ID
                sensor_data['timestamp'] = datetime.now().isoformat()
                sensor_data['device_id'] = self.device_id
                
                # Replace None values with 0 for numeric fields
                for key in ['temperature', 'humidity', 'mq2', 'mq7', 'mq135']:
                    if sensor_data.get(key) is None:
                        sensor_data[key] = 0
                
                # Send to dashboard
                success = self.api_client.send_sensor_data(sensor_data)
                
                if success:
                    temp = sensor_data.get('temperature', 0)
                    mq2 = sensor_data.get('mq2', 0)
                    mq7 = sensor_data.get('mq7', 0)
                    mq135 = sensor_data.get('mq135', 0)
                    logger.info(f"Sensor data sent - Temp: {temp:.1f}°C, MQ2: {mq2}, MQ7: {mq7}, MQ135: {mq135}")
                else:
                    logger.warning("Failed to send sensor data")

                if self.cloud_uploader:
                    try:
                        self.cloud_uploader.publish_sensor_data(sensor_data)
                    except Exception as exc:
                        logger.error(f"Error publishing to cloud services: {exc}")
                
            except Exception as e:
                logger.error(f"Error in sensor loop: {e}")
            
            # Wait before next reading
            shutdown_event.wait(self.update_interval)
    
    def _get_nav_gps(self) -> dict:
        """GPS provider for NavController — returns latest SIM7600E position."""
        with self._latest_gps_lock:
            data = self._latest_gps.copy()
        if data.get('latitude') and data.get('longitude'):
            return data
        return None

    def _get_mega_gps(self) -> dict:
        """Fallback GPS provider — polls Neo-6M via Mega I2C.

        Called by NavController when primary SIM7600E GPS has no fix.
        Returns dict with 'valid', 'latitude', 'longitude', 'speed',
        'satellites' or None on failure.
        """
        try:
            if self.robot_link and not self.wireless_backup_active:
                return self.robot_link.request_gps_data()
        except Exception as e:
            logger.debug("Mega GPS poll failed: %s", e)
        return None

    def _nav_event_callback(self, event_type: str, payload: dict):
        """Forward NavController events (e.g. heading_acquired) to the dashboard.

        Sends via HTTP POST to /api/event so it's broadcast over WebSocket.
        Non-blocking — runs in the nav thread context.
        """
        try:
            import requests as _req
            url = f"{self.dashboard_url}/api/event"
            event_data = {
                'type': event_type.upper(),
                'device_id': CONFIG.get('device_id', 'robot_01'),
                'timestamp': None,  # server will fill in
            }
            event_data.update(payload)
            _req.post(url, json=event_data, timeout=2)
            logger.debug("Nav event sent: %s", event_type)
        except Exception as e:
            logger.debug("Nav event send failed: %s", e)

    def nav_status_loop(self):
        """Broadcast Pi-side navigation status to dashboard.
        2 Hz when nav is active, 1 Hz when idle (still shows compass heading).
        Runs even without NavController — sends compass-only data if available."""
        logger.info("Starting nav status broadcast loop...")
        idle_counter = 0
        while not shutdown_event.is_set():
            try:
                status = None
                is_active = False

                if self.nav_controller:
                    # Full nav status from NavController
                    is_active = self.nav_controller.is_active
                    idle_counter += 1
                    if is_active or idle_counter >= 2:
                        idle_counter = 0
                        status = self.nav_controller.get_status()
                elif self.compass:
                    # No NavController but compass is available — send heading-only
                    idle_counter += 1
                    if idle_counter >= 2:
                        idle_counter = 0
                        heading = None
                        try:
                            heading = self.compass.read_heading()
                        except Exception:
                            pass
                        status = {
                            'state': 'IDLE',
                            'current_heading': round(heading, 1) if heading is not None else None,
                            'target_bearing': None,
                            'heading_error': None,
                            'distance_to_wp': None,
                            'current_wp_index': 0,
                            'total_waypoints': 0,
                        }
                else:
                    # Neither NavController nor compass — heartbeat only (every 5s)
                    idle_counter += 1
                    if idle_counter >= 10:
                        idle_counter = 0
                        status = {
                            'state': 'IDLE',
                            'current_heading': None,
                            'target_bearing': None,
                            'heading_error': None,
                            'distance_to_wp': None,
                            'current_wp_index': 0,
                            'total_waypoints': 0,
                        }

                if status is not None:
                    ts = datetime.utcnow().isoformat()
                    status['device_id'] = self.device_id
                    status['timestamp'] = ts
                    try:
                        self.api_client.send_event({
                            'device_id': self.device_id,
                            'type': 'NAV_STATUS',
                            'nav': status,
                            'timestamp': ts,
                        })
                    except Exception as exc:
                        logger.debug("Nav status broadcast failed: %s", exc)
            except Exception as e:
                logger.debug("Nav status loop error: %s", e)
            shutdown_event.wait(0.5)

    def gps_loop(self):
        """GPS loop: reads position and sends to dashboard.

        Primary:  SIM7600E GPS on Pi (high-accuracy, cellular module).
        Fallback: Neo-6M GPS via Mega I2C (when SIM7600E has no fix).

        Also stores the latest fix in _latest_gps for the Pi-side
        NavController, and forwards SIM7600E data to Mega as a seed.
        """
        logger.info("Starting GPS loop...")
        neo_consecutive_fails = 0  # throttle Neo-6M error logging

        while not shutdown_event.is_set():
            try:
                # Skip I2C GPS polling if wireless backup control is active
                if self.wireless_backup_active:
                    logger.debug("GPS loop paused (wireless backup active)")
                    shutdown_event.wait(self.gps_interval)
                    continue

                gps_data = None
                gps_source = None

                # ── Primary: SIM7600E GPS ─────────────────────────────
                if self.sim7600e:
                    try:
                        gps_data = self.sim7600e.get_gps_data()
                    except Exception as e:
                        logger.debug(f"SIM7600E GPS read failed: {e}")
                        gps_data = None

                    if gps_data and gps_data.get('latitude') is not None and gps_data.get('longitude') is not None:
                        gps_source = 'SIM7600E'
                        # Forward to Mega as seed for Neo-6M
                        if self.robot_link and not self._joystick_bus_priority_active():
                            try:
                                self.robot_link.send_gps_data(gps_data)
                                logger.debug("Forwarded SIM7600E GPS to Mega")
                            except Exception as e:
                                logger.debug(f"Could not forward GPS to Mega: {e}")
                    else:
                        gps_data = None  # ensure clean None for fallback

                # ── Fallback: Neo-6M via Mega I2C ─────────────────────
                if (
                    gps_data is None
                    and self.robot_link
                    and not self.wireless_backup_active
                    and not self._joystick_bus_priority_active()
                ):
                    try:
                        mega_gps = self.robot_link.request_gps_data()
                        if mega_gps and mega_gps.get('valid') and mega_gps.get('latitude') and mega_gps.get('longitude'):
                            # Neo-6M returns valid coordinates
                            gps_data = {
                                'latitude': float(mega_gps['latitude']),
                                'longitude': float(mega_gps['longitude']),
                                'altitude': 0.0,
                                'speed': float(mega_gps.get('speed', 0) or 0),
                                'heading': float(mega_gps.get('heading', 0) or 0),
                                'satellites': int(mega_gps.get('satellites', 0) or 0),
                                'source': 'Neo-6M',
                            }
                            gps_source = 'Neo-6M'
                            neo_consecutive_fails = 0
                            logger.debug("Neo-6M fallback GPS: %.6f, %.6f (sats=%d)",
                                         gps_data['latitude'], gps_data['longitude'],
                                         gps_data['satellites'])
                        else:
                            neo_consecutive_fails += 1
                            if neo_consecutive_fails <= 3 or neo_consecutive_fails % 30 == 0:
                                logger.debug("Neo-6M GPS: no valid fix (attempt %d)", neo_consecutive_fails)
                    except Exception as e:
                        neo_consecutive_fails += 1
                        if neo_consecutive_fails <= 3 or neo_consecutive_fails % 30 == 0:
                            logger.debug("Neo-6M GPS poll failed: %s", e)

                # ── Process valid GPS data ────────────────────────────
                if gps_data and gps_data.get('latitude') is not None and gps_data.get('longitude') is not None:
                    # Store for Pi-side nav controller (thread-safe)
                    with self._latest_gps_lock:
                        self._latest_gps = {
                            'latitude': float(gps_data['latitude']),
                            'longitude': float(gps_data['longitude']),
                            'speed': float(gps_data.get('speed', 0) or 0),
                        }

                    # Add compass heading (Pi-side) so the dashboard shows heading even when GPS is stationary.
                    if self.compass:
                        try:
                            gps_data['heading'] = float(self.compass.read_heading())
                        except Exception as e:
                            logger.debug(f"Compass read failed: {e}")
                            gps_data['heading'] = gps_data.get('heading', 0.0)

                    # Tag source for dashboard display
                    if gps_source:
                        gps_data['source'] = gps_source

                    # Add timestamp and device ID
                    gps_data['timestamp'] = datetime.now().isoformat()
                    gps_data['device_id'] = self.device_id

                    # Send to dashboard
                    success = self.api_client.send_gps_data(gps_data)

                    if success:
                        logger.debug("GPS data sent [%s]: %.6f, %.6f",
                                     gps_source, gps_data['latitude'], gps_data['longitude'])
                    else:
                        logger.warning("Failed to send GPS data")
                else:
                    logger.debug("No valid GPS data from any source")

            except Exception as e:
                logger.error(f"Error in GPS loop: {e}")

            # Wait before next request
            shutdown_event.wait(self.gps_interval)
    
    def status_loop(self):
        """Update robot status"""
        logger.info("Starting status loop...")
        
        while not shutdown_event.is_set():
            try:
                # Skip I2C status polling if wireless backup control is active
                if self.wireless_backup_active:
                    logger.debug("Status loop paused (wireless backup active)")
                    shutdown_event.wait(self.status_interval)
                    continue
                
                # Get system info
                import psutil
                
                status_data = {
                    'online': True,
                    'battery': self.get_battery_level(),
                    'signal_strength': self.gsm.get_signal_strength() if self.gsm else 0,
                    'network_health': self.qmi_health.stats,
                    'system_info': {
                        'cpu': psutil.cpu_percent(),
                        'memory': psutil.virtual_memory().percent,
                        'temperature': self.get_cpu_temperature()
                    },
                    'device_id': self.device_id
                }

                joystick_priority = self._joystick_bus_priority_active()
                if self.robot_link and not joystick_priority:
                    nav_status = self.robot_link.request_status()
                    if nav_status:
                        status_data['navigation'] = nav_status
                        
                        # Check for waypoint completion
                        if nav_status.get('waypoint_just_completed', False):
                            try:
                                # Get current waypoints to find the one that was completed
                                waypoints = self.api_client.get_waypoints()
                                if waypoints:
                                    # The current waypoint index tells us which waypoint was just completed
                                    current_idx = nav_status.get('current_waypoint_index', 0)
                                    if current_idx > 0 and current_idx <= len(waypoints):
                                        # Find the waypoint with sequence = current_idx
                                        completed_waypoint = None
                                        for wp in waypoints:
                                            if wp.get('sequence') == current_idx:
                                                completed_waypoint = wp
                                                break
                                        
                                        if completed_waypoint:
                                            self.api_client.complete_waypoint(completed_waypoint['id'])
                                            logger.info(f"Waypoint {completed_waypoint['id']} (sequence {current_idx}) marked as completed")
                            except Exception as exc:
                                logger.error(f"Failed to complete waypoint: {exc}")

                    # Obstacle events are handled in obstacle_loop() for higher frequency.

                    if self.comm_mode == 'i2c':
                        try:
                            self.robot_link.send_heartbeat()
                        except Exception as exc:
                            logger.debug(f"Heartbeat failed: {exc}")

                success = self.api_client.send_status(status_data)

                if success:
                    logger.info(f"Status sent successfully: Battery={status_data['battery']:.1f}%, Online={status_data['online']}")
                else:
                    logger.error(f"Failed to send status update to dashboard")

            except Exception as e:
                logger.error(f"Error in status loop: {e}")
                import traceback
                logger.error(f"Status loop traceback: {traceback.format_exc()}")

            # Wait before next update
            shutdown_event.wait(self.status_interval)

    def obstacle_loop(self):
        """Continuously poll obstacle status and alert dashboard when present."""
        logger.info("Starting obstacle alert loop...")

        # How often we poll the Mega for obstacle status (seconds)
        poll_interval = float(CONFIG.get('obstacle_poll_interval', 0.15))  # Faster: 150ms for real-time response
        # How often we emit OBSTACLE_DETECTED events while the obstacle persists (seconds)
        emit_interval = float(CONFIG.get('obstacle_emit_interval', 0.5))  # Faster alerts

        last_emit = 0.0
        last_state = False

        while not shutdown_event.is_set():
            try:
                if self._joystick_bus_priority_active():
                    last_state = False
                    shutdown_event.wait(max(poll_interval, 0.35))
                    continue

                obstacle = None
                if self.robot_link:
                    try:
                        obstacle = self.robot_link.request_obstacle_status()
                    except Exception:
                        obstacle = None

                now = time.time()
                state = bool(obstacle and obstacle.get('obstacle'))
                mega_planning = bool(obstacle and obstacle.get('planning'))
                mega_avoiding = bool(obstacle and obstacle.get('avoiding'))
                mega_blocked = bool(obstacle and obstacle.get('blocked'))

                # Emit on rising edge immediately, then periodically while active.
                should_emit = False
                if state and not last_state:
                    should_emit = True
                elif state and (now - last_emit) >= emit_interval:
                    should_emit = True

                if should_emit:
                    dist_cm = -1
                    if obstacle and obstacle.get('distance_cm') is not None:
                        try:
                            dist_cm = int(obstacle.get('distance_cm'))
                        except Exception:
                            dist_cm = -1

                    payload = {
                        'device_id': self.device_id,
                        'type': 'OBSTACLE_DETECTED',
                        'distance_cm': dist_cm,
                        'direction': 'FRONT',
                        'mega_planning': mega_planning,
                        'mega_avoiding': mega_avoiding,
                        'mega_blocked': mega_blocked,
                        'timestamp': datetime.utcnow().isoformat(),
                    }
                    try:
                        resp = self.api_client.send_event(payload)
                        last_emit = now
                        # Only tell NavController to wait for AI if
                        # the dashboard actually triggered AI analysis
                        if (resp and resp.get('ai_triggered')
                                and self.nav_controller
                                and self.nav_controller.is_active):
                            self.nav_controller.notify_ai_triggered()
                    except Exception as exc:
                        logger.debug(f"Failed to send obstacle event: {exc}")

                last_state = state

            except Exception as exc:
                logger.debug(f"Obstacle loop error: {exc}")

            shutdown_event.wait(poll_interval)
    
    def command_loop(self):
        """Poll dashboard for control commands (DB queue — non-latency-critical)"""
        logger.info("Starting command loop...")

        while not shutdown_event.is_set():
            try:
                commands = self.api_client.fetch_pending_commands()
                if commands:
                    for command in commands:
                        self._process_command(command)
            except Exception as exc:
                logger.error(f"Error in command loop: {exc}")

            shutdown_event.wait(self.command_poll_interval)

    def instant_command_loop(self):
        """High-frequency poll for ALL dashboard commands.
        Polls /api/commands/instant every ~100 ms.  The endpoint returns
        a list of queued commands (may be empty).  Each command is
        processed in order so nothing is lost."""
        logger.info("Starting instant command loop...")
        import requests as _requests
        base_url = (
            self.api_client.base_url.rstrip('/')
            if getattr(self.api_client, 'base_url', None)
            else CONFIG.get('dashboard_api', {}).get('base_url', '').rstrip('/')
        )
        url = f"{base_url}/api/commands/instant"
        session = _requests.Session()
        consecutive_errors = 0

        # Use the same timeout as the rest of the API (default 10s from config),
        # split into (connect, read).  GSM/cellular links need 2-5s just for
        # the TCP handshake, so the old 0.5s hard-coded value always timed out.
        api_timeout = CONFIG.get('dashboard_api', {}).get('timeout', 10)
        connect_timeout = min(api_timeout, 5)      # cap connect at 5s
        read_timeout    = min(api_timeout, 5)       # cap read at 5s
        poll_timeout    = (connect_timeout, read_timeout)

        # Poll interval: go faster while joystick analog control is active.
        poll_interval = float(CONFIG.get('instant_poll_interval', 0.08))
        fast_poll_interval = float(CONFIG.get('instant_poll_interval_fast', 0.05))

        logger.info("Instant command loop polling: %s  timeout=%s  interval=%.2fs fast=%.2fs",
                     url, poll_timeout, poll_interval, fast_poll_interval)

        while not shutdown_event.is_set():
            try:
                resp = session.get(url, timeout=poll_timeout)
                if resp.status_code == 200:
                    data = resp.json()
                    commands = data.get('commands', [])
                    # Backward compat: old single-slot format
                    if not commands and data.get('command'):
                        cmd = data['command']
                        commands = [cmd] if cmd else []
                    for cmd in commands:
                        cmd_type = cmd.get('command_type', '?')
                        logger.info("Instant command received: %s (seq=%s)",
                                    cmd_type, cmd.get('seq', '?'))
                        self._process_command({
                            'id': None,
                            'command_type': cmd_type,
                            'payload': cmd.get('payload', {}),
                        })
                    if consecutive_errors > 0:
                        logger.info("Instant command loop reconnected after %d errors", consecutive_errors)
                    consecutive_errors = 0
                else:
                    consecutive_errors += 1
                    if consecutive_errors <= 3 or consecutive_errors % 30 == 0:
                        logger.warning("Instant command poll HTTP %d (error #%d) url=%s",
                                       resp.status_code, consecutive_errors, url)
            except Exception as exc:
                consecutive_errors += 1
                if consecutive_errors <= 3 or consecutive_errors % 30 == 0:
                    logger.warning("Instant command poll error #%d: %s (url=%s)",
                                   consecutive_errors, exc, url)

            interval = fast_poll_interval if self._joystick_bus_priority_active() else poll_interval
            shutdown_event.wait(interval)

    def _process_command(self, command: dict) -> None:
        command_id = command.get('id')
        command_type = command.get('command_type')
        payload = command.get('payload') or {}
        if isinstance(payload, str):
            try:
                payload = json.loads(payload)
            except json.JSONDecodeError:
                payload = {'raw': payload}

        logger.info(f"Processing command #{command_id}: {command_type}")

        success = False
        error_message = None

        try:
            if command_type == 'PING':
                success = self.robot_link.ping() if self.robot_link else False
            elif command_type == 'NAV_START':
                # Pi-side navigation: start nav controller (NOT Mega)
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                if self.nav_controller:
                    self._handoff_to_navigation()
                    success = self.nav_controller.start()
                else:
                    logger.warning("NavController not available")
                    success = False
            elif command_type == 'NAV_PAUSE':
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                if self.nav_controller:
                    self.nav_controller.pause()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_RESUME':
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                if self.nav_controller:
                    self._handoff_to_navigation()
                    self.nav_controller.resume()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_STOP':
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                if self.nav_controller:
                    self.nav_controller.stop()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_ACCEPT_HEADING':
                if self.nav_controller:
                    success = self.nav_controller.accept_heading()
                else:
                    success = False
            elif command_type == 'CONFIRM_HEADING':
                # Dashboard confirms or rejects the acquired heading.
                # payload: {accepted: true/false}
                accepted = bool(payload.get('accepted', True))
                if self.nav_controller:
                    success = self.nav_controller.confirm_heading(accepted)
                else:
                    success = False
            elif command_type == 'MANUAL_DRIVE':
                # RULE: Manual commands ALWAYS take priority over everything.
                # If NavController is actively driving, pause it so the user
                # gets immediate control.  They can resume nav later.
                direction = (payload or {}).get('direction', '').lower()
                raw_throttle = (payload or {}).get('throttle')
                raw_steer = (payload or {}).get('steer')
                analog_motion = False
                try:
                    analog_motion = abs(int(raw_throttle or 0)) > 0 or abs(int(raw_steer or 0)) > 0
                except (TypeError, ValueError):
                    analog_motion = False
                if direction not in ('', 'none') or analog_motion:
                    if self.nav_controller and self.nav_controller.is_active:
                        logger.info("Manual drive '%s' — pausing NavController (manual priority)",
                                    direction)
                        self.nav_controller.pause()
                success = self._handle_manual_drive(payload)
            elif command_type == 'MANUAL_OVERRIDE':
                success = self._handle_manual_override(payload)
            elif command_type == 'MANUAL_SPEED':
                success = self._handle_manual_speed(payload)
            elif command_type == 'AUTO_SPEED':
                success = self._handle_auto_speed(payload)
            elif command_type == 'AUTO_SPEED_MODE':
                success = self._handle_auto_speed_mode(payload)
            elif command_type == 'MANUAL_TARGET_START':
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                success = self._handle_manual_target_start(payload)
            elif command_type == 'MANUAL_TARGET_RETURN_HOME':
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                success = self._handle_manual_target_return_home(payload)
            elif command_type == 'ENGAGE_WIRELESS':
                engage = payload.get('engage', False)
                if self.robot_link and hasattr(self.robot_link, 'engage_wireless_control'):
                    success = self.robot_link.engage_wireless_control(engage)
                    if success:
                        self.wireless_backup_active = engage
                        logger.info(f"CC1101 backup control {'engaged - Pi I2C paused' if engage else 'disengaged - Pi I2C resumed'}")
                else:
                    success = False
                    logger.warning("Wireless control command not supported")
            elif command_type == 'NAV_RETURN_HOME':
                self._ai_resume_epoch += 1   # cancel stale AI resumes
                if self.nav_controller:
                    self._handoff_to_navigation()
                    success = self.nav_controller.return_home()
                else:
                    success = False
            elif command_type == 'WAYPOINT_PUSH':
                success = self._handle_waypoint_push()
            elif command_type == 'SOUND_BUZZER':
                duration = int((payload or {}).get('duration', 3))
                success = self.robot_link.sound_buzzer(duration) if hasattr(self.robot_link, 'sound_buzzer') else False
            elif command_type == 'AI_OVERRIDE':
                # DEPRECATED — AI obstacle avoidance is now automatic.
                # Kept for backward compatibility; no-op.
                logger.info("AI_OVERRIDE command received (deprecated, ignored)")
                success = True
            elif command_type == 'AI_DRIVE':
                # AI Vision navigation advice.
                # payload: {direction: FORWARD|LEFT|RIGHT|STOP,
                #           safety: SAFE|CAUTION|DANGER, reason: str}
                ai_dir = (payload.get('direction') or 'stop').upper()
                ai_safety = (payload.get('safety') or 'DANGER').upper()
                ai_reason = payload.get('reason', '')
                ai_source = payload.get('source', '')
                logger.info("AI_DRIVE: %s  safety=%s  reason=%s  source=%s",
                            ai_dir, ai_safety, ai_reason, ai_source)

                # If NavController is active, handle based on current nav state
                if (self.nav_controller
                        and self.nav_controller.is_active
                        and hasattr(self.nav_controller, 'receive_ai_advice')):

                    nav_state = getattr(self.nav_controller, '_state', None)
                    nav_state_name = nav_state.name if nav_state else ''

                    if nav_state_name == 'OBSTACLE_DETECTED':
                        # NavController is waiting for advice — deliver it
                        self.nav_controller.receive_ai_advice(ai_dir, ai_safety, ai_reason)
                    elif (ai_dir == 'STOP' or ai_safety == 'DANGER') and nav_state_name == 'NAVIGATING':
                        # AI sees danger during normal navigation — proactive stop
                        # Uses dedicated method that safely transitions NavController
                        self.nav_controller.handle_ai_proactive_stop(ai_dir, ai_safety, ai_reason)
                    elif ai_dir in ('LEFT', 'RIGHT') and ai_safety == 'CAUTION':
                        # AI sees partial obstruction — feed as advice for next obstacle check
                        self.nav_controller.receive_ai_advice(ai_dir, ai_safety, ai_reason)
                    else:
                        # FORWARD+SAFE/CAUTION — path clear, no action needed during normal nav
                        logger.debug("AI_DRIVE FORWARD+%s during %s — no action needed",
                                     ai_safety, nav_state_name)
                    success = True
                else:
                    # Direct AI driving — execute using the current autonomous speed
                    ai_speed = self._get_ai_drive_speed(payload)
                    if ai_dir == 'STOP':
                        success = self._handle_manual_drive({'direction': 'stop', 'speed': ai_speed})
                    elif ai_dir == 'FORWARD':
                        success = self._handle_manual_drive({'direction': 'forward', 'speed': ai_speed})
                    elif ai_dir == 'LEFT':
                        success = self._handle_manual_drive({'direction': 'left', 'speed': ai_speed})
                    elif ai_dir == 'RIGHT':
                        success = self._handle_manual_drive({'direction': 'right', 'speed': ai_speed})
                    else:
                        logger.warning("Unknown AI_DRIVE direction: %s", ai_dir)
                        success = False
            # FOLLOW_LINE removed — no line follower hardware
            else:
                logger.warning(f"Unknown command: {command_type}")
                error_message = f"Unknown command {command_type}"

            status = 'completed' if success else 'failed'
            if not success and not error_message:
                error_message = 'Hardware command failed'

        except Exception as exc:
            logger.exception(f"Command execution error: {exc}")
            success = False
            error_message = str(exc)
            status = 'failed'

        finally:
            if command_id is not None:
                self.api_client.ack_command(command_id, status=status, error_message=error_message)

    def _handle_manual_drive(self, payload: dict) -> bool:
        mode = str((payload or {}).get('mode', '')).strip().lower()
        direction = (payload or {}).get('direction', '').lower()

        if mode == 'analog' or 'throttle' in (payload or {}) or 'steer' in (payload or {}):
            return self._handle_manual_drive_analog(payload)

        try:
            speed = int((payload or {}).get('speed', 180))
        except (TypeError, ValueError):
            speed = 180
        speed = max(60, min(255, speed))

        if not hasattr(self.robot_link, 'manual_drive'):
            logger.warning("Manual drive not supported by current comm link")
            return False

        # Latching behavior:
        # - Click a direction: keeps moving until STOP is clicked.
        # - Click the same direction again: toggles to STOP.
        # - Click STOP: stops.
        # This prevents Mega's manual timeout from stopping motion after ~1.5s.
        requested = (direction or 'stop').strip().lower()
        if requested in ('', 'none'):
            requested = 'stop'

        self._cancel_manual_assist(reason=f"manual {requested or 'command'}")
        self._clear_analog_drive_target()

        with self._manual_drive_lock:
            if requested in ('stop', 'brake'):
                self._manual_drive_active = False
                self._manual_drive_direction = 'stop'
                self._manual_drive_speed = speed
                logger.info("Manual drive STOP (latched)")
            else:
                if self._manual_drive_active and requested == self._manual_drive_direction:
                    # toggle off
                    self._manual_drive_active = False
                    self._manual_drive_direction = 'stop'
                    self._manual_drive_speed = speed
                    logger.info("Manual drive toggled OFF -> STOP")
                else:
                    self._manual_drive_active = True
                    self._manual_drive_direction = requested
                    self._manual_drive_speed = speed
                    logger.info(f"Manual drive latched: {requested} @ {speed}")

        # Start drive loop thread lazily
        if not self._manual_drive_thread or not self._manual_drive_thread.is_alive():
            self._manual_drive_thread = Thread(target=self._manual_drive_loop, daemon=True)
            self._manual_drive_thread.start()

        # Apply immediately (first command)
        try:
            return self._manual_drive_apply_once()
        except Exception as exc:
            logger.warning("Manual drive apply failed: %s", exc)
            return False

    def _set_analog_drive_target(self, throttle: int, steer: int) -> None:
        throttle = max(-255, min(255, int(throttle)))
        steer = max(-255, min(255, int(steer)))
        with self._analog_drive_lock:
            self._analog_drive_throttle = throttle
            self._analog_drive_steer = steer
            self._analog_drive_active = abs(throttle) > 0 or abs(steer) > 0
            self._analog_drive_last_input_mono = time.monotonic()

    def _clear_analog_drive_target(self) -> None:
        with self._analog_drive_lock:
            self._analog_drive_throttle = 0
            self._analog_drive_steer = 0
            self._analog_drive_active = False
            self._analog_drive_last_input_mono = time.monotonic()

    def _joystick_bus_priority_active(self) -> bool:
        with self._analog_drive_lock:
            if self._analog_drive_active:
                return True
            last_input = self._analog_drive_last_input_mono
        return last_input > 0 and (time.monotonic() - last_input) < self._analog_bus_priority_hold

    def _send_analog_drive_once(self, throttle: int, steer: int, joystick_active: bool) -> bool:
        if not self.robot_link:
            return False

        throttle = max(-255, min(255, int(throttle)))
        steer = max(-255, min(255, int(steer)))

        success = False
        if hasattr(self.robot_link, 'send_raw_motor'):
            success = bool(self.robot_link.send_raw_motor(throttle, steer, joystick_active=joystick_active))
        else:
            left_motor = max(-127, min(127, int(round((throttle + steer) * 127 / 255.0))))
            right_motor = max(-127, min(127, int(round((throttle - steer) * 127 / 255.0))))
            if hasattr(self.robot_link, 'send_manual_control'):
                success = bool(self.robot_link.send_manual_control(left_motor, right_motor, joystick_active=joystick_active))

        if success:
            self._analog_drive_last_send_mono = time.monotonic()
        return success

    def _analog_drive_loop(self) -> None:
        """Continuously stream the latest joystick target to the Mega."""
        release_pending = False

        while not shutdown_event.is_set():
            with self._analog_drive_lock:
                throttle = self._analog_drive_throttle
                steer = self._analog_drive_steer
                active = self._analog_drive_active

            if active:
                self._send_analog_drive_once(throttle, steer, joystick_active=True)
                release_pending = True
                shutdown_event.wait(self._analog_drive_interval)
                continue

            if release_pending:
                self._send_analog_drive_once(0, 0, joystick_active=False)
                release_pending = False

            shutdown_event.wait(0.01)

    def _handle_manual_drive_analog(self, payload: dict) -> bool:
        """Apply continuous raw throttle/steer control from the dashboard joystick."""
        if not self.robot_link:
            logger.warning("Analog manual drive not supported by current comm link")
            return False

        try:
            throttle = int((payload or {}).get('throttle', 0))
            steer = int((payload or {}).get('steer', 0))
        except (TypeError, ValueError):
            logger.warning("Invalid analog manual payload: %r", payload)
            return False

        throttle = max(-255, min(255, throttle))
        steer = max(-255, min(255, steer))

        self._cancel_manual_assist(reason="analog manual command")

        with self._manual_drive_lock:
            self._manual_drive_active = False
            self._manual_drive_direction = 'stop'
            self._manual_drive_speed = max(abs(throttle), abs(steer), 0)

        self._set_analog_drive_target(throttle, steer)
        return self._send_analog_drive_once(throttle, steer, joystick_active=(abs(throttle) > 0 or abs(steer) > 0))

    def _get_ai_drive_speed(self, payload: dict) -> int:
        requested = (payload or {}).get('speed')
        if requested is not None:
            try:
                return max(60, min(255, int(requested)))
            except (TypeError, ValueError):
                logger.debug("Invalid AI speed payload: %r", requested)

        if self.nav_controller:
            nav_speed = getattr(self.nav_controller, 'DRIVE_SPEED', None)
            if nav_speed is not None:
                try:
                    return max(60, min(255, int(nav_speed)))
                except (TypeError, ValueError):
                    logger.debug("Invalid NavController DRIVE_SPEED: %r", nav_speed)

        return max(60, min(255, int(self._manual_drive_speed or 180)))

    def _manual_drive_apply_once(self) -> bool:
        if not self.robot_link or not hasattr(self.robot_link, 'manual_drive'):
            return False
        with self._manual_drive_lock:
            active = self._manual_drive_active
            direction = self._manual_drive_direction
            speed = self._manual_drive_speed

        if not active:
            # Stop command
            return bool(self.robot_link.manual_drive('stop', speed))

        return bool(self.robot_link.manual_drive(direction, speed))

    def _manual_drive_loop(self) -> None:
        """Continuously refresh manual-drive commands while latched active."""
        # Refresh rate must be < Mega MANUAL_TIMEOUT (1500ms). Use ~5Hz.
        while not shutdown_event.is_set():
            try:
                if self.robot_link and hasattr(self.robot_link, 'manual_drive'):
                    with self._manual_drive_lock:
                        active = self._manual_drive_active
                    if active:
                        self._manual_drive_apply_once()
            except Exception as exc:
                logger.debug("Manual drive loop error: %s", exc)

            shutdown_event.wait(0.2)

    @staticmethod
    def _normalize_heading_error(error: float) -> float:
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0
        return error

    def _read_obstacle_distance_cm(self) -> int:
        if not self.robot_link or not hasattr(self.robot_link, 'request_obstacle_status'):
            return -1
        try:
            obstacle = self.robot_link.request_obstacle_status()
            if not obstacle:
                return -1
            return int(obstacle.get('distance_cm', -1))
        except Exception:
            return -1

    def _read_current_heading(self) -> Optional[float]:
        try:
            if self.compass and hasattr(self.compass, 'read_heading'):
                heading = self.compass.read_heading()
                if heading is not None:
                    return float(heading) % 360.0
        except Exception:
            pass
        if self.nav_controller is not None:
            try:
                return float(getattr(self.nav_controller, '_current_heading', 0.0)) % 360.0
            except Exception:
                pass
        return None

    def _manual_assist_turn_direction(self, logical_direction: str) -> str:
        direction = (logical_direction or 'left').strip().lower()
        if self._invert_rotation_direction:
            return 'right' if direction == 'left' else 'left'
        return direction

    def _send_manual_assist_drive(self, direction: str, speed: int) -> bool:
        if not self.robot_link or not hasattr(self.robot_link, 'manual_drive'):
            return False
        direction = (direction or 'stop').strip().lower()
        if direction in ('left', 'right'):
            direction = self._manual_assist_turn_direction(direction)
        return bool(self.robot_link.manual_drive(direction, speed))

    def _cancel_manual_assist(self, reason: str = 'cancelled') -> None:
        with self._manual_assist_lock:
            was_active = self._manual_assist_active
            self._manual_assist_active = False
        self._manual_assist_cancel.set()
        if was_active:
            logger.info("Manual assist avoidance cancelled — %s", reason)

    def _resume_latched_manual_forward(self, speed: int) -> None:
        with self._manual_drive_lock:
            self._manual_drive_active = True
            self._manual_drive_direction = 'forward'
            self._manual_drive_speed = max(60, min(255, int(speed)))

        if not self._manual_drive_thread or not self._manual_drive_thread.is_alive():
            self._manual_drive_thread = Thread(target=self._manual_drive_loop, daemon=True)
            self._manual_drive_thread.start()

        try:
            self._manual_drive_apply_once()
        except Exception as exc:
            logger.debug("Manual forward resume apply failed: %s", exc)

    def _run_manual_assist_avoidance(self, resume_heading: Optional[float], resume_speed: int) -> None:
        logger.info("Manual assist avoidance started (heading=%s speed=%d)",
                    f"{resume_heading:.1f}°" if resume_heading is not None else "unknown",
                    resume_speed)

        nav = self.nav_controller
        safe_distance_cm = int(getattr(nav, 'SAFE_AVOID_DISTANCE_CM', 40) or 40)
        reverse_timeout = float(getattr(nav, 'SAFE_REVERSE_TIMEOUT', 3.0) or 3.0)
        obstacle_turn_time = float(getattr(nav, 'OBSTACLE_TURN_TIME', 1.0) or 1.0)
        obstacle_pass_time = float(getattr(nav, 'OBSTACLE_PASS_TIME', 1.5) or 1.5)
        heading_deadband = float(getattr(nav, 'HEADING_DEADBAND', 5.0) or 5.0)
        reacquire_timeout = float(CONFIG.get('manual_assist_reacquire_timeout', 8.0))
        max_attempts = int(CONFIG.get('manual_assist_max_attempts', 3))

        reverse_speed = int(nav._get_reverse_speed()) if nav and hasattr(nav, '_get_reverse_speed') else 90
        turn_speed = int(nav._get_turn_speed(fast=True)) if nav and hasattr(nav, '_get_turn_speed') else 150
        forward_speed = int(nav._get_avoid_forward_speed()) if nav and hasattr(nav, '_get_avoid_forward_speed') else max(95, min(140, int(resume_speed)))

        success = False

        try:
            for attempt in range(1, max_attempts + 1):
                if self._manual_assist_cancel.is_set() or shutdown_event.is_set():
                    return

                self._manual_assist_last_turn = 'left' if self._manual_assist_last_turn == 'right' else 'right'
                avoid_direction = self._manual_assist_last_turn
                logger.info("Manual assist attempt %d/%d using %s avoidance", attempt, max_attempts, avoid_direction)

                # Phase 1: reverse until safe distance
                reverse_start = time.time()
                while not self._manual_assist_cancel.is_set() and not shutdown_event.is_set():
                    dist_cm = self._read_obstacle_distance_cm()
                    if dist_cm <= 0 or dist_cm >= safe_distance_cm:
                        break
                    if (time.time() - reverse_start) >= reverse_timeout:
                        logger.warning("Manual assist reverse timeout at %d cm", dist_cm)
                        break
                    self._send_manual_assist_drive('reverse', reverse_speed)
                    shutdown_event.wait(0.1)

                self._send_manual_assist_drive('stop', reverse_speed)
                if self._manual_assist_cancel.is_set() or shutdown_event.is_set():
                    return

                # Phase 2: turn away
                turn_start = time.time()
                while (time.time() - turn_start) < obstacle_turn_time:
                    if self._manual_assist_cancel.is_set() or shutdown_event.is_set():
                        return
                    self._send_manual_assist_drive(avoid_direction, turn_speed)
                    shutdown_event.wait(0.1)

                self._send_manual_assist_drive('stop', turn_speed)
                if self._manual_assist_cancel.is_set() or shutdown_event.is_set():
                    return

                # Phase 3: move past obstacle
                forward_start = time.time()
                obstacle_reappeared = False
                while (time.time() - forward_start) < obstacle_pass_time:
                    if self._manual_assist_cancel.is_set() or shutdown_event.is_set():
                        return
                    dist_cm = self._read_obstacle_distance_cm()
                    if 0 < dist_cm < safe_distance_cm:
                        obstacle_reappeared = True
                        logger.info("Manual assist forward interrupted at %d cm", dist_cm)
                        break
                    self._send_manual_assist_drive('forward', forward_speed)
                    shutdown_event.wait(0.1)

                self._send_manual_assist_drive('stop', forward_speed)
                if self._manual_assist_cancel.is_set() or shutdown_event.is_set():
                    return

                if obstacle_reappeared:
                    continue

                # Phase 4: reacquire original heading before resuming forward
                if resume_heading is not None:
                    reacquire_start = time.time()
                    while not self._manual_assist_cancel.is_set() and not shutdown_event.is_set():
                        current_heading = self._read_current_heading()
                        if current_heading is None:
                            break
                        error = self._normalize_heading_error(resume_heading - current_heading)
                        if abs(error) <= heading_deadband:
                            break
                        if (time.time() - reacquire_start) >= reacquire_timeout:
                            logger.warning("Manual assist heading reacquire timeout at error %.1f°", error)
                            break
                        turn_dir = 'right' if error > 0 else 'left'
                        self._send_manual_assist_drive(turn_dir, turn_speed)
                        shutdown_event.wait(0.1)
                    self._send_manual_assist_drive('stop', turn_speed)

                success = True
                break
        finally:
            self._send_manual_assist_drive('stop', resume_speed)
            with self._manual_assist_lock:
                cancelled = self._manual_assist_cancel.is_set()
                self._manual_assist_active = False

            if not cancelled and success:
                logger.info("Manual assist avoidance complete — resuming forward")
                self._resume_latched_manual_forward(resume_speed)
            elif cancelled:
                logger.info("Manual assist avoidance exited due to operator override")
            else:
                logger.warning("Manual assist avoidance ended without a clean resume")

    def _start_manual_assist_avoidance(self) -> bool:
        if not self.robot_link or not hasattr(self.robot_link, 'manual_drive'):
            return False

        with self._manual_assist_lock:
            if self._manual_assist_active:
                return False

        with self._manual_drive_lock:
            if not self._manual_drive_active or self._manual_drive_direction != 'forward':
                return False
            resume_speed = self._manual_drive_speed
            self._manual_drive_active = False
            self._manual_drive_direction = 'stop'

        resume_heading = self._read_current_heading()
        self._manual_assist_cancel.clear()
        with self._manual_assist_lock:
            self._manual_assist_active = True
            self._manual_assist_resume_heading = resume_heading
            self._manual_assist_resume_speed = resume_speed
            self._manual_assist_resume_direction = 'forward'

        try:
            self._manual_drive_apply_once()
        except Exception:
            pass

        self._manual_assist_thread = Thread(
            target=self._run_manual_assist_avoidance,
            args=(resume_heading, resume_speed),
            daemon=True,
            name='manual-assist-avoidance',
        )
        self._manual_assist_thread.start()
        return True

    def _handle_manual_override(self, payload: dict) -> bool:
        mode = (payload or {}).get('mode', 'hold')
        logger.info(f"Manual override -> {mode}")
        normalized = str(mode or 'hold').strip().lower()

        if normalized in ('release', 'off', 'disable', 'disabled', 'auto'):
            self._stop_latched_manual_drive(release_override=True)
            return True

        if normalized in ('hold', 'on', 'enable', 'enabled'):
            if self.nav_controller and self.nav_controller.is_active:
                logger.info("Manual override requested — pausing NavController")
                self._ai_resume_epoch += 1
                self.nav_controller.pause()

        if not hasattr(self.robot_link, 'manual_override'):
            return False
        return self.robot_link.manual_override(mode)

    def _handle_manual_speed(self, payload: dict) -> bool:
        speed = int((payload or {}).get('value', 180))
        speed = max(60, min(255, speed))
        if not hasattr(self.robot_link, 'manual_speed'):
            return False
        return self.robot_link.manual_speed(speed)

    def _handle_auto_speed(self, payload: dict) -> bool:
        """Set autonomous navigation base speed (affects waypoint following)."""
        speed = int((payload or {}).get('value', 120))
        speed = max(60, min(255, speed))
        applied = False
        if self.nav_controller and hasattr(self.nav_controller, 'set_drive_speed'):
            applied = bool(self.nav_controller.set_drive_speed(speed))
        elif self.robot_link and hasattr(self.robot_link, 'set_auto_speed'):
            applied = bool(self.robot_link.set_auto_speed(speed))
        return applied

    def _handle_auto_speed_mode(self, payload: dict) -> bool:
        enabled = bool((payload or {}).get('enabled', False))
        if not self.nav_controller or not hasattr(self.nav_controller, 'set_adaptive_speed'):
            return False
        return bool(self.nav_controller.set_adaptive_speed(enabled))

    def _release_manual_override(self) -> None:
        if not self.robot_link:
            return

        self._clear_analog_drive_target()

        try:
            if hasattr(self.robot_link, 'send_raw_motor'):
                self.robot_link.send_raw_motor(0, 0, joystick_active=False)
                return
            if hasattr(self.robot_link, 'release_manual_override_only'):
                self.robot_link.release_manual_override_only()
            elif hasattr(self.robot_link, 'send_manual_control'):
                self.robot_link.send_manual_control(0, 0, joystick_active=False)
        except Exception as exc:
            logger.debug("Manual override release failed: %s", exc)

    def _stop_latched_manual_drive(self, release_override: bool = False) -> None:
        self._cancel_manual_assist(reason="manual latch released")
        self._clear_analog_drive_target()
        with self._manual_drive_lock:
            self._manual_drive_active = False
            self._manual_drive_direction = 'stop'

        try:
            self._manual_drive_apply_once()
        except Exception:
            pass

        if release_override:
            self._release_manual_override()

    def _handoff_to_navigation(self) -> None:
        """Ensure manual latching is fully released before Pi nav takes over."""
        self._stop_latched_manual_drive(release_override=True)

    def _handle_manual_target_start(self, payload: dict) -> bool:
        """Start heading-lock navigation to a single target coordinate."""
        if not self.nav_controller or not hasattr(self.nav_controller, 'start_heading_lock_target'):
            return False

        try:
            latitude = float((payload or {}).get('latitude'))
            longitude = float((payload or {}).get('longitude'))
        except (TypeError, ValueError):
            logger.warning("MANUAL_TARGET_START rejected — invalid coordinates")
            return False

        self._handoff_to_navigation()

        logger.info("Starting heading-lock target navigation to %.6f, %.6f", latitude, longitude)
        return bool(self.nav_controller.start_heading_lock_target(latitude, longitude))

    def _handle_manual_target_return_home(self, payload: dict) -> bool:
        """Return to the saved departure point for the current heading-lock run."""
        if not self.nav_controller or not hasattr(self.nav_controller, 'return_to_heading_lock_home'):
            return False

        self._handoff_to_navigation()
        logger.info("Starting heading-lock return-home navigation")
        return bool(self.nav_controller.return_to_heading_lock_home())

    def _handle_waypoint_push(self) -> bool:
        """Load waypoints from dashboard DB into Pi nav controller (NOT Mega)."""
        waypoints = self.api_client.get_waypoints()
        if not waypoints:
            logger.warning("No waypoints to load")
            if self.nav_controller:
                self.nav_controller.set_waypoints([])
            return False
        waypoints = sorted(waypoints, key=lambda w: w.get('sequence', 0))
        if self.nav_controller:
            self.nav_controller.set_waypoints(waypoints)
            logger.info("Loaded %d waypoints into Pi NavController", len(waypoints))
            return True
        else:
            logger.warning("NavController not available — cannot load waypoints")
            return False
    
    def camera_loop(self):
        """Capture and stream camera frames (simple synchronous version)"""
        if not self.camera or not self.camera.enabled:
            logger.info("Camera not enabled; skipping camera loop")
            return

        logger.info("Starting camera loop (simple JPEG relay)...")
        max_frame_bytes = int(CONFIG.get('camera', {}).get('max_frame_size', 2 * 1024 * 1024))
        fps = max(1, int(CONFIG.get('camera', {}).get('fps', 5)))
        base_url = self.api_client.base_url.rstrip('/') if getattr(self.api_client, 'base_url', None) else CONFIG.get('dashboard_api', {}).get('base_url', '').rstrip('/')

        interval = 1.0 / float(fps)
        next_capture = time.time()

        try:
            import requests
            session = requests.Session()
            url = f"{base_url}/api/camera/frame_binary"
            headers = {'Content-Type': 'image/jpeg'}

            while not shutdown_event.is_set():
                if time.time() >= next_capture:
                    next_capture = time.time() + interval
                    try:
                        jpeg = self.camera.capture_frame_jpeg()
                        if jpeg and len(jpeg) <= max_frame_bytes:
                            # Feed MJPEG direct-stream server (near-zero latency)
                            mjpeg_update_frame(jpeg)

                            timestamp_iso = datetime.utcnow().isoformat()
                            params = {
                                'device_id': self.device_id,
                                'timestamp': timestamp_iso
                            }
                            resp = session.post(
                                url,
                                params=params,
                                data=jpeg,
                                headers=headers,
                                timeout=(1.5, 2.5)
                            )
                            if resp.status_code in (200, 202):
                                logger.debug("Camera frame uploaded: %d bytes", len(jpeg))
                            else:
                                logger.debug("Camera upload failed: %s", resp.status_code)
                        elif jpeg:
                            logger.warning("Captured frame too large: %d bytes", len(jpeg))
                        else:
                            logger.debug("Camera capture returned None")
                    except Exception as exc:
                        logger.debug("Camera capture/upload failed: %s", exc)

                shutdown_event.wait(0.01)
        except Exception as exc:
            logger.error("Camera loop error: %s", exc)

    def get_battery_level(self):
        """Get battery level from sysfs power_supply or UPS HAT.

        Checks, in order:
        0. ADS1115 channel 3 via voltage divider module (if enabled in config)
        1. /sys/class/power_supply/BAT*/capacity  (Linux laptop / UPS HAT)
        2. INA219 I2C sensor on 0x40 (common Pi UPS boards)
        3. Returns -1 (unknown) if no hardware found.
        """
        # 0. ADS1115 channel A3 — DC voltage sensor module (resistor divider)
        try:
            vs_cfg = CONFIG.get('sensors', {}).get('voltage_sensor', {})
            if vs_cfg.get('enabled', False):
                from Adafruit_ADS1x15 import ADS1115 as _ADS1115
                adc_cfg = CONFIG.get('sensors', {}).get('adc', {})
                _ads = _ADS1115(address=adc_cfg.get('address', 0x48))
                channel = vs_cfg.get('adc_channel', 3)
                gain = adc_cfg.get('gain', 1)
                raw = _ads.read_adc(channel, gain=gain)
                if raw > 0:
                    # gain=1 → ±4.096 V full-scale, 32767 = +4.096 V
                    adc_voltage = raw * (4.096 / 32767.0)
                    ratio = vs_cfg.get('voltage_divider_ratio', 5.0)
                    v_in = adc_voltage * ratio
                    v_min = vs_cfg.get('v_min', 6.0)
                    v_max = vs_cfg.get('v_max', 8.4)
                    if v_min <= v_in <= (v_max + 1.0):  # +1 V tolerance above full
                        pct = ((v_in - v_min) / (v_max - v_min)) * 100.0
                        return round(max(0.0, min(100.0, pct)), 1)
        except Exception:
            pass

        # 1. sysfs battery (Linux kernel generic)
        try:
            import glob
            for bat_path in sorted(glob.glob('/sys/class/power_supply/BAT*/capacity')):
                with open(bat_path, 'r') as f:
                    pct = float(f.read().strip())
                    return round(pct, 1)
        except Exception:
            pass

        # 2. INA219 I2C voltage sensor (common UPS HATs)
        try:
            import smbus2
            bus = smbus2.SMBus(1)
            raw = bus.read_word_data(0x40, 0x02)  # bus voltage register
            voltage = ((raw >> 3) * 4) / 1000.0  # mV → V
            bus.close()
            if 6.0 < voltage < 25.0:
                # Simple linear mapping for typical 2S-3S LiPo
                # 3S: 12.6V = 100%, 9.0V = 0%
                # 2S: 8.4V = 100%, 6.0V = 0%
                if voltage > 10.0:  # likely 3S
                    pct = ((voltage - 9.0) / (12.6 - 9.0)) * 100.0
                else:  # likely 2S
                    pct = ((voltage - 6.0) / (8.4 - 6.0)) * 100.0
                return round(max(0.0, min(100.0, pct)), 1)
        except Exception:
            pass

        # 3. No battery hardware found
        return -1.0

    def get_cpu_temperature(self):
        """Get Raspberry Pi CPU temperature"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read().strip()) / 1000.0
                return temp
        except:
            return 0.0

    def shutdown(self):
        """Gracefully shutdown all systems"""
        logger.info("Shutting down robot controller...")
        
        # Set shutdown event
        shutdown_event.set()
        
        # Send offline status
        try:
            self.api_client.send_status({
                'online': False,
                'device_id': self.device_id
            })
        except:
            pass
        
        # Stop components
        try:
            if self.mjpeg_server:
                self.mjpeg_server.stop()
            if self.camera and hasattr(self.camera, 'stop'):
                self.camera.stop()
            if self.robot_link:
                if self.comm_mode == 'serial' and hasattr(self.robot_link, 'stop'):
                    self.robot_link.stop()
                elif hasattr(self.robot_link, 'close'):
                    self.robot_link.close()
            if self.gsm and hasattr(self.gsm, 'disconnect'):
                self.gsm.disconnect()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        
        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2)
        
        logger.info("Shutdown complete")


def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info(f"Received signal {signum}, initiating shutdown...")
    shutdown_event.set()


def main():
    """Main entry point"""
    logger.info("=" * 60)
    logger.info("Environmental Monitoring Robot - Raspberry Pi")
    logger.info("=" * 60)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create robot controller
    robot = None
    
    try:
        robot = RobotController()
        
        # Start robot
        if robot.start():
            logger.info("Robot is running. Press Ctrl+C to stop.")
            
            # Keep main thread alive
            while not shutdown_event.is_set():
                time.sleep(1)
        else:
            logger.error("Failed to start robot")
            return 1
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        return 1
    finally:
        if robot:
            robot.shutdown()
    
    logger.info("Program terminated")
    return 0


if __name__ == '__main__':
    sys.exit(main())
