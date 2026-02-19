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
            "update_interval": 5,
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
        
        # Dashboard API (required)
        try:
            self.api_client = DashboardAPI(CONFIG['dashboard_api'], self.device_id)
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
                logger.info("Pi-side NavController initialized")
            except Exception as e:
                logger.warning(f"NavController init failed: {e}")
        
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

            # Start Pi-side navigation status broadcast loop (always runs —
            # sends full nav status, compass-only, or heartbeat depending on
            # which components are available)
            nav_status_thread = Thread(target=self.nav_status_loop, daemon=True)
            nav_status_thread.start()
            self.threads.append(nav_status_thread)
            
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
        """GPS loop: SIM7600E is the primary GPS source for the dashboard.
        Always forwards position to Mega as a seed so navigation can start
        before the Neo-6M gets its own fix.  Once Neo-6M locks, the Mega
        ignores the seed automatically.
        """
        logger.info("Starting GPS loop...")
        
        while not shutdown_event.is_set():
            try:
                # Skip I2C GPS polling if wireless backup control is active
                if self.wireless_backup_active:
                    logger.debug("GPS loop paused (wireless backup active)")
                    shutdown_event.wait(self.gps_interval)
                    continue
                
                gps_data = None
                
                # Primary source: SIM7600E GPS
                if self.sim7600e:
                    gps_data = self.sim7600e.get_gps_data()
                    if gps_data and self.robot_link:
                        # Always forward to Mega — seeds its position until
                        # the Neo-6M gets its own fix (seedPosition is a no-op
                        # once Neo-6M has locked).
                        try:
                            self.robot_link.send_gps_data(gps_data)
                            logger.debug("Forwarded SIM7600E GPS to Mega")
                        except Exception as e:
                            logger.debug(f"Could not forward GPS to Mega: {e}")
                
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
                            gps_data['heading'] = 0.0
                    
                    # Add timestamp and device ID
                    gps_data['timestamp'] = datetime.now().isoformat()
                    gps_data['device_id'] = self.device_id
                    
                    # Send to dashboard
                    success = self.api_client.send_gps_data(gps_data)
                    
                    if success:
                        try:
                            logger.debug(f"GPS data sent: {gps_data['latitude']:.6f}, {gps_data['longitude']:.6f}")
                        except Exception:
                            logger.debug("GPS data sent (values present)")
                    else:
                        logger.warning("Failed to send GPS data")
                else:
                    logger.debug("No valid GPS data available")
                
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
                    'system_info': {
                        'cpu': psutil.cpu_percent(),
                        'memory': psutil.virtual_memory().percent,
                        'temperature': self.get_cpu_temperature()
                    },
                    'device_id': self.device_id
                }

                if self.robot_link:
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
                obstacle = None
                if self.robot_link:
                    try:
                        obstacle = self.robot_link.request_obstacle_status()
                    except Exception:
                        obstacle = None

                now = time.time()
                state = bool(obstacle and obstacle.get('obstacle'))

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
                        'timestamp': datetime.utcnow().isoformat(),
                    }
                    try:
                        self.api_client.send_event(payload)
                        last_emit = now
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

        # Poll interval: fast on LAN, still responsive on cellular
        poll_interval = CONFIG.get('instant_poll_interval', 0.15)  # 150ms default

        logger.info("Instant command loop polling: %s  timeout=%s  interval=%.2fs",
                     url, poll_timeout, poll_interval)

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

            shutdown_event.wait(poll_interval)

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
                if self.nav_controller:
                    success = self.nav_controller.start()
                else:
                    logger.warning("NavController not available")
                    success = False
            elif command_type == 'NAV_PAUSE':
                if self.nav_controller:
                    self.nav_controller.pause()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_RESUME':
                if self.nav_controller:
                    self.nav_controller.resume()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_STOP':
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
            elif command_type == 'MANUAL_DRIVE':
                success = self._handle_manual_drive(payload)
            elif command_type == 'MANUAL_OVERRIDE':
                success = self._handle_manual_override(payload)
            elif command_type == 'MANUAL_SPEED':
                success = self._handle_manual_speed(payload)
            elif command_type == 'AUTO_SPEED':
                success = self._handle_auto_speed(payload)
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
                if self.nav_controller:
                    success = self.nav_controller.return_home()
                else:
                    success = False
            elif command_type == 'WAYPOINT_PUSH':
                success = self._handle_waypoint_push()
            elif command_type == 'SOUND_BUZZER':
                duration = int((payload or {}).get('duration', 3))
                success = self.robot_link.sound_buzzer(duration) if hasattr(self.robot_link, 'sound_buzzer') else False
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
        direction = (payload or {}).get('direction', '').lower()
        speed = int((payload or {}).get('speed', 180))
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

    def _handle_manual_override(self, payload: dict) -> bool:
        mode = (payload or {}).get('mode', 'hold')
        if not hasattr(self.robot_link, 'manual_override'):
            return False
        logger.info(f"Manual override -> {mode}")
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
        if not hasattr(self.robot_link, 'set_auto_speed'):
            return False
        return bool(self.robot_link.set_auto_speed(speed))

    def _handle_waypoint_push(self) -> bool:
        """Load waypoints from dashboard DB into Pi nav controller (NOT Mega)."""
        waypoints = self.api_client.get_waypoints()
        if not waypoints:
            logger.warning("No waypoints to load")
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
        """Get battery level (implement based on hardware)"""
        # TODO: Implement actual battery level reading
        # This is a placeholder that returns mock data
        try:
            # If you have a battery monitoring circuit, read it here
            # For now, return a simulated value
            return 85.0
        except:
            return 0.0

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