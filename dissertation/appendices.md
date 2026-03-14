# APPENDICES

---

## Appendix A: Raspberry Pi Main Controller Source Code

**File:** `raspberry_pi/main.py`

**Description:** This file contains the `RobotController` class — the top-level orchestrator of all Raspberry Pi subsystems. It initialises all hardware interfaces (sensor manager, compass, GPS module, I2C link, camera, cloud uploader, dashboard API client) and launches seven concurrent threads: the sensor loop, GPS loop, status loop, obstacle alert loop, command polling loop, instant command loop, and navigation status broadcast loop. The `_process_command()` method handles all 18 command types dispatched from the web dashboard (NAV_START, NAV_PAUSE, NAV_RESUME, NAV_STOP, MANUAL_DRIVE, WAYPOINT_PUSH, AI_DRIVE, ENGAGE_WIRELESS, etc.). The file also implements the signal handler and `main()` entry point.

---

```python
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
                "max_frame_size": 2 * 1024 * 1024
            },
            "log_level": "INFO",
            "i2c": {
                "bus": 1,
                "mega_address": 8
            }
        }, "<default>"


CONFIG, _CONFIG_PATH = _load_config()

logger = setup_logger('main', CONFIG.get('log_level', 'INFO'))
logger.info("Loaded config from %s", _CONFIG_PATH)

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

        self.sensor_manager = None
        try:
            self.sensor_manager = SensorManager(CONFIG['sensors'])
            logger.info("Sensor Manager initialized")
        except Exception as e:
            logger.warning(f"Sensor Manager not available: {e}")

        self.compass = None
        try:
            compass_cal = CONFIG.get('compass', {})
            decl = compass_cal.get('declination_deg', -0.5)
            self.compass = Compass(declination_deg=decl, config=CONFIG)
            logger.info("Compass initialized")
        except Exception as e:
            logger.warning(f"Compass not available: {e}")

        self.gsm = None
        self.sim7600e = None
        self.gps_from_gsm = False

        gsm_cfg = CONFIG.get('gsm') or {}
        sim_cfg = CONFIG.get('sim7600e')
        gsm_module_type = str(gsm_cfg.get('module_type', '')).strip().upper()

        if not sim_cfg and gsm_cfg and gsm_module_type == 'SIM7600E':
            sim_cfg = gsm_cfg

        if sim_cfg:
            try:
                self.sim7600e = SIM7600EGPS(sim_cfg)
                self.gps_from_gsm = True
                logger.info("SIM7600E module initialized")
            except Exception as e:
                logger.warning(f"SIM7600E module not available: {e}")

        try:
            gsm_enabled = bool(gsm_cfg.get('enabled', True))
            if gsm_enabled and gsm_cfg and not self.sim7600e:
                self.gsm = GSMModule(gsm_cfg)
                logger.info("GSM Module initialized (legacy)")
        except Exception as e:
            logger.warning(f"GSM Module not available: {e}")

        self.comm_mode = None
        self.robot_link = None
        self._manual_drive_lock = Lock()
        self._manual_drive_active = False
        self._manual_drive_direction = 'stop'
        self._manual_drive_speed = 180
        self._manual_drive_thread = None
        self.wireless_backup_active = False

        try:
            if CONFIG.get('i2c'):
                self.robot_link = I2CComm(CONFIG['i2c'])
                self.comm_mode = 'i2c'
                logger.info("Configured I2C communication with Mega")
            elif CONFIG.get('arduino'):
                self.robot_link = ArduinoComm(CONFIG['arduino'])
                self.comm_mode = 'serial'
                logger.info("Configured serial communication with Mega")
        except Exception as e:
            logger.warning(f"Arduino Mega communication not available: {e}")

        try:
            self.api_client = DashboardAPI(CONFIG['dashboard_api'], self.device_id)
            from raspberry_pi.communication.api_client import set_qmi_health
            set_qmi_health(get_qmi_monitor())
            logger.info("Dashboard API Client initialized")
        except Exception as e:
            logger.error(f"Failed to initialize Dashboard API: {e}")
            raise

        try:
            self.cloud_uploader = CloudUploader(CONFIG.get('cloud_integrations'))
            logger.info("Cloud uploader configured")
        except Exception as e:
            logger.warning(f"Cloud uploader not available: {e}")
            self.cloud_uploader = None

        self.camera = None
        self.mjpeg_server = None
        if CONFIG.get('camera', {}).get('enabled', False):
            try:
                self.camera = CameraStream(CONFIG['camera'])
                if self.camera and self.camera.enabled:
                    mjpeg_port = int(CONFIG.get('camera', {}).get('mjpeg_port', 8081))
                    self.mjpeg_server = MJPEGServer(host='0.0.0.0', port=mjpeg_port)
                    self.mjpeg_server.start()
            except Exception as e:
                logger.warning(f"Camera not available: {e}")

        self.command_poll_interval = CONFIG.get('command_poll_interval', 2)
        self._latest_gps = {}
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
                if self.robot_link:
                    self.nav_controller._mega_gps_provider = self._get_mega_gps
                self.nav_controller.set_event_callback(self._nav_event_callback)
                logger.info("Pi-side NavController initialized")
            except Exception as e:
                logger.warning(f"NavController init failed: {e}")

        qmi_cfg = CONFIG.get('qmi_health', {})
        self.qmi_health = get_qmi_monitor(enabled=qmi_cfg.get('enabled', True))
        self._ai_resume_epoch = 0
        self.threads = []

    def start(self):
        """Start all robot systems"""
        logger.info("Starting Robot Controller...")
        try:
            if self.sim7600e:
                try:
                    if self.sim7600e.connect():
                        logger.info("SIM7600E module connected")
                except Exception as e:
                    logger.warning(f"SIM7600E initialization failed: {e}")

            if self.robot_link:
                try:
                    if self.comm_mode == 'i2c':
                        ping_ok = False
                        for attempt in range(1, 11):
                            if self.robot_link.ping():
                                ping_ok = True
                                break
                            time.sleep(0.5)
                        if not ping_ok:
                            logger.warning("Mega did not acknowledge I2C ping")
                except Exception as e:
                    logger.warning(f"Failed to start navigation link: {e}")

            if self.camera and self.camera.enabled:
                camera_thread = Thread(target=self.camera_loop, daemon=True)
                camera_thread.start()
                self.threads.append(camera_thread)

            if self.sensor_manager:
                sensor_thread = Thread(target=self.sensor_loop, daemon=True)
                sensor_thread.start()
                self.threads.append(sensor_thread)

            gps_thread = Thread(target=self.gps_loop, daemon=True)
            gps_thread.start()
            self.threads.append(gps_thread)

            status_thread = Thread(target=self.status_loop, daemon=True)
            status_thread.start()
            self.threads.append(status_thread)

            if self.robot_link and hasattr(self.robot_link, 'request_obstacle_status'):
                obstacle_thread = Thread(target=self.obstacle_loop, daemon=True)
                obstacle_thread.start()
                self.threads.append(obstacle_thread)

            command_thread = Thread(target=self.command_loop, daemon=True)
            command_thread.start()
            self.threads.append(command_thread)

            instant_cmd_thread = Thread(target=self.instant_command_loop, daemon=True)
            instant_cmd_thread.start()
            self.threads.append(instant_cmd_thread)

            nav_status_thread = Thread(target=self.nav_status_loop, daemon=True)
            nav_status_thread.start()
            self.threads.append(nav_status_thread)

            self.qmi_health.start()
            logger.info("All systems started successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to start robot: {e}")
            return False

    def sensor_loop(self):
        """Read and transmit sensor data"""
        while not shutdown_event.is_set():
            try:
                sensor_data = {}
                if self.sensor_manager:
                    try:
                        sensor_data = self.sensor_manager.read_all()
                    except Exception as sensor_err:
                        logger.debug(f"Sensor manager read failed: {sensor_err}")

                if self.compass:
                    try:
                        heading = self.compass.read_heading()
                        sensor_data['heading'] = heading
                    except Exception as e:
                        sensor_data['heading'] = 0

                sensor_data['timestamp'] = datetime.now().isoformat()
                sensor_data['device_id'] = self.device_id

                for key in ['temperature', 'humidity', 'mq2', 'mq7', 'mq135']:
                    if sensor_data.get(key) is None:
                        sensor_data[key] = 0

                success = self.api_client.send_sensor_data(sensor_data)
                if not success:
                    logger.warning("Failed to send sensor data")

                if self.cloud_uploader:
                    try:
                        self.cloud_uploader.publish_sensor_data(sensor_data)
                    except Exception as exc:
                        logger.error(f"Error publishing to cloud: {exc}")

            except Exception as e:
                logger.error(f"Error in sensor loop: {e}")
            shutdown_event.wait(self.update_interval)

    def _get_nav_gps(self) -> dict:
        """GPS provider for NavController — returns latest SIM7600E position."""
        with self._latest_gps_lock:
            data = self._latest_gps.copy()
        if data.get('latitude') and data.get('longitude'):
            return data
        return None

    def _get_mega_gps(self) -> dict:
        """Fallback GPS provider — polls Neo-6M via Mega I2C."""
        try:
            if self.robot_link and not self.wireless_backup_active:
                return self.robot_link.request_gps_data()
        except Exception as e:
            logger.debug("Mega GPS poll failed: %s", e)
        return None

    def _nav_event_callback(self, event_type: str, payload: dict):
        """Forward NavController events to the dashboard."""
        try:
            import requests as _req
            url = f"{self.dashboard_url}/api/event"
            event_data = {'type': event_type.upper(), 'device_id': CONFIG.get('device_id', 'robot_01'), 'timestamp': None}
            event_data.update(payload)
            _req.post(url, json=event_data, timeout=2)
        except Exception as e:
            logger.debug("Nav event send failed: %s", e)

    def gps_loop(self):
        """GPS loop: reads position from SIM7600E (primary) or Neo-6M (fallback)."""
        while not shutdown_event.is_set():
            try:
                if self.wireless_backup_active:
                    shutdown_event.wait(self.gps_interval)
                    continue

                gps_data = None
                gps_source = None

                if self.sim7600e:
                    try:
                        gps_data = self.sim7600e.get_gps_data()
                    except Exception:
                        gps_data = None

                    if gps_data and gps_data.get('latitude') is not None:
                        gps_source = 'SIM7600E'
                        if self.robot_link:
                            try:
                                self.robot_link.send_gps_data(gps_data)
                            except Exception:
                                pass
                    else:
                        gps_data = None

                if gps_data is None and self.robot_link and not self.wireless_backup_active:
                    try:
                        mega_gps = self.robot_link.request_gps_data()
                        if mega_gps and mega_gps.get('valid') and mega_gps.get('latitude'):
                            gps_data = {
                                'latitude': float(mega_gps['latitude']),
                                'longitude': float(mega_gps['longitude']),
                                'altitude': 0.0,
                                'speed': float(mega_gps.get('speed', 0) or 0),
                                'satellites': int(mega_gps.get('satellites', 0) or 0),
                                'source': 'Neo-6M',
                            }
                            gps_source = 'Neo-6M'
                    except Exception:
                        pass

                if gps_data and gps_data.get('latitude') is not None:
                    with self._latest_gps_lock:
                        self._latest_gps = {
                            'latitude': float(gps_data['latitude']),
                            'longitude': float(gps_data['longitude']),
                            'speed': float(gps_data.get('speed', 0) or 0),
                        }

                    if self.compass:
                        try:
                            gps_data['heading'] = float(self.compass.read_heading())
                        except Exception:
                            pass

                    gps_data['timestamp'] = datetime.now().isoformat()
                    gps_data['device_id'] = self.device_id
                    self.api_client.send_gps_data(gps_data)

            except Exception as e:
                logger.error(f"Error in GPS loop: {e}")
            shutdown_event.wait(self.gps_interval)

    def status_loop(self):
        """Update robot status"""
        while not shutdown_event.is_set():
            try:
                if self.wireless_backup_active:
                    shutdown_event.wait(self.status_interval)
                    continue

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
                    if self.comm_mode == 'i2c':
                        try:
                            self.robot_link.send_heartbeat()
                        except Exception:
                            pass

                self.api_client.send_status(status_data)

            except Exception as e:
                logger.error(f"Error in status loop: {e}")
            shutdown_event.wait(self.status_interval)

    def obstacle_loop(self):
        """Continuously poll obstacle status and alert dashboard."""
        poll_interval = float(CONFIG.get('obstacle_poll_interval', 0.15))
        emit_interval = float(CONFIG.get('obstacle_emit_interval', 0.5))
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
                should_emit = (state and not last_state) or (state and (now - last_emit) >= emit_interval)

                if should_emit:
                    dist_cm = -1
                    if obstacle and obstacle.get('distance_cm') is not None:
                        try:
                            dist_cm = int(obstacle.get('distance_cm'))
                        except Exception:
                            pass

                    payload = {
                        'device_id': self.device_id,
                        'type': 'OBSTACLE_DETECTED',
                        'distance_cm': dist_cm,
                        'direction': 'FRONT',
                        'timestamp': datetime.utcnow().isoformat(),
                    }
                    try:
                        resp = self.api_client.send_event(payload)
                        last_emit = now
                        if (resp and resp.get('ai_triggered')
                                and self.nav_controller
                                and self.nav_controller.is_active):
                            self.nav_controller.notify_ai_triggered()
                    except Exception:
                        pass

                last_state = state
            except Exception:
                pass
            shutdown_event.wait(poll_interval)

    def command_loop(self):
        """Poll dashboard for control commands"""
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
        """High-frequency poll for all dashboard commands (100ms)."""
        import requests as _requests
        base_url = (
            self.api_client.base_url.rstrip('/')
            if getattr(self.api_client, 'base_url', None)
            else CONFIG.get('dashboard_api', {}).get('base_url', '').rstrip('/')
        )
        url = f"{base_url}/api/commands/instant"
        session = _requests.Session()
        api_timeout = CONFIG.get('dashboard_api', {}).get('timeout', 10)
        poll_timeout = (min(api_timeout, 5), min(api_timeout, 5))
        poll_interval = CONFIG.get('instant_poll_interval', 0.15)

        while not shutdown_event.is_set():
            try:
                resp = session.get(url, timeout=poll_timeout)
                if resp.status_code == 200:
                    data = resp.json()
                    commands = data.get('commands', [])
                    for cmd in commands:
                        self._process_command({
                            'id': None,
                            'command_type': cmd.get('command_type', '?'),
                            'payload': cmd.get('payload', {}),
                        })
            except Exception:
                pass
            shutdown_event.wait(poll_interval)

    def _process_command(self, command: dict) -> None:
        """Dispatch a command dict to the appropriate handler."""
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
                self._ai_resume_epoch += 1
                success = self.nav_controller.start() if self.nav_controller else False
            elif command_type == 'NAV_PAUSE':
                self._ai_resume_epoch += 1
                if self.nav_controller:
                    self.nav_controller.pause()
                    success = True
            elif command_type == 'NAV_RESUME':
                self._ai_resume_epoch += 1
                if self.nav_controller:
                    self.nav_controller.resume()
                    success = True
            elif command_type == 'NAV_STOP':
                self._ai_resume_epoch += 1
                if self.nav_controller:
                    self.nav_controller.stop()
                    success = True
            elif command_type == 'MANUAL_DRIVE':
                direction = (payload or {}).get('direction', '').lower()
                if direction not in ('', 'stop', 'brake', 'none'):
                    if self.nav_controller and self.nav_controller.is_active:
                        self.nav_controller.pause()
                success = self._handle_manual_drive(payload)
            elif command_type == 'NAV_RETURN_HOME':
                self._ai_resume_epoch += 1
                success = self.nav_controller.return_home() if self.nav_controller else False
            elif command_type == 'WAYPOINT_PUSH':
                success = self._handle_waypoint_push()
            elif command_type == 'ENGAGE_WIRELESS':
                engage = payload.get('engage', False)
                if self.robot_link and hasattr(self.robot_link, 'engage_wireless_control'):
                    success = self.robot_link.engage_wireless_control(engage)
                    if success:
                        self.wireless_backup_active = engage
            elif command_type == 'AI_DRIVE':
                ai_dir = (payload.get('direction') or 'stop').upper()
                ai_safety = (payload.get('safety') or 'DANGER').upper()
                ai_reason = payload.get('reason', '')
                if (self.nav_controller and self.nav_controller.is_active
                        and hasattr(self.nav_controller, 'receive_ai_advice')):
                    self.nav_controller.receive_ai_advice(ai_dir, ai_safety, ai_reason)
                    success = True
            else:
                error_message = f"Unknown command {command_type}"

        except Exception as exc:
            logger.exception(f"Command execution error: {exc}")
            success = False
            error_message = str(exc)
        finally:
            if command_id is not None:
                self.api_client.ack_command(
                    command_id,
                    status='completed' if success else 'failed',
                    error_message=error_message
                )

    def _handle_manual_drive(self, payload: dict) -> bool:
        direction = (payload or {}).get('direction', '').lower()
        speed = max(60, min(255, int((payload or {}).get('speed', 180))))
        if not hasattr(self.robot_link, 'manual_drive'):
            return False
        requested = (direction or 'stop').strip().lower()
        if requested in ('', 'none'):
            requested = 'stop'
        with self._manual_drive_lock:
            if requested in ('stop', 'brake'):
                self._manual_drive_active = False
                self._manual_drive_direction = 'stop'
            else:
                if self._manual_drive_active and requested == self._manual_drive_direction:
                    self._manual_drive_active = False
                    self._manual_drive_direction = 'stop'
                else:
                    self._manual_drive_active = True
                    self._manual_drive_direction = requested
            self._manual_drive_speed = speed
        if not self._manual_drive_thread or not self._manual_drive_thread.is_alive():
            self._manual_drive_thread = Thread(target=self._manual_drive_loop, daemon=True)
            self._manual_drive_thread.start()
        try:
            return self._manual_drive_apply_once()
        except Exception:
            return False

    def _manual_drive_apply_once(self) -> bool:
        if not self.robot_link or not hasattr(self.robot_link, 'manual_drive'):
            return False
        with self._manual_drive_lock:
            active = self._manual_drive_active
            direction = self._manual_drive_direction
            speed = self._manual_drive_speed
        if not active:
            return bool(self.robot_link.manual_drive('stop', speed))
        return bool(self.robot_link.manual_drive(direction, speed))

    def _manual_drive_loop(self) -> None:
        """Refresh manual drive commands at 5Hz to prevent Mega timeout."""
        while not shutdown_event.is_set():
            try:
                if self.robot_link and hasattr(self.robot_link, 'manual_drive'):
                    with self._manual_drive_lock:
                        active = self._manual_drive_active
                    if active:
                        self._manual_drive_apply_once()
            except Exception:
                pass
            shutdown_event.wait(0.2)

    def _handle_waypoint_push(self) -> bool:
        """Load waypoints from dashboard DB into Pi NavController."""
        waypoints = self.api_client.get_waypoints()
        if not waypoints:
            return False
        waypoints = sorted(waypoints, key=lambda w: w.get('sequence', 0))
        if self.nav_controller:
            self.nav_controller.set_waypoints(waypoints)
            return True
        return False

    def nav_status_loop(self):
        """Broadcast Pi-side navigation status to dashboard at 2 Hz."""
        idle_counter = 0
        while not shutdown_event.is_set():
            try:
                status = None
                idle_counter += 1
                if self.nav_controller:
                    is_active = self.nav_controller.is_active
                    if is_active or idle_counter >= 2:
                        idle_counter = 0
                        status = self.nav_controller.get_status()
                elif self.compass and idle_counter >= 2:
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
                        'distance_to_wp': None,
                        'current_wp_index': 0,
                        'total_waypoints': 0,
                    }

                if status is not None:
                    ts = datetime.utcnow().isoformat()
                    status['device_id'] = self.device_id
                    status['timestamp'] = ts
                    try:
                        self.api_client.send_event({'device_id': self.device_id, 'type': 'NAV_STATUS', 'nav': status, 'timestamp': ts})
                    except Exception:
                        pass
            except Exception:
                pass
            shutdown_event.wait(0.5)

    def camera_loop(self):
        """Capture and relay camera frames at configured FPS."""
        if not self.camera or not self.camera.enabled:
            return
        max_frame_bytes = int(CONFIG.get('camera', {}).get('max_frame_size', 2 * 1024 * 1024))
        fps = max(1, int(CONFIG.get('camera', {}).get('fps', 5)))
        base_url = self.api_client.base_url.rstrip('/')
        interval = 1.0 / float(fps)
        next_capture = time.time()
        try:
            import requests
            session = requests.Session()
            url = f"{base_url}/api/camera/frame_binary"
            while not shutdown_event.is_set():
                if time.time() >= next_capture:
                    next_capture = time.time() + interval
                    try:
                        jpeg = self.camera.capture_frame_jpeg()
                        if jpeg and len(jpeg) <= max_frame_bytes:
                            mjpeg_update_frame(jpeg)
                            session.post(url, params={'device_id': self.device_id, 'timestamp': datetime.utcnow().isoformat()}, data=jpeg, headers={'Content-Type': 'image/jpeg'}, timeout=(1.5, 2.5))
                    except Exception:
                        pass
                shutdown_event.wait(0.01)
        except Exception as exc:
            logger.error("Camera loop error: %s", exc)

    def get_battery_level(self):
        """Get battery level from sysfs, INA219, or ADS1115 voltage divider."""
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
                    adc_voltage = raw * (4.096 / 32767.0)
                    ratio = vs_cfg.get('voltage_divider_ratio', 5.0)
                    v_in = adc_voltage * ratio
                    v_min = vs_cfg.get('v_min', 6.0)
                    v_max = vs_cfg.get('v_max', 8.4)
                    if v_min <= v_in <= (v_max + 1.0):
                        pct = ((v_in - v_min) / (v_max - v_min)) * 100.0
                        return round(max(0.0, min(100.0, pct)), 1)
        except Exception:
            pass
        try:
            import glob
            for bat_path in sorted(glob.glob('/sys/class/power_supply/BAT*/capacity')):
                with open(bat_path, 'r') as f:
                    return round(float(f.read().strip()), 1)
        except Exception:
            pass
        return -1.0

    def get_cpu_temperature(self):
        """Get Raspberry Pi CPU temperature from sysfs."""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return 0.0

    def shutdown(self):
        """Gracefully shutdown all systems."""
        logger.info("Shutting down robot controller...")
        shutdown_event.set()
        try:
            self.api_client.send_status({'online': False, 'device_id': self.device_id})
        except Exception:
            pass
        try:
            if self.mjpeg_server:
                self.mjpeg_server.stop()
            if self.camera and hasattr(self.camera, 'stop'):
                self.camera.stop()
            if self.robot_link and hasattr(self.robot_link, 'close'):
                self.robot_link.close()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        for thread in self.threads:
            thread.join(timeout=2)
        logger.info("Shutdown complete")


def signal_handler(signum, frame):
    logger.info(f"Received signal {signum}, initiating shutdown...")
    shutdown_event.set()


def main():
    logger.info("=" * 60)
    logger.info("Environmental Monitoring Robot - Raspberry Pi")
    logger.info("=" * 60)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    robot = None
    try:
        robot = RobotController()
        if robot.start():
            logger.info("Robot is running. Press Ctrl+C to stop.")
            while not shutdown_event.is_set():
                time.sleep(1)
        else:
            return 1
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        return 1
    finally:
        if robot:
            robot.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

---

## Appendix B: Arduino Mega Navigation Firmware Source Code

**File:** `arduino_mega/robot_navigation/robot_navigation.ino`

**Description:** This is the complete Arduino Mega 2560 firmware. It implements the three-state finite state machine (STATE_I2C / STATE_WIRELESS / STATE_FAILSAFE), the eleven-step non-blocking superloop, the I2C slave ISR and deferred command handler (17 commands), the CC1101 radio polling driver, obstacle avoidance safety guard, non-blocking buzzer, GPS response builder, I2C bus watchdog with 9-clock recovery, and all state transition logic. The firmware never uses `delay()` or blocking I/O in the main loop.

---

```cpp
/*
 * Environmental Monitoring Robot — Navigation Controller (Arduino Mega 2560)
 *
 * Three-state architecture:
 *   STATE_I2C       (AUTO)     – Raspberry Pi owns the motors
 *   STATE_WIRELESS             – ESP8266 CC1101 remote owns the motors
 *   STATE_FAILSAFE             – Both comms lost, motors halted
 *
 * Superloop order:
 *  1. Deferred CC1101 init       6. Buzzer tick
 *  2. SPI Service (CC1101 poll)  7. I2C bus watchdog
 *  3. I2C Service (deferred ISR) 8. Decision Layer
 *  4. MODE MANAGER               9. Obstacle buzzer
 *  5. Sensor Tasks              10. Status print  11. GPS broadcast
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <SPI.h>

#include "gps_handler.h"
#include "navigation.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "globals.h"
#include "cc1101_driver.h"

CC1101Driver wireless;
#include "wireless_interface.h"

GPSHandler gps;
Navigation navigation;
MotorControl motors;
ObstacleAvoidance obstacleAvoid;

RobotState robotState = STATE_I2C;

static int8_t lastManualLeft  = 0;
static int8_t lastManualRight = 0;

uint8_t responseBuffer[32];
uint8_t responseLength = 0;

PendingWaypoint pendingWaypoints[MAX_WAYPOINTS];
uint8_t pendingWaypointCount = 0;

bool navigationActive        = false;
bool manualOverride          = false;
bool i2cHandshakeComplete    = false;
bool wirelessHandshakeComplete = false;

volatile bool     i2cCommandPending   = false;
volatile uint8_t  i2cPendingCommand   = 0;
volatile uint8_t  i2cPendingLength    = 0;
uint8_t           i2cPendingPayload[32];
volatile unsigned long lastI2CActivityMs = 0;

unsigned long lastStatusUpdate   = 0;
unsigned long lastManualCommand  = 0;
unsigned long lastHeartbeat      = 0;
unsigned long lastCC1101Poll     = 0;

int manualSpeed = 180;
ControlMode controlMode = MODE_AUTO;
float piHeading = 0.0;

static uint8_t  buzzerPulsesLeft = 0;
static uint16_t buzzerOnMs       = 0;
static uint16_t buzzerOffMs      = 0;
static bool     buzzerToneOn     = false;
static unsigned long buzzerPhaseStart = 0;

void onI2CReceive(int bytes);
void onI2CRequest();
void handleI2CCommand(uint8_t cmd, const uint8_t* payload, uint8_t len);
void prepareAck(uint8_t code = 0);
void prepareError(uint8_t code);
void prepareGpsResponse();
void prepareStatusResponse();
void resetPendingWaypoints();
void storePendingWaypoint(const WaypointPacket& pkt);
void commitPendingWaypoints();
void transitionToState(RobotState newState);
void pollCC1101();
void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags);
void processWirelessCommand(uint8_t cmd, uint8_t speed);
void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void updateBuzzer();

static inline bool frontObstacle() {
  const int d = obstacleAvoid.getDistance();
  return (d > 0 && d < OBSTACLE_THRESHOLD);
}

void recoverI2CBus() {
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(50);
  if (digitalRead(SDA) == LOW) {
    for (uint8_t i = 0; i < 9; i++) {
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }
  }
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

bool isI2CBusStuck() {
  uint8_t twsr = TWSR & 0xF8;
  if (twsr == 0x00) return true;
  if (twsr != 0xF8 && twsr != 0x60 && twsr != 0x80 && twsr != 0x88 &&
      twsr != 0xA8 && twsr != 0xB8 && twsr != 0xC0 && twsr != 0xC8)
    return true;
  return false;
}

void i2cReinitSlave() {
  Wire.end();
  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  GPS_SERIAL.begin(GPS_BAUD);

  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  gps.begin(GPS_SERIAL);
  motors.begin();
  obstacleAvoid.begin();
  navigation.begin(&gps, &motors, &obstacleAvoid);

  tone(BUZZER_PIN, BUZZER_FREQ);
  delay(3000);
  noTone(BUZZER_PIN);
}

void loop() {
  const unsigned long now = millis();

  // 1. Deferred CC1101 init
  static bool cc1101Ready = false;
  if (!cc1101Ready && now > 5000) {
    cc1101Ready = true;
    wireless.begin();
  }

  // 2. SPI Service — poll CC1101
  if (cc1101Ready && (now - lastCC1101Poll >= CC1101_POLL_INTERVAL)) {
    lastCC1101Poll = now;
    pollCC1101();
  }

  // CC1101 RX health check (every 2 seconds)
  {
    static unsigned long lastRxCheck = 0;
    if (cc1101Ready && (now - lastRxCheck >= 2000)) {
      lastRxCheck = now;
      wireless.ensureRxMode();
    }
  }

  // 3. I2C Service — deferred ISR command
  if (i2cCommandPending) {
    uint8_t cmd, len;
    uint8_t payload[32];
    noInterrupts();
    cmd = i2cPendingCommand;
    len = i2cPendingLength;
    if (len > 32) len = 32;
    memcpy(payload, (const void*)i2cPendingPayload, len);
    i2cCommandPending = false;
    interrupts();
    handleI2CCommand(cmd, payload, len);
  }

  // 4. MODE MANAGER
  {
    const bool wirelessAlive = cc1101Ready &&
                               wireless.getLastRxTime() > 0 &&
                               (now - wireless.getLastRxTime() <= WIRELESS_LINK_TIMEOUT);
    const bool piAlive = i2cHandshakeComplete &&
                         (now - lastI2CActivityMs <= I2C_LINK_TIMEOUT);

    if (wirelessAlive) {
      if (robotState != STATE_WIRELESS) transitionToState(STATE_WIRELESS);
    } else if (piAlive) {
      if (robotState == STATE_WIRELESS || robotState == STATE_FAILSAFE)
        transitionToState(STATE_I2C);
    } else if (i2cHandshakeComplete || wireless.getLastRxTime() > 0) {
      if (robotState != STATE_FAILSAFE) transitionToState(STATE_FAILSAFE);
    }
  }

  // 5. Sensor Tasks
  gps.update();
  obstacleAvoid.update();

  // Safety: emergency obstacle stop for non-nav forward motion
  if (obstacleAvoid.isObstacleDetected() && !navigationActive) {
    if (motors.isMovingForward()) {
      motors.stop();
    }
  }

  // 6. Buzzer tick
  updateBuzzer();

  // 7. I2C bus watchdog
  if (i2cHandshakeComplete) {
    static unsigned long lastWatchdog = 0;
    if (now - lastWatchdog > 5000) {
      lastWatchdog = now;
      if (isI2CBusStuck()) i2cReinitSlave();
    }
  }

  // 8. Decision Layer
  switch (robotState) {
    case STATE_I2C:
      if (!manualOverride && navigationActive) {
        if (gps.isValid()) {
          navigation.update();
          if (navigation.isComplete()) {
            navigationActive = false;
            motors.stop();
          }
        } else {
          static unsigned long lastGpsWarn = 0;
          if (now - lastGpsWarn > 5000) {
            motors.stop();
            lastGpsWarn = now;
          }
        }
      }
      if (manualOverride && (now - lastManualCommand > MANUAL_TIMEOUT)) {
        manualOverride = false;
        motors.stop();
        controlMode = MODE_AUTO;
        if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
          navigationActive = true;
          navigation.resume();
        }
      }
      break;

    case STATE_WIRELESS:
      if (frontObstacle() && lastManualLeft > 0 && lastManualRight > 0) {
        motors.stop();
        lastManualLeft = 0;
        lastManualRight = 0;
      }
      break;

    case STATE_FAILSAFE:
      motors.stop();
      {
        static unsigned long lastFailsafeBeep = 0;
        if (now - lastFailsafeBeep >= FAILSAFE_BEEP_INTERVAL) {
          beepPatternNB(2, 300, 200);
          lastFailsafeBeep = now;
        }
      }
      break;
  }

  // 9. Obstacle buzzer
  if (frontObstacle()) {
    static unsigned long lastObstacleBeep = 0;
    if (now - lastObstacleBeep > 2000) {
      beepPatternNB(3, 100, 80);
      lastObstacleBeep = now;
    }
  }

  // 10. Status print
  if (now - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = now;
    DEBUG_SERIAL.print(F("# ST="));
    switch (robotState) {
      case STATE_I2C:      DEBUG_SERIAL.print(F("AUTO"));     break;
      case STATE_WIRELESS: DEBUG_SERIAL.print(F("WIRELESS")); break;
      case STATE_FAILSAFE: DEBUG_SERIAL.print(F("FAILSAFE")); break;
    }
    DEBUG_SERIAL.print(F(" nav="));
    DEBUG_SERIAL.println(navigationActive ? F("RUN") : F("IDLE"));
  }
}

void transitionToState(RobotState newState) {
  if (newState == robotState) return;
  motors.stop();
  lastManualLeft  = 0;
  lastManualRight = 0;
  switch (robotState) {
    case STATE_WIRELESS:
      manualOverride = false;
      controlMode = MODE_AUTO;
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigationActive = true;
        navigation.resume();
      }
      break;
    case STATE_I2C:
      if (navigationActive) { navigation.pause(); navigationActive = false; }
      break;
    case STATE_FAILSAFE:
      break;
  }
  robotState = newState;
  switch (newState) {
    case STATE_I2C:
      break;
    case STATE_WIRELESS:
      manualOverride = true;
      controlMode = MODE_MANUAL;
      wirelessHandshakeComplete = true;
      beepPatternNB(1, 150, 0);
      break;
    case STATE_FAILSAFE:
      manualOverride = false;
      controlMode = MODE_AUTO;
      navigationActive = false;
      navigation.stop();
      beepPatternNB(5, 100, 100);
      break;
  }
}

void pollCC1101() {
  WirelessMessage msg;
  if (!wireless.receive(msg)) return;
  switch (msg.type) {
    case MSG_TYPE_RAW_MOTOR:
      if (msg.length >= sizeof(RawMotorPacket) && robotState == STATE_WIRELESS) {
        RawMotorPacket* pkt = (RawMotorPacket*)msg.data;
        processRawMotorCommand(pkt->throttle, pkt->steer, pkt->flags);
      }
      break;
    case MSG_TYPE_COMMAND:
      if (msg.length >= 2 && robotState == STATE_WIRELESS)
        processWirelessCommand(msg.data[0], msg.data[1]);
      break;
    case MSG_TYPE_HANDSHAKE:
      wirelessHandshakeComplete = true;
      break;
    default:
      break;
  }
}

void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags) {
  lastManualCommand = millis();
  int left  = constrain(throttle + steer, -255, 255);
  int right = constrain(throttle - steer, -255, 255);
  if (abs(left)  < 15) left  = 0;
  if (abs(right) < 15) right = 0;
  if (frontObstacle() && left > 0 && right > 0) {
    motors.stop();
  } else {
    motors.setMotors(left, right);
  }
  lastManualLeft  = (int8_t)constrain(left,  -127, 127);
  lastManualRight = (int8_t)constrain(right, -127, 127);
}

void processWirelessCommand(uint8_t cmd, uint8_t speed) {
  lastManualCommand = millis();
  switch (cmd) {
    case WIRELESS_CMD_MOTOR_FORWARD:  motors.forward(speed);   break;
    case WIRELESS_CMD_MOTOR_BACKWARD: motors.backward(speed);  break;
    case WIRELESS_CMD_MOTOR_LEFT:     motors.turnLeft(speed);  break;
    case WIRELESS_CMD_MOTOR_RIGHT:    motors.turnRight(speed); break;
    case WIRELESS_CMD_MOTOR_STOP:     motors.stop();           break;
    case WIRELESS_CMD_MODE_AUTO:      transitionToState(STATE_I2C); break;
    default: break;
  }
}

void onI2CReceive(int bytes) {
  lastI2CActivityMs = millis();
  if (bytes <= 0) return;
  uint8_t cmd = Wire.read();
  uint8_t len = bytes - 1;
  if (len > 32) len = 32;
  for (uint8_t i = 0; i < len; i++)
    i2cPendingPayload[i] = Wire.available() ? Wire.read() : 0;
  i2cPendingCommand = cmd;
  i2cPendingLength  = len;
  i2cCommandPending = true;
}

void onI2CRequest() {
  lastI2CActivityMs = millis();
  if (responseLength > 0 && responseLength <= sizeof(responseBuffer))
    Wire.write(responseBuffer, responseLength);
  else
    Wire.write((uint8_t)0);
}

void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length) {
  if (robotState == STATE_WIRELESS || robotState == STATE_FAILSAFE) {
    switch (command) {
      case CMD_PING: case CMD_HEARTBEAT: case CMD_REQUEST_GPS:
      case CMD_REQUEST_STATUS: case CMD_REQUEST_OBSTACLE: case CMD_SOUND_BUZZER:
      case CMD_SET_AUTO_SPEED: case CMD_SEND_HEADING: case CMD_EMERGENCY_STOP:
        break;
      default: prepareAck(); return;
    }
  }

  switch (command) {
    case CMD_PING:
      i2cHandshakeComplete = true;
      beepPatternNB(2, 100, 100);
      prepareAck();
      break;
    case CMD_NAV_START:
      if (navigation.getWaypointCount() > 0) {
        navigationActive = true;
        navigation.start();
        manualOverride = false;
        prepareAck();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;
    case CMD_NAV_PAUSE:
      navigation.pause();
      navigationActive = false;
      motors.stop();
      prepareAck();
      break;
    case CMD_NAV_RESUME:
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigation.resume();
        navigationActive = true;
        prepareAck();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;
    case CMD_WAYPOINT_CLEAR:
      navigation.clearWaypoints();
      resetPendingWaypoints();
      navigationActive = false;
      motors.stop();
      prepareAck();
      break;
    case CMD_WAYPOINT_PACKET:
      if (length >= 11) {
        WaypointPacket pkt;
        pkt.id  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        pkt.seq = payload[2];
        memcpy(&pkt.latitude,  &payload[3], 4);
        memcpy(&pkt.longitude, &payload[7], 4);
        storePendingWaypoint(pkt);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    case CMD_WAYPOINT_COMMIT:
      commitPendingWaypoints();
      prepareAck();
      break;
    case CMD_NAV_STOP:
      navigation.stop();
      navigationActive = false;
      motors.stop();
      prepareAck();
      break;
    case CMD_REQUEST_GPS:
      prepareGpsResponse();
      break;
    case CMD_REQUEST_STATUS:
      prepareStatusResponse();
      break;
    case CMD_HEARTBEAT:
      lastHeartbeat = millis();
      prepareAck();
      break;
    case CMD_SEND_GPS:
      if (length >= 16) {
        float lat, lon, spd, hdg;
        memcpy(&lat, &payload[0],  sizeof(float));
        memcpy(&lon, &payload[4],  sizeof(float));
        memcpy(&spd, &payload[8],  sizeof(float));
        memcpy(&hdg, &payload[12], sizeof(float));
        gps.seedPosition((double)lat, (double)lon);
        navigation.updateGpsData(lat, lon, spd, hdg);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    case CMD_SEND_HEADING:
      if (length >= 4) {
        memcpy(&piHeading, &payload[0], sizeof(float));
        navigation.setHeading(piHeading);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    case CMD_MANUAL_OVERRIDE:
      if (length >= 3) {
        int8_t leftMotor  = (int8_t)payload[0];
        int8_t rightMotor = (int8_t)payload[1];
        bool active       = payload[2] != 0;
        if (active && navigationActive) { navigationActive = false; navigation.pause(); }
        if (active && frontObstacle() && leftMotor > 0 && rightMotor > 0) {
          motors.stop();
        } else {
          motors.setMotors((int)leftMotor, (int)rightMotor);
        }
        manualOverride = true;
        lastManualCommand = millis();
        lastManualLeft  = leftMotor;
        lastManualRight = rightMotor;
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    case CMD_EMERGENCY_STOP:
      motors.stop();
      navigationActive = false;
      navigation.stop();
      beepPatternNB(5, 50, 50);
      prepareAck();
      break;
    case CMD_REQUEST_OBSTACLE: {
      int dist = obstacleAvoid.getDistance();
      bool obs = (dist > 0 && dist < OBSTACLE_THRESHOLD);
      responseBuffer[0] = RESP_OBSTACLE;
      responseBuffer[1] = obs ? 1 : 0;
      if (dist < 0) dist = 0;
      responseBuffer[2] = (dist >> 8) & 0xFF;
      responseBuffer[3] = dist & 0xFF;
      responseLength = 4;
      break;
    }
    case CMD_SOUND_BUZZER:
      if (length >= 1) {
        uint8_t secs = payload[0];
        if (secs > 0 && secs <= 10) beepPatternNB(secs, 900, 100);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    default:
      prepareError(0xFE);
      break;
  }
}

void prepareAck(uint8_t code)   { responseBuffer[0] = RESP_ACK;   responseBuffer[1] = code; responseLength = 2; }
void prepareError(uint8_t code) { responseBuffer[0] = RESP_ERROR; responseBuffer[1] = code; responseLength = 2; }

void prepareGpsResponse() {
  responseBuffer[0] = RESP_GPS;
  if (!gps.isValid()) { responseBuffer[1] = 0; responseLength = 2; return; }
  responseBuffer[1] = 1;
  float lat = gps.getLatitude(), lon = gps.getLongitude(), spd = gps.getSpeed();
  float hdg = piHeading;
  uint8_t sat = gps.getSatellites();
  memcpy(&responseBuffer[2],  &lat, 4);
  memcpy(&responseBuffer[6],  &lon, 4);
  memcpy(&responseBuffer[10], &spd, 4);
  memcpy(&responseBuffer[14], &hdg, 4);
  responseBuffer[18] = sat;
  responseLength = 19;
}

void prepareStatusResponse() {
  responseBuffer[0] = RESP_STATUS;
  responseBuffer[1] = (controlMode == MODE_AUTO) ? 0 : 1;
  responseBuffer[2] = navigationActive ? 1 : 0;
  responseBuffer[3] = manualOverride   ? 1 : 0;
  responseBuffer[4] = navigation.getWaypointCount();
  responseBuffer[5] = 85;   // battery placeholder
  responseBuffer[6] = 0;    // signal placeholder
  responseBuffer[7] = navigation.getCurrentWaypointIndex();
  responseBuffer[8] = navigation.isWaypointJustCompleted() ? 1 : 0;
  if (navigation.isWaypointJustCompleted()) navigation.clearWaypointCompletionFlag();
  responseLength = 9;
}

void resetPendingWaypoints() { pendingWaypointCount = 0; }

void storePendingWaypoint(const WaypointPacket& pkt) {
  if (pendingWaypointCount >= MAX_WAYPOINTS) return;
  PendingWaypoint slot;
  slot.latitude  = pkt.latitude;
  slot.longitude = pkt.longitude;
  slot.id  = pkt.id;
  slot.seq = pkt.seq;
  pendingWaypoints[pendingWaypointCount++] = slot;
}

void commitPendingWaypoints() {
  navigation.clearWaypoints();
  for (uint8_t i = 0; i < pendingWaypointCount; ++i)
    navigation.addWaypoint(pendingWaypoints[i].latitude, pendingWaypoints[i].longitude, pendingWaypoints[i].id);
  resetPendingWaypoints();
}

void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  buzzerPulsesLeft = pulses;
  buzzerOnMs  = onMs;
  buzzerOffMs = offMs;
  buzzerToneOn = true;
  buzzerPhaseStart = millis();
  tone(BUZZER_PIN, BUZZER_FREQ);
}

void updateBuzzer() {
  if (buzzerPulsesLeft == 0) return;
  unsigned long elapsed = millis() - buzzerPhaseStart;
  if (buzzerToneOn) {
    if (elapsed >= buzzerOnMs) {
      noTone(BUZZER_PIN);
      buzzerToneOn = false;
      buzzerPhaseStart = millis();
      if (--buzzerPulsesLeft == 0) return;
    }
  } else {
    if (elapsed >= buzzerOffMs) {
      tone(BUZZER_PIN, BUZZER_FREQ);
      buzzerToneOn = true;
      buzzerPhaseStart = millis();
    }
  }
}

uint8_t readSignalQuality() {
  int8_t rssi = wireless.getRSSI();
  if (rssi == 0) return 0;
  if (rssi > -50) return 100;
  if (rssi < -90) return 10;
  return (rssi + 100) * 2;
}
```

---

## Appendix C: Navigation Controller Source Code

**File:** `raspberry_pi/navigation/nav_controller.py`

**Description:** This module implements the Pi-side `NavController` class. It runs the eleven-state autonomous navigation state machine (IDLE → PREPARING → ACQUIRING_HEADING → HEADING_ACQUIRED → NAVIGATING → OBSTACLE_DETECTED → OBSTACLE_AVOID → WAYPOINT_REACHED → COMPLETE / PAUSED) at 10 Hz in a daemon thread. All geodesic calculations use GeographicLib Karney WGS-84 (with Haversine spherical fallback). Motor commands are sent to the Arduino Mega via I2C. The controller supports AI Vision obstacle avoidance advice, user-initiated heading confirmation, return-to-start, and motor trim compensation.

---

```python
"""
Pi-Side Autonomous Navigation Controller
==========================================
Runs the full navigation state machine on the Raspberry Pi.
Sends simple drive commands to Arduino Mega over I2C.
Geodesic calculations use GeographicLib (Karney WGS-84).
Falls back to spherical haversine if library unavailable.
"""

import math
import time
import logging
import threading
from enum import Enum, auto
from typing import Optional, Dict, List, Any

try:
    from geographiclib.geodesic import Geodesic
    _geod = Geodesic.WGS84
    _HAS_GEOGRAPHICLIB = True
except ImportError:
    _geod = None
    _HAS_GEOGRAPHICLIB = False

logger = logging.getLogger("nav_controller")


class NavState(Enum):
    IDLE               = auto()
    PREPARING           = auto()
    ACQUIRING_HEADING   = auto()
    HEADING_ACQUIRED    = auto()
    NAVIGATING          = auto()
    OBSTACLE_DETECTED   = auto()
    OBSTACLE_AVOID      = auto()
    WAYPOINT_REACHED    = auto()
    COMPLETE            = auto()
    PAUSED              = auto()


class NavController:
    """Pi-side autonomous navigation engine."""

    WAYPOINT_RADIUS      = 3.0
    HEADING_DEADBAND     = 5.0
    ACQUIRE_SLOW_THRESH  = 15.0
    ROTATION_SPEED_FAST  = 150
    ROTATION_SPEED_SLOW  = 100
    DRIVE_SPEED          = 150
    OBSTACLE_TURN_TIME   = 1.0
    OBSTACLE_CHECK_PAUSE = 0.3
    COURSE_DRIFT_THRESH  = 12.0
    NAV_GRACE_PERIOD     = 2.0
    HEADING_ACQUIRE_TIMEOUT = 30.0
    HEADING_HOLD_TIME    = 10.0
    AI_ADVICE_TIMEOUT    = 10.0
    PREPARE_TIME         = 3.0
    WAYPOINT_HOLD_TIME   = 3.0
    NAV_LOOP_HZ         = 10

    def __init__(self, compass, robot_link, gps_provider, config: dict = None):
        self.compass = compass
        self.robot_link = robot_link
        self._gps_provider = gps_provider
        self._mega_gps_provider = None
        self._notification = None
        self._gps_wait_logged = False
        self._neo_satellites = 0
        self._waypoints: List[Dict[str, Any]] = []
        self._current_wp_index = 0
        self._state = NavState.IDLE
        self._state_entry_time = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._obstacle_phase = 'stop'
        self._obstacle_phase_start = 0.0
        self._avoid_direction = 'left'
        self._avoid_attempts = 0
        self._current_heading = 0.0
        self._current_heading_magnetic = 0.0
        self._target_bearing = 0.0
        self._last_heading_error = 0.0
        self._ai_advice_event = threading.Event()
        self._ai_advice_direction = None
        self._ai_advice_safety = None
        self._ai_analysis_triggered = False
        self._heading_confirmed = threading.Event()
        self._heading_confirm_result = None
        self._event_callback = None
        self._distance_to_wp = 0.0
        self._completed_waypoints: List[int] = []
        cfg = config or {}
        nav_cfg = cfg.get('navigation', {})
        self._motor_trim_left  = int(nav_cfg.get('motor_trim_left', 0))
        self._motor_trim_right = int(nav_cfg.get('motor_trim_right', 0))
        self.DRIVE_SPEED = int(nav_cfg.get('drive_speed', self.DRIVE_SPEED))
        self.WAYPOINT_RADIUS = float(nav_cfg.get('waypoint_radius', self.WAYPOINT_RADIUS))

    @property
    def state(self) -> NavState:
        with self._lock:
            return self._state

    @property
    def is_active(self) -> bool:
        return self._running and self._state not in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED)

    def get_status(self) -> dict:
        heading = self._current_heading
        if not self._running and self.compass:
            try:
                h = self.compass.read_heading()
                if h is not None:
                    heading = h
            except Exception:
                pass
        with self._lock:
            countdown = None
            if self._state == NavState.HEADING_ACQUIRED:
                countdown = max(0, round(self.HEADING_HOLD_TIME - (time.time() - self._state_entry_time)))
            elif self._state == NavState.PREPARING:
                countdown = max(0, round(self.PREPARE_TIME - (time.time() - self._state_entry_time)))
            elif self._state == NavState.WAYPOINT_REACHED:
                countdown = max(0, round(self.WAYPOINT_HOLD_TIME - (time.time() - self._state_entry_time)))
            notif = self._notification
            self._notification = None
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
                'countdown': countdown,
                'notification': notif,
                'neo_satellites': self._neo_satellites,
            }

    def return_home(self):
        with self._lock:
            if not self._waypoints:
                return False
            self._waypoints = list(reversed(self._waypoints))
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._avoid_attempts = 0
            self._avoid_direction = 'left'
        self._running = True
        self._send_stop()
        self._enter_state(NavState.PREPARING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        return True

    def set_waypoints(self, waypoints: List[Dict[str, Any]]):
        with self._lock:
            self._waypoints = list(waypoints)
            self._current_wp_index = 0
            self._completed_waypoints = []

    def start(self):
        with self._lock:
            if not self._waypoints:
                return False
            self._current_wp_index = 0
            self._completed_waypoints = []
            self._avoid_attempts = 0
            self._avoid_direction = 'left'
        self._running = True
        self._send_stop()
        self._enter_state(NavState.PREPARING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()
        return True

    def stop(self):
        self._running = False
        self._send_stop()
        self._enter_state(NavState.IDLE)

    def pause(self):
        self._send_stop()
        self._enter_state(NavState.PAUSED)

    def resume(self):
        if self._state != NavState.PAUSED:
            return
        self._running = True
        self._send_stop()
        self._enter_state(NavState.PREPARING)
        if not self._thread or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._nav_loop, daemon=True)
            self._thread.start()

    def accept_heading(self):
        with self._lock:
            if self._state not in (NavState.PREPARING, NavState.ACQUIRING_HEADING):
                return False
        self._send_stop()
        self._send_stop()
        self._heading_acquired_emitted = False
        self._enter_state(NavState.HEADING_ACQUIRED)
        return True

    def confirm_heading(self, accepted: bool = True):
        self._heading_confirm_result = accepted
        self._heading_confirmed.set()
        return True

    def set_event_callback(self, callback):
        self._event_callback = callback

    def receive_ai_advice(self, direction: str, safety: str, reason: str = ''):
        self._ai_advice_direction = direction.upper() if direction else 'STOP'
        self._ai_advice_safety = safety.upper() if safety else 'DANGER'
        self._ai_advice_event.set()

    def notify_ai_triggered(self):
        self._ai_analysis_triggered = True

    def _emit_event(self, event_type: str, payload: dict = None):
        if self._event_callback:
            try:
                self._event_callback(event_type, payload or {})
            except Exception:
                pass

    def _nav_loop(self):
        interval = 1.0 / self.NAV_LOOP_HZ
        while self._running:
            try:
                self._tick()
            except Exception as e:
                logger.error("Nav tick error: %s", e, exc_info=True)
                self._send_stop()
            time.sleep(interval)

    def _tick(self):
        state = self.state
        if state in (NavState.IDLE, NavState.COMPLETE, NavState.PAUSED):
            return

        self._current_heading = self._read_heading()

        if state == NavState.PREPARING:
            self._handle_preparing()
            return

        gps = self._get_gps()
        if not gps:
            if not getattr(self, '_gps_wait_logged', False):
                self._gps_wait_logged = True
                self._notification = {'level': 'warning', 'msg': 'Waiting for GPS fix...'}
            return

        with self._lock:
            if self._current_wp_index >= len(self._waypoints):
                self._enter_state(NavState.COMPLETE)
                self._send_stop()
                return
            wp = self._waypoints[self._current_wp_index]

        self._gps_wait_logged = False
        wp_lat = float(wp['latitude'])
        wp_lon = float(wp['longitude'])
        cur_lat, cur_lon = gps['latitude'], gps['longitude']

        self._distance_to_wp = self._haversine(cur_lat, cur_lon, wp_lat, wp_lon)
        self._target_bearing = self._bearing(cur_lat, cur_lon, wp_lat, wp_lon)
        self._last_heading_error = self._normalize_error(self._target_bearing - self._current_heading)

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

    def _handle_preparing(self):
        elapsed = time.time() - self._state_entry_time
        if int(elapsed) != getattr(self, '_prep_last_sec', -1):
            self._prep_last_sec = int(elapsed)
            self._send_stop()
        if elapsed >= self.PREPARE_TIME:
            self._enter_state(NavState.ACQUIRING_HEADING)

    def _handle_acquiring_heading(self):
        error = self._last_heading_error
        abs_error = abs(error)
        if time.time() - self._state_entry_time > self.HEADING_ACQUIRE_TIMEOUT:
            self._enter_state(NavState.NAVIGATING)
            return
        if abs_error <= self.HEADING_DEADBAND:
            self._send_stop()
            self._send_stop()
            self._heading_acquired_emitted = False
            self._enter_state(NavState.HEADING_ACQUIRED)
            return
        speed = self.ROTATION_SPEED_SLOW if abs_error < self.ACQUIRE_SLOW_THRESH else self.ROTATION_SPEED_FAST
        if error > 0:
            self._send_rotate_right(speed)
        else:
            self._send_rotate_left(speed)

    def _handle_heading_acquired(self):
        elapsed = time.time() - self._state_entry_time
        if not getattr(self, '_heading_acquired_emitted', False):
            self._heading_acquired_emitted = True
            self._heading_confirmed.clear()
            self._heading_confirm_result = None
            self._emit_event('heading_acquired', {
                'heading': round(self._current_heading, 1),
                'target_bearing': round(self._target_bearing, 1),
                'heading_error': round(self._last_heading_error, 1),
            })
        tick_num = int(elapsed * self.NAV_LOOP_HZ)
        if tick_num % self.NAV_LOOP_HZ == 0:
            self._send_stop()
        if self._heading_confirmed.is_set():
            self._heading_acquired_emitted = False
            if self._heading_confirm_result is False:
                self._enter_state(NavState.ACQUIRING_HEADING)
            else:
                self._enter_state(NavState.NAVIGATING)
            return
        if elapsed >= self.HEADING_HOLD_TIME:
            self._heading_acquired_emitted = False
            self._enter_state(NavState.NAVIGATING)

    def _handle_navigating(self):
        if self._distance_to_wp <= self.WAYPOINT_RADIUS:
            self._send_stop()
            self._send_stop()
            self._notification = {'level': 'success', 'msg': f'Waypoint {self._current_wp_index + 1}/{len(self._waypoints)} reached!'}
            self._enter_state(NavState.WAYPOINT_REACHED)
            return
        if self._check_obstacle():
            self._send_stop()
            self._ai_advice_event.clear()
            self._ai_advice_direction = None
            self._ai_advice_safety = None
            self._enter_state(NavState.OBSTACLE_DETECTED)
            return
        nav_elapsed = time.time() - self._state_entry_time
        if nav_elapsed >= self.NAV_GRACE_PERIOD:
            if abs(self._last_heading_error) > self.COURSE_DRIFT_THRESH:
                self._send_stop()
                self._enter_state(NavState.ACQUIRING_HEADING)
                return
        self._send_forward(self.DRIVE_SPEED)

    def _handle_obstacle_detected(self):
        elapsed = time.time() - self._state_entry_time
        if elapsed < self.OBSTACLE_CHECK_PAUSE:
            self._send_stop()
            return
        if self._ai_advice_event.is_set():
            direction = self._ai_advice_direction or 'STOP'
            safety = self._ai_advice_safety or 'DANGER'
            if direction == 'FORWARD' and safety in ('SAFE', 'CAUTION'):
                self._enter_state(NavState.ACQUIRING_HEADING)
                return
            if direction in ('LEFT', 'RIGHT') and safety != 'DANGER':
                self._avoid_attempts += 1
                self._avoid_direction = direction.lower()
                self._obstacle_phase = 'turn'
                self._obstacle_phase_start = time.time()
                self._enter_state(NavState.OBSTACLE_AVOID)
                return
            self._enter_traditional_avoidance()
            return
        if self._ai_analysis_triggered:
            if elapsed < self.AI_ADVICE_TIMEOUT:
                self._send_stop()
                return
            self._enter_traditional_avoidance()
            return
        self._enter_traditional_avoidance()

    def _enter_traditional_avoidance(self):
        self._avoid_attempts += 1
        self._avoid_direction = 'left' if self._avoid_attempts % 2 == 1 else 'right'
        self._obstacle_phase = 'turn'
        self._obstacle_phase_start = time.time()
        self._enter_state(NavState.OBSTACLE_AVOID)

    def _handle_obstacle_avoid(self):
        elapsed = time.time() - self._obstacle_phase_start
        if self._obstacle_phase == 'turn':
            if elapsed < self.OBSTACLE_TURN_TIME:
                if self._avoid_direction == 'left':
                    self._send_rotate_left(self.ROTATION_SPEED_FAST)
                else:
                    self._send_rotate_right(self.ROTATION_SPEED_FAST)
            else:
                self._send_stop()
                self._obstacle_phase = 'recheck'
                self._obstacle_phase_start = time.time()
        elif self._obstacle_phase == 'recheck':
            if elapsed >= self.OBSTACLE_CHECK_PAUSE:
                if self._check_obstacle():
                    self._obstacle_phase = 'turn'
                    self._obstacle_phase_start = time.time()
                    if self._avoid_attempts > 5:
                        self._advance_waypoint()
                else:
                    self._send_stop()
                    self._enter_state(NavState.PREPARING)

    def _handle_waypoint_reached(self):
        elapsed = time.time() - self._state_entry_time
        if int(elapsed) != getattr(self, '_wr_last_sec', -1):
            self._wr_last_sec = int(elapsed)
            self._send_stop()
        if elapsed >= self.WAYPOINT_HOLD_TIME:
            self._advance_waypoint()

    def _send_forward(self, speed: int):
        self._send_motor_command(speed + self._motor_trim_left, speed + self._motor_trim_right)

    def _send_rotate_left(self, speed: int):
        self._send_motor_command(-speed, speed)

    def _send_rotate_right(self, speed: int):
        self._send_motor_command(speed, -speed)

    def _send_stop(self):
        self._send_motor_command(0, 0)

    def _send_motor_command(self, left: int, right: int):
        try:
            if self.robot_link and hasattr(self.robot_link, 'send_manual_control'):
                self.robot_link.send_manual_control(left, right, joystick_active=True)
        except Exception as e:
            logger.debug("Motor command failed: %s", e)

    def _read_heading(self) -> float:
        try:
            if hasattr(self.compass, 'read_heading_magnetic'):
                self._current_heading_magnetic = self.compass.read_heading_magnetic()
            return self.compass.read_heading()
        except Exception:
            return self._current_heading

    def _get_gps(self) -> Optional[dict]:
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
        if self._mega_gps_provider:
            try:
                mega = self._mega_gps_provider()
                if mega and mega.get('valid') and mega.get('latitude') and mega.get('longitude'):
                    self._neo_satellites = mega.get('satellites', 0)
                    return {'latitude': mega['latitude'], 'longitude': mega['longitude'], 'speed': mega.get('speed', 0)}
            except Exception:
                pass
        return None

    def _check_obstacle(self) -> bool:
        try:
            if self.robot_link and hasattr(self.robot_link, 'request_obstacle_status'):
                result = self.robot_link.request_obstacle_status()
                if result:
                    return result.get('obstacle', False)
        except Exception:
            pass
        return False

    def _enter_state(self, new_state: NavState):
        with self._lock:
            old = self._state
            self._state = new_state
            self._state_entry_time = time.time()
        if old != new_state:
            logger.info("NAV: %s → %s", old.name, new_state.name)
        if new_state not in (NavState.OBSTACLE_DETECTED, NavState.OBSTACLE_AVOID):
            self._ai_analysis_triggered = False

    def _advance_waypoint(self):
        with self._lock:
            if self._current_wp_index < len(self._waypoints):
                wp = self._waypoints[self._current_wp_index]
                self._completed_waypoints.append(wp.get('id', self._current_wp_index))
                self._current_wp_index += 1
            if self._current_wp_index >= len(self._waypoints):
                self._send_stop()
                self._notification = {'level': 'success', 'msg': 'All waypoints complete!'}
                self._enter_state(NavState.COMPLETE)
                self._running = False
                return
        self._avoid_attempts = 0
        self._send_stop()
        self._enter_state(NavState.PREPARING)

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2) -> float:
        if _HAS_GEOGRAPHICLIB:
            return _geod.Inverse(lat1, lon1, lat2, lon2)['s12']
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def _bearing(lat1, lon1, lat2, lon2) -> float:
        if _HAS_GEOGRAPHICLIB:
            return _geod.Inverse(lat1, lon1, lat2, lon2)['azi1'] % 360.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
        return math.degrees(math.atan2(x, y)) % 360.0

    @staticmethod
    def _normalize_error(error: float) -> float:
        while error > 180:  error -= 360
        while error < -180: error += 360
        return error
```

---

## Appendix D: Gas Calibration Module Source Code

**File:** `raspberry_pi/utils/gas_calibration.py`

**Description:** This module implements the `GasCalibration` class, which converts raw 16-bit ADS1115 ADC values to calibrated gas concentrations in parts per million (PPM) for five sensor/gas combinations (MQ-2 Smoke, MQ-2 LPG, MQ-135 CO₂, MQ-135 NH₃, MQ-7 CO). The calibration model is PPM = a × (Rs/R0)^b, where Rs is the sensor resistance computed from the output voltage and load resistor, and R0 is the estimated baseline resistance in clean air. All constants are derived from the manufacturer datasheet log-log characteristic curves.

---

```python
"""
Gas Sensor PPM Calibration for MQ Series Sensors
Converts ADS1115 raw ADC values to gas concentrations
"""

import logging

logger = logging.getLogger('gas_calibration')


class GasCalibration:
    """Calibrate MQ sensor raw values to PPM concentrations"""

    # ADS1115 settings
    ADS1115_MAX_VALUE    = 32767   # 15-bit single-ended
    ADS1115_VOLTAGE_RANGE = 6.144  # PGA 2/3 gain: ±6.144 V range
    VOLTS_PER_BIT = ADS1115_VOLTAGE_RANGE / ADS1115_MAX_VALUE

    # Hardware configuration
    RL    = 1000.0  # Load resistor in Ohms (typical MQ breakout board)
    V_IN  = 5.0     # Supply voltage

    # Sensor calibration constants from datasheets
    # Format: { 'Gas': [a, b, ratio_in_clean_air] }
    # PPM = a * (Rs/R0)^b  where R0 = Rs / ratio_in_clean_air
    CALIB = {
        'MQ2_Smoke':    [3697.4, -3.109, 9.83],   # Smoke/LPG detection
        'MQ2_LPG':      [2000.0, -2.95,  9.83],   # LPG specific
        'MQ135_CO2':    [110.47, -2.862, 3.60],   # CO2 / air quality
        'MQ135_NH3':    [102.2,  -2.473, 3.60],   # Ammonia
        'MQ7_CO':       [99.04,  -1.518, 27.5],   # Carbon monoxide
    }

    @classmethod
    def raw_to_voltage(cls, raw_adc: int) -> float:
        """Convert ADS1115 raw value to voltage."""
        return raw_adc * cls.VOLTS_PER_BIT

    @classmethod
    def calculate_rs(cls, volts: float) -> float:
        """Calculate sensor resistance from output voltage.
        Rs = ((V_in * RL) / V_out) - RL
        """
        if volts <= 0:
            return float('inf')
        return ((cls.V_IN * cls.RL) / volts) - cls.RL

    @classmethod
    def calculate_ppm(cls, raw_adc: int, sensor_type: str) -> float:
        """
        Calculate PPM concentration from raw ADC value.

        Args:
            raw_adc:     Raw ADC value from ADS1115
            sensor_type: One of 'MQ2_Smoke', 'MQ2_LPG', 'MQ135_CO2',
                         'MQ135_NH3', 'MQ7_CO'
        Returns:
            Gas concentration in PPM (parts per million)
        """
        if raw_adc <= 0:
            return 0.0

        calib = cls.CALIB.get(sensor_type)
        if not calib:
            logger.error(f"Unknown sensor type: {sensor_type}")
            return 0.0

        a, b, ratio_clean_air = calib

        volts = cls.raw_to_voltage(raw_adc)
        if volts <= 0:
            return 0.0

        rs  = cls.calculate_rs(volts)
        r0  = rs / ratio_clean_air       # baseline resistance in clean air
        ppm = a * ((rs / r0) ** b)       # power-law PPM model

        return round(ppm, 2)

    @classmethod
    def get_all_concentrations(cls, mq2_raw: int, mq135_raw: int, mq7_raw: int) -> dict:
        """
        Calculate all gas concentrations from raw sensor values.

        Returns:
            Dictionary with raw ADC values, voltages, and calculated PPM
            for all five sensor/gas combinations.
        """
        return {
            # Raw ADC values (for cloud upload / debugging)
            'mq2_raw':   mq2_raw,
            'mq135_raw': mq135_raw,
            'mq7_raw':   mq7_raw,

            # Calibrated PPM concentrations
            'mq2_smoke_ppm':  cls.calculate_ppm(mq2_raw,   'MQ2_Smoke'),
            'mq2_lpg_ppm':    cls.calculate_ppm(mq2_raw,   'MQ2_LPG'),
            'mq135_co2_ppm':  cls.calculate_ppm(mq135_raw, 'MQ135_CO2'),
            'mq135_nh3_ppm':  cls.calculate_ppm(mq135_raw, 'MQ135_NH3'),
            'mq7_co_ppm':     cls.calculate_ppm(mq7_raw,   'MQ7_CO'),

            # Voltage readings (for diagnostics)
            'mq2_volts':   round(cls.raw_to_voltage(mq2_raw),   3),
            'mq135_volts': round(cls.raw_to_voltage(mq135_raw), 3),
            'mq7_volts':   round(cls.raw_to_voltage(mq7_raw),   3),
        }

    @classmethod
    def get_sensor_status(cls, raw_adc: int) -> str:
        """Get human-readable sensor status from raw ADC value."""
        if raw_adc < 500:
            return "Very Low (Check wiring)"
        elif raw_adc < 2000:
            return "Clean"
        elif raw_adc < 10000:
            return "Normal"
        elif raw_adc < 20000:
            return "Elevated"
        else:
            return "High Alert"


def calculate_gas_concentrations(mq2: int, mq135: int, mq7: int) -> dict:
    """Convenience wrapper for GasCalibration.get_all_concentrations()."""
    return GasCalibration.get_all_concentrations(mq2, mq135, mq7)
```

---

*End of Appendices*
