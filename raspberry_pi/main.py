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

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import custom modules
from raspberry_pi.sensors.sensor_manager import SensorManager
from raspberry_pi.communication.gsm_module import GSMModule
from raspberry_pi.communication.sim7600e_gps import SIM7600EGPS
from raspberry_pi.communication.serial_comm import ArduinoComm
from raspberry_pi.communication.i2c_comm import I2CComm
from raspberry_pi.communication.cloud_uploader import CloudUploader
from raspberry_pi.communication.api_client import DashboardAPI
from raspberry_pi.camera.camera_stream import CameraStream
from raspberry_pi.utils.logger import setup_logger
from raspberry_pi.utils.data_formatter import DataFormatter

# Configuration
try:
    with open('config.json', 'r') as f:
        CONFIG = json.load(f)
except FileNotFoundError:
    # Try relative to this file's directory
    import os
    config_path = os.path.join(os.path.dirname(__file__), 'config.json')
    try:
        with open(config_path, 'r') as f:
            CONFIG = json.load(f)
    except FileNotFoundError:
        print("Warning: config.json not found, using default configuration")
        CONFIG = {
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
        }

# Setup logging
logger = setup_logger('main', CONFIG.get('log_level', 'INFO'))

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
        
        # GSM Module (optional - legacy, prefer SIM7600E)
        self.gsm = None
        try:
            if CONFIG.get('gsm') and not CONFIG.get('sim7600e'):
                self.gsm = GSMModule(CONFIG['gsm'])
                logger.info("GSM Module initialized (legacy)")
        except Exception as e:
            logger.warning(f"GSM Module not available: {e}")
        
        # SIM7600E with GPS (optional) - if configured, takes priority over Neo-6M
        self.sim7600e = None
        self.gps_from_gsm = False
        try:
            if CONFIG.get('sim7600e'):
                self.sim7600e = SIM7600EGPS(CONFIG['sim7600e'])
                self.gps_from_gsm = True
                logger.info("SIM7600E module initialized (GPS from GSM module)")
        except Exception as e:
            logger.warning(f"SIM7600E module not available: {e}")
        
        # Arduino Mega communication (optional)
        self.comm_mode = None
        self.robot_link = None
        try:
            if CONFIG.get('i2c'):
                self.robot_link = I2CComm(CONFIG['i2c'])
                self.comm_mode = 'i2c'
                logger.info("Configured I2C communication with Mega")
                if not getattr(self.robot_link, 'bus', None):
                    logger.warning("I2C bus not initialized; Mega link may be offline")
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
        if CONFIG.get('camera', {}).get('enabled', False):
            try:
                self.camera = CameraStream(CONFIG['camera'])
                if self.camera and self.camera.enabled:
                    logger.info("Camera Stream initialized")
                else:
                    logger.warning("Camera Stream unavailable after init")
            except Exception as e:
                logger.warning(f"Camera not available: {e}")
        
        self.command_poll_interval = CONFIG.get('command_poll_interval', 2)
        
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
                        if not self.robot_link.ping():
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
            
            # Start waypoint check loop
            waypoint_thread = Thread(target=self.waypoint_loop, daemon=True)
            waypoint_thread.start()
            self.threads.append(waypoint_thread)
            # Start command polling loop
            command_thread = Thread(target=self.command_loop, daemon=True)
            command_thread.start()
            self.threads.append(command_thread)
            
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
                if not self.sensor_manager:
                    logger.debug("Sensor manager not available, skipping")
                    shutdown_event.wait(self.update_interval)
                    continue
                
                # Read sensors
                sensor_data = self.sensor_manager.read_all() if self.sensor_manager else {}
                
                # Add timestamp and device ID
                sensor_data['timestamp'] = datetime.now().isoformat()
                sensor_data['device_id'] = self.device_id
                
                # Replace None values with 0 for numeric fields
                for key in ['temperature', 'humidity', 'mq7', 'mq135']:
                    if sensor_data.get(key) is None:
                        sensor_data[key] = 0
                
                # Send to dashboard
                success = self.api_client.send_sensor_data(sensor_data)
                
                if success:
                    temp = sensor_data.get('temperature', 0)
                    if temp is not None:
                        logger.debug(f"Sensor data sent: Temp={temp:.1f}°C")
                    else:
                        logger.debug("Sensor data sent (no readings)")
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
    
    def gps_loop(self):
        """Request and transmit GPS data from Arduino or SIM7600E"""
        logger.info("Starting GPS loop...")
        
        while not shutdown_event.is_set():
            try:
                gps_data = None
                
                # Priority 1: Get GPS from SIM7600E if available (dashboard feed only by default)
                if self.sim7600e:
                    gps_data = self.sim7600e.get_gps_data()
                    if gps_data:
                        # Optionally forward to Mega if configured
                        forward_cfg = CONFIG.get('sim7600e', {}).get('forward_to_mega', False)
                        if forward_cfg and self.robot_link:
                            try:
                                self.robot_link.send_gps_data(gps_data)
                                logger.debug("Forwarded SIM7600E GPS to Mega (per config)")
                            except Exception as e:
                                logger.debug(f"Could not forward GPS to Mega: {e}")
                
                # Fallback: Request GPS data from Arduino Neo-6M
                if not gps_data and self.robot_link:
                    gps_data = self.robot_link.request_gps_data()
                
                if gps_data and gps_data.get('latitude') is not None and gps_data.get('longitude') is not None:
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

                    # Poll obstacle status and notify dashboard
                    try:
                        obstacle = self.robot_link.request_obstacle_status()
                    except Exception:
                        obstacle = None
                    if obstacle and obstacle.get('obstacle'):
                        status_data['event'] = {
                            'type': 'OBSTACLE_DETECTED',
                            'distance_cm': obstacle.get('distance_cm', -1),
                            'timestamp': datetime.utcnow().isoformat()
                        }
                        # Send immediate event notification
                        try:
                            self.api_client.send_event({
                                'device_id': self.device_id,
                                'type': 'OBSTACLE_DETECTED',
                                'distance_cm': obstacle.get('distance_cm', -1),
                                'timestamp': status_data['event']['timestamp']
                            })
                        except Exception as exc:
                            logger.debug(f"Failed to send obstacle event: {exc}")

                    if self.comm_mode == 'i2c':
                        try:
                            self.robot_link.send_heartbeat()
                        except Exception as exc:
                            logger.debug(f"Heartbeat failed: {exc}")

                success = self.api_client.send_status(status_data)

                if success:
                    logger.debug(f"Status sent: Battery={status_data['battery']:.1f}%")

            except Exception as e:
                logger.error(f"Error in status loop: {e}")

            # Wait before next update
            shutdown_event.wait(self.status_interval)
    
    def command_loop(self):
        """Poll dashboard for control commands"""
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
                success = self.robot_link.start_navigation()
            elif command_type == 'NAV_PAUSE':
                success = self.robot_link.pause_navigation()
            elif command_type == 'NAV_RESUME':
                success = self.robot_link.resume_navigation()
            elif command_type == 'NAV_STOP':
                success = self.robot_link.stop_navigation()
            elif command_type == 'MANUAL_DRIVE':
                success = self._handle_manual_drive(payload)
            elif command_type == 'MANUAL_OVERRIDE':
                success = self._handle_manual_override(payload)
            elif command_type == 'MANUAL_SPEED':
                success = self._handle_manual_speed(payload)
            elif command_type == 'WAYPOINT_PUSH':
                success = self._handle_waypoint_push()
            elif command_type == 'FOLLOW_LINE':
                # Payload: {"enabled": true|false}
                enabled = bool((payload or {}).get('enabled', True))
                if hasattr(self.robot_link, 'set_line_follow'):
                    success = self.robot_link.set_line_follow(enabled)
                else:
                    success = False
            elif command_type == 'FOLLOW_LINE_OFF':
                if hasattr(self.robot_link, 'set_line_follow'):
                    success = self.robot_link.set_line_follow(False)
                else:
                    success = False
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
            self.api_client.ack_command(command_id, status=status, error_message=error_message)

    def _handle_manual_drive(self, payload: dict) -> bool:
        direction = (payload or {}).get('direction', '').lower()
        speed = int((payload or {}).get('speed', 180))
        speed = max(60, min(255, speed))

        if not hasattr(self.robot_link, 'manual_drive'):
            logger.warning("Manual drive not supported by current comm link")
            return False

        logger.info(f"Manual drive command: {direction} @ {speed}")
        return self.robot_link.manual_drive(direction, speed)

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

    def _handle_waypoint_push(self) -> bool:
        waypoints = self.api_client.get_waypoints()
        if not waypoints:
            return False
        waypoints = sorted(waypoints, key=lambda w: w.get('sequence', 0))
        return self.robot_link.send_waypoints(waypoints)
    
    def camera_loop(self):
        """Capture and stream camera frames (optimized)"""
        if not self.camera or not self.camera.enabled:
            logger.info("Camera not enabled; skipping camera loop")
            return

        logger.info("Starting camera loop (optimized JPEG capture)...")
        max_frame_bytes = int(CONFIG.get('camera', {}).get('max_frame_size', 2 * 1024 * 1024))
        base_url = self.api_client.base_url.rstrip('/') if getattr(self.api_client, 'base_url', None) else CONFIG.get('dashboard_api', {}).get('base_url', '').rstrip('/')

        while not shutdown_event.is_set():
            try:
                # Capture JPEG bytes directly
                jpeg_bytes = self.camera.capture_frame_jpeg()
                if not jpeg_bytes:
                    logger.debug("No JPEG captured")
                else:
                    # Safety check on frame size
                    if len(jpeg_bytes) > max_frame_bytes:
                        logger.warning("Captured frame too large (%d bytes) - skipping", len(jpeg_bytes))
                    else:
                        # Send raw JPEG bytes to dashboard binary endpoint to avoid base64 overhead
                        # POST binary data with Content-Type image/jpeg
                        try:
                            import requests
                            url = f"{base_url}/api/camera/frame_binary"
                            params = {
                                'device_id': self.device_id,
                                'timestamp': datetime.utcnow().isoformat()
                            }
                            headers = {'Content-Type': 'image/jpeg'}
                            resp = requests.post(url, params=params, data=jpeg_bytes, headers=headers, timeout=5)
                            if resp.status_code not in (200, 202):
                                logger.debug("Server returned %s for frame upload", resp.status_code)
                        except Exception as exc:
                            logger.debug("Failed to upload binary frame: %s", exc)
            except Exception as e:
                logger.exception("Error in camera loop: %s", e)

            # Sleep according to configured fps (respect shutdown_event)
            shutdown_event.wait(1.0 / max(1, int(CONFIG.get('camera', {}).get('fps', 5))))

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