"""
Raspberry Pi Robot Controller
Connects to the Environmental Monitoring Dashboard

This script runs on Raspberry Pi and:
- Reads sensor data (DHT, MQ sensors)
- Gets GPS location
- Captures camera frames
- Sends data to dashboard via HTTP
- Receives and executes commands from dashboard
"""

import time
import json
import base64
import logging
from datetime import datetime
import requests
import threading

# Import hardware libraries
try:
    import Adafruit_DHT
    import Adafruit_ADS1x15
    import RPi.GPIO as GPIO
    import serial
    import psutil
    from camera_stream import CameraStream
    HARDWARE_AVAILABLE = True
except ImportError as e:
    print(f"Hardware libraries not available: {e}")
    HARDWARE_AVAILABLE = False

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('robot_controller')

class RobotController:
    """Main robot controller for Raspberry Pi"""

    def __init__(self, dashboard_url="http://192.168.1.100:5000", device_id="robot_01"):
        """
        Initialize robot controller

        Args:
            dashboard_url: URL of the dashboard server
            device_id: Unique identifier for this robot
        """
        self.dashboard_url = dashboard_url.rstrip('/')
        self.device_id = device_id
        self.running = False

        # Hardware initialization
        self.camera = None
        self.gps_serial = None
        self.adc = None

        if HARDWARE_AVAILABLE:
            self._init_hardware()

        # Sensor pins (adjust based on your wiring)
        self.DHT_PIN = 4  # GPIO pin for DHT sensor
        self.MQ_PINS = {
            'mq2': 0,   # ADC channel for MQ2
            'mq135': 1, # ADC channel for MQ135
            'mq7': 2    # ADC channel for MQ7
        }

        logger.info(f"Robot controller initialized for device: {device_id}")

    def _init_hardware(self):
        """Initialize hardware components"""
        try:
            # Initialize ADC for MQ sensors
            self.adc = Adafruit_ADS1x15.ADS1115()

            # Initialize camera
            camera_config = {
                'resolution': (640, 480),
                'fps': 10,
                'rotation': 0
            }
            self.camera = CameraStream(camera_config)

            # Initialize GPS serial (adjust port as needed)
            try:
                self.gps_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
                logger.info("GPS serial initialized")
            except:
                logger.warning("GPS serial not available")

            # Initialize GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.DHT_PIN, GPIO.IN)

            logger.info("Hardware initialized successfully")

        except Exception as e:
            logger.error(f"Hardware initialization failed: {e}")

    def read_sensors(self):
        """Read all sensor data"""
        data = {
            'timestamp': datetime.now().isoformat(),
            'device_id': self.device_id
        }

        if not HARDWARE_AVAILABLE:
            # Return mock data for testing
            data.update({
                'temperature': 25.5,
                'humidity': 60.0,
                'mq2': 150,
                'mq135': 200,
                'mq7': 100
            })
            return data

        try:
            # Read DHT sensor
            humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, self.DHT_PIN)
            if temperature is not None:
                data['temperature'] = round(temperature, 1)
            if humidity is not None:
                data['humidity'] = round(humidity, 1)

            # Read MQ sensors via ADC
            for sensor_name, channel in self.MQ_PINS.items():
                try:
                    value = self.adc.read_adc(channel, gain=1)
                    data[sensor_name] = value
                except:
                    data[sensor_name] = 0

        except Exception as e:
            logger.error(f"Error reading sensors: {e}")

        return data

    def read_gps(self):
        """Read GPS data"""
        if not self.gps_serial:
            # Return mock GPS data for testing
            return {
                'latitude': 40.7128,
                'longitude': -74.0060,
                'altitude': 10.5,
                'speed': 1.2,
                'heading': 180.0,
                'satellites': 8,
                'timestamp': datetime.now().isoformat(),
                'device_id': self.device_id
            }

        # GPS parsing would go here - simplified for example
        # You would parse NMEA sentences from GPS module
        return None

    def capture_camera_frame(self):
        """Capture and encode camera frame"""
        if not self.camera or not self.camera.enabled:
            return None

        try:
            frame = self.camera.capture_frame()
            if frame is not None:
                # Encode frame as base64
                import cv2
                _, buffer = cv2.imencode('.jpg', frame)
                frame_b64 = base64.b64encode(buffer).decode('utf-8')

                return {
                    'frame': frame_b64,
                    'timestamp': datetime.now().isoformat(),
                    'device_id': self.device_id
                }
        except Exception as e:
            logger.error(f"Error capturing camera frame: {e}")

        return None

    def get_system_status(self):
        """Get system status information"""
        try:
            return {
                'online': True,
                'battery': 85.5,  # You'd read actual battery level
                'signal_strength': -75,  # WiFi signal strength
                'system_info': {
                    'cpu': psutil.cpu_percent(),
                    'memory': psutil.virtual_memory().percent,
                    'disk': psutil.disk_usage('/').percent
                },
                'device_id': self.device_id
            }
        except:
            return {
                'online': True,
                'battery': 0,
                'device_id': self.device_id
            }

    def send_data_to_dashboard(self, endpoint, data):
        """Send data to dashboard via HTTP POST"""
        try:
            url = f"{self.dashboard_url}{endpoint}"
            response = requests.post(url, json=data, timeout=10)

            if response.status_code == 200:
                logger.debug(f"Data sent successfully to {endpoint}")
                return True
            else:
                logger.error(f"Failed to send data to {endpoint}: {response.status_code}")
                return False

        except Exception as e:
            logger.error(f"Error sending data to dashboard: {e}")
            return False

    def get_pending_commands(self):
        """Get pending commands from dashboard"""
        try:
            url = f"{self.dashboard_url}/api/commands/pending"
            params = {'device_id': self.device_id}
            response = requests.get(url, params=params, timeout=10)

            if response.status_code == 200:
                data = response.json()
                return data.get('commands', [])
            else:
                logger.error(f"Failed to get commands: {response.status_code}")
                return []

        except Exception as e:
            logger.error(f"Error getting commands: {e}")
            return []

    def acknowledge_command(self, command_id, status='completed', error_message=None):
        """Acknowledge command execution"""
        try:
            url = f"{self.dashboard_url}/api/commands/{command_id}/ack"
            data = {'status': status}
            if error_message:
                data['error_message'] = error_message

            response = requests.post(url, json=data, timeout=10)
            return response.status_code == 200

        except Exception as e:
            logger.error(f"Error acknowledging command: {e}")
            return False

    def execute_command(self, command):
        """Execute a received command"""
        command_type = command.get('command_type')
        payload = command.get('payload', {})

        logger.info(f"Executing command: {command_type}")

        try:
            if command_type == 'move':
                # Implement movement logic here
                direction = payload.get('direction')
                speed = payload.get('speed', 0)
                # Your motor control code here
                logger.info(f"Moving {direction} at speed {speed}")

            elif command_type == 'stop':
                # Stop all movement
                logger.info("Stopping robot")

            elif command_type == 'goto_waypoint':
                # Navigate to waypoint
                waypoint_id = payload.get('waypoint_id')
                logger.info(f"Navigating to waypoint {waypoint_id}")

            elif command_type == 'set_mode':
                # Change robot mode
                mode = payload.get('mode')
                logger.info(f"Setting mode to {mode}")

            # Acknowledge successful execution
            self.acknowledge_command(command['id'], 'completed')

        except Exception as e:
            logger.error(f"Error executing command {command_type}: {e}")
            self.acknowledge_command(command['id'], 'failed', str(e))

    def run(self):
        """Main control loop"""
        self.running = True
        logger.info("Starting robot control loop")

        sensor_interval = 5  # Send sensor data every 5 seconds
        gps_interval = 10    # Send GPS data every 10 seconds
        camera_interval = 2  # Send camera frames every 2 seconds
        status_interval = 30 # Send status every 30 seconds
        command_interval = 3 # Check for commands every 3 seconds

        last_sensor = 0
        last_gps = 0
        last_camera = 0
        last_status = 0
        last_command = 0

        try:
            while self.running:
                current_time = time.time()

                # Send sensor data
                if current_time - last_sensor >= sensor_interval:
                    sensor_data = self.read_sensors()
                    self.send_data_to_dashboard('/api/sensor_data', sensor_data)
                    last_sensor = current_time

                # Send GPS data
                if current_time - last_gps >= gps_interval:
                    gps_data = self.read_gps()
                    if gps_data:
                        self.send_data_to_dashboard('/api/gps_data', gps_data)
                    last_gps = current_time

                # Send camera frame
                if current_time - last_camera >= camera_interval:
                    camera_data = self.capture_camera_frame()
                    if camera_data:
                        self.send_data_to_dashboard('/api/camera/frame', camera_data)
                    last_camera = current_time

                # Send status
                if current_time - last_status >= status_interval:
                    status_data = self.get_system_status()
                    self.send_data_to_dashboard('/api/status', status_data)
                    last_status = current_time

                # Check for commands
                if current_time - last_command >= command_interval:
                    commands = self.get_pending_commands()
                    for command in commands:
                        # Execute commands in separate thread to avoid blocking
                        threading.Thread(target=self.execute_command, args=(command,)).start()
                    last_command = current_time

                time.sleep(1)  # Small delay to prevent CPU hogging

        except KeyboardInterrupt:
            logger.info("Robot control loop stopped by user")
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
        finally:
            self.cleanup()

    def stop(self):
        """Stop the robot controller"""
        self.running = False
        logger.info("Stopping robot controller")

    def cleanup(self):
        """Cleanup resources"""
        if self.camera:
            self.camera.stop()
        if HARDWARE_AVAILABLE:
            GPIO.cleanup()
        logger.info("Cleanup completed")


if __name__ == '__main__':
    # Configuration
    DASHBOARD_IP = "192.168.1.100"  # Change to your dashboard's IP
    DEVICE_ID = "robot_01"

    # Create and run controller
    controller = RobotController(
        dashboard_url=f"http://{DASHBOARD_IP}:5000",
        device_id=DEVICE_ID
    )

    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()</content>
<parameter name="filePath">/home/thewizard/RobotControl/raspberry_pi/robot_controller.py