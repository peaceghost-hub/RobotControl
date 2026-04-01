"""
Configuration file for the Dashboard Server
"""

import os
from datetime import timedelta


def _load_dotenv() -> None:
    """Read dashboard/.env into os.environ without requiring python-dotenv."""
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.env')
    if not os.path.exists(env_path):
        return

    try:
        with open(env_path, 'r', encoding='utf-8') as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                if key and key not in os.environ:
                    os.environ[key] = value
    except OSError:
        # Keep config startup non-blocking if the env file is unreadable.
        pass


_load_dotenv()


class Config:
    """Application configuration"""
    
    # Flask Configuration
    SECRET_KEY = os.environ.get('SECRET_KEY') or 'dev-secret-key-change-in-production'
    DEBUG = os.environ.get('DEBUG', 'True') == 'True'
    
    # Server Configuration
    HOST = os.environ.get('HOST', '0.0.0.0')
    PORT = int(os.environ.get('PORT', 5000))
    
    # Database Configuration
    SQLALCHEMY_DATABASE_URI = os.environ.get('DATABASE_URL') or \
        'sqlite:////home/thewizard/RobotControl/database/robot_monitor.db'
    SQLALCHEMY_TRACK_MODIFICATIONS = False
    SQLALCHEMY_ECHO = False
    
    # Session Configuration
    PERMANENT_SESSION_LIFETIME = timedelta(hours=24)
    
    # CORS Configuration
    CORS_HEADERS = 'Content-Type'
    
    # WebSocket Configuration
    SOCKETIO_MESSAGE_QUEUE = None
    SOCKETIO_ASYNC_MODE = 'threading'
    
    # API Configuration
    API_RATE_LIMIT = 100  # requests per minute
    
    # Camera Configuration
    CAMERA_FRAME_RATE = int(os.environ.get('CAMERA_FRAME_RATE', 10))  # FPS when relaying frames
    CAMERA_RESOLUTION = (
        int(os.environ.get('CAMERA_RES_WIDTH', 640)),
        int(os.environ.get('CAMERA_RES_HEIGHT', 480))
    )
    CAMERA_QUALITY = int(os.environ.get('CAMERA_QUALITY', 85))  # JPEG quality (0-100)
    CAMERA_STREAM_MODE = os.environ.get('CAMERA_STREAM_MODE', 'relay')  # relay | direct
    CAMERA_STREAM_URL = os.environ.get('CAMERA_STREAM_URL', '')  # direct stream endpoint
    CAMERA_ALLOW_FALLBACK = os.environ.get('CAMERA_ALLOW_FALLBACK', 'True') == 'True'
    # MJPEG relay served by dashboard when True
    CAMERA_MJPEG_ENABLED = os.environ.get('CAMERA_MJPEG_ENABLED', 'True') == 'True'
    CAMERA_MAX_FRAME_SIZE = int(os.environ.get('CAMERA_MAX_FRAME_SIZE', 2 * 1024 * 1024))
    
    # Robot Configuration
    ROBOT_TIMEOUT = 30  # seconds - consider robot offline if no update
    DEFAULT_DEVICE_ID = 'robot_01'
    SUPPORTED_DEVICE_IDS = os.environ.get('SUPPORTED_DEVICE_IDS', 'robot_01,esp32_01').split(',')
    
    # Map Configuration
    DEFAULT_MAP_CENTER = [0, 0]  # [latitude, longitude]
    DEFAULT_MAP_ZOOM = 15
    MAP_PROVIDER = os.environ.get('MAP_PROVIDER', 'OpenStreetMap')  # e.g. OpenStreetMap, OpenAIMaps
    MAP_TILE_URL = os.environ.get('MAP_TILE_URL', 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png')
    MAP_ATTRIBUTION = os.environ.get('MAP_ATTRIBUTION', '© OpenStreetMap contributors')
    MAP_API_KEY = os.environ.get('MAP_API_KEY', '')
    
    # Alert Thresholds
    BATTERY_LOW_THRESHOLD = 20  # percent
    TEMP_HIGH_THRESHOLD = 50  # Celsius
    TEMP_LOW_THRESHOLD = -10  # Celsius
    HUMIDITY_HIGH_THRESHOLD = 90  # percent
    GAS_ALERT_THRESHOLD = 400  # MQ sensor reading
    
    # Data Retention
    KEEP_SENSOR_DATA_DAYS = 30
    KEEP_GPS_DATA_DAYS = 30
    KEEP_LOGS_DAYS = 90
    
    # ESP32 Support
    SUPPORT_ESP32 = True
    ESP32_DEVICES = ['esp32_01', 'esp32_02']  # Add ESP32 device IDs here

    # Command queue settings
    COMMAND_RETRY_INTERVAL = 2  # seconds

    # ZigBee Backup Link
    ZIGBEE_ENABLED = os.environ.get('ZIGBEE_ENABLED', 'False') == 'True'
    ZIGBEE_PORT = os.environ.get('ZIGBEE_PORT', '/dev/ttyUSB1')
    ZIGBEE_BAUDRATE = int(os.environ.get('ZIGBEE_BAUDRATE', 115200))
    ZIGBEE_DEVICE_ID = os.environ.get('ZIGBEE_DEVICE_ID', 'robot_backup')
    ZIGBEE_FORWARD_COMMANDS = os.environ.get('ZIGBEE_FORWARD_COMMANDS', 'True') == 'True'
    BACKUP_LOCATION_TTL = int(os.environ.get('BACKUP_LOCATION_TTL', 30))  # seconds

    # USB Joystick Bridge (ESP8266 -> Dashboard backend -> Pi)
    JOYSTICK_ENABLED = os.environ.get('JOYSTICK_ENABLED', 'False') == 'True'
    JOYSTICK_PORT = os.environ.get('JOYSTICK_PORT', '/dev/ttyUSB0')
    JOYSTICK_BAUDRATE = int(os.environ.get('JOYSTICK_BAUDRATE', 115200))
    JOYSTICK_DEVICE_ID = os.environ.get('JOYSTICK_DEVICE_ID', 'esp8266_joystick')
    JOYSTICK_TIMEOUT_MS = int(os.environ.get('JOYSTICK_TIMEOUT_MS', 1000))
    JOYSTICK_READ_TIMEOUT_MS = int(os.environ.get('JOYSTICK_READ_TIMEOUT_MS', 150))
    JOYSTICK_CONNECT_GRACE_MS = int(os.environ.get('JOYSTICK_CONNECT_GRACE_MS', 1200))
    JOYSTICK_STALE_MS = int(os.environ.get('JOYSTICK_STALE_MS', 2500))
    JOYSTICK_DEADBAND = int(os.environ.get('JOYSTICK_DEADBAND', 18))
    JOYSTICK_FORWARD_INTERVAL_MS = int(os.environ.get('JOYSTICK_FORWARD_INTERVAL_MS', 120))
    JOYSTICK_STEER_GAIN = float(os.environ.get('JOYSTICK_STEER_GAIN', 1.0))
