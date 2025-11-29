"""
Configuration file for the Dashboard Server
"""

import os
from datetime import timedelta


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
    CAMERA_FRAME_RATE = 10  # FPS when relaying frames
    CAMERA_RESOLUTION = (640, 480)
    CAMERA_QUALITY = 85  # JPEG quality (0-100)
    CAMERA_STREAM_MODE = os.environ.get('CAMERA_STREAM_MODE', 'relay')  # relay | direct
    CAMERA_STREAM_URL = os.environ.get('CAMERA_STREAM_URL', '')  # direct stream endpoint
    CAMERA_ALLOW_FALLBACK = os.environ.get('CAMERA_ALLOW_FALLBACK', 'True') == 'True'
    
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
