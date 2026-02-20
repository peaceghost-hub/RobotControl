from __future__ import annotations

"""
Web Dashboard for ARM-Based Environmental Monitoring Robot
Flask Backend Server

This server handles:
- Sensor data reception and storage
- Live camera feed streaming (MJPEG and WebSocket)
- GPS location tracking
- Waypoint management
- Robot status monitoring
- WebSocket communication for real-time updates
"""

from collections import deque
from datetime import datetime
import json
import logging
from threading import Lock
import base64
from typing import Optional
import time
import sys
import os

# Add parent directory to path for utils import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from raspberry_pi.utils.gas_calibration import GasCalibration

try:
    from flask import Flask, render_template, request, jsonify, Response, stream_with_context
    from flask_socketio import SocketIO, emit
    from flask_cors import CORS
    FLASK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Flask dependencies not available. Install with: pip install -r requirements.txt")
    print(f"Error: {e}")
    FLASK_AVAILABLE = False
    # Define dummy classes/functions for development
    class ConfigDict:
        def from_object(self, obj): pass
    class Flask:
        def __init__(self, *args, **kwargs): 
            self.config = ConfigDict()
        def route(self, *args, **kwargs): return lambda f: f
        def run(self, *args, **kwargs): pass
        def before_first_request(self, *args, **kwargs): return lambda f: f
    class SocketIO:
        def __init__(self, *args, **kwargs): pass
        def run(self, *args, **kwargs): pass
        def on(self, *args, **kwargs): return lambda f: f
    def emit(*args, **kwargs): pass
    class CORS:
        def __init__(self, *args, **kwargs): pass

try:
    from database import db, SensorReading, GPSLocation, Waypoint, RobotStatus, RobotCommand
except ImportError:
    try:
        from .database import db, SensorReading, GPSLocation, Waypoint, RobotStatus, RobotCommand
    except ImportError:
        print("Warning: Could not import database models")
        # Define dummy classes for development
        class db:
            def init_app(self, *args, **kwargs): pass
        class SensorReading:
            pass
        class GPSLocation:
            pass
        class Waypoint:
            pass
        class RobotStatus:
            pass
        class RobotCommand:
            pass

try:
    from config import Config
except ImportError:
    try:
        from .config import Config
    except ImportError:
        print("Warning: Could not import config")
        # Define dummy Config class
        class Config:
            SECRET_KEY = 'dummy'
            SQLALCHEMY_DATABASE_URI = 'sqlite:///dummy.db'
            SQLALCHEMY_TRACK_MODIFICATIONS = False
            PORT = 5000
            DEBUG = False

# Optional ZigBee bridge
try:
    from zigbee_bridge import ZigbeeBridge
except ImportError:  # pragma: no cover - optional dependency
    ZigbeeBridge = None

# Optional Moondream AI Vision
try:
    from ai_vision import MoondreamVision, NullVision
except ImportError:
    try:
        from .ai_vision import MoondreamVision, NullVision
    except ImportError:
        MoondreamVision = None
        NullVision = None

# Initialize Flask app
app = Flask(__name__)
app.config.from_object(Config)
CORS(app)

# Initialize SocketIO for real-time communication
# Force threading async mode to avoid eventlet/gevent on Python 3.13
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Initialize database
db.init_app(app)

# Thread lock for concurrent access
thread_lock = Lock()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Store latest data for quick access
# camera_frame will store bytes under 'frame_bytes' and base64 under 'frame_b64' for API compatibility
latest_data = {
    'sensors': {},
    'gps': {},
    'status': {
        'online': False,
        'battery': 0,
        'signal_strength': None,
        'last_update': None
    },
    'camera_frame': None,
    'backup': {
        'active': False,
        'device_id': None,
        'last_update': None,
        'last_location': None,
        'reason': None,
        'last_command': None
    },
    'nav_status': None
}

# Buffer robot events (optional, small ring buffer)
recent_events = []
MAX_EVENTS = 50

# ---- Fast Command Channel ----
# Thread-safe queue for ALL dashboard->Pi commands (manual drive, nav, waypoints).
# Bypasses DB polling; the Pi fetches at high frequency (~100ms).
# Each entry: {'command_type': str, 'payload': dict, 'device_id': str,
#              'seq': int, 'timestamp': str}
_instant_queue = deque(maxlen=50)  # bounded queue — protects memory
_instant_command_seq = 0           # monotonic counter for ordering

backup_state = {
    'active': False,
    'device_id': None,
    'last_update': None,
    'last_location': None,
    'reason': None,
    'last_command': None
}

zigbee_bridge = None

# ---- AI Vision Engine (Moondream 2) ----
# Runs on the dashboard server (GPU/CPU), NOT on the Pi.
# Model loads lazily in background on first enable — no startup delay.
if MoondreamVision is not None:
    ai_vision = MoondreamVision()
else:
    ai_vision = NullVision() if NullVision is not None else type('_NV', (), {
        'status': 'unavailable', 'enabled': False, 'is_ready': False,
        'last_result': None, 'load_model': lambda s: None,
        'unload_model': lambda s: None, 'set_enabled': lambda s, e: None,
        'set_mode': lambda s, *a: None, 'set_interval': lambda s, f: None,
        'query_once': lambda s, *a: None, 'detect_once': lambda s, *a: None,
        'get_status': lambda s: {'status': 'unavailable', 'enabled': False},
    })()

def _get_latest_frame_bytes():
    """Provide the AI engine with the latest JPEG frame."""
    with thread_lock:
        entry = latest_data.get('camera_frame')
    if entry and entry.get('frame_bytes'):
        return entry['frame_bytes']
    return None

# Wire frame provider + SocketIO emitter into the vision engine
if hasattr(ai_vision, '_frame_provider'):
    ai_vision._frame_provider = _get_latest_frame_bytes
if hasattr(ai_vision, '_emit_fn'):
    def _ai_emit(event, data):
        socketio.emit(event, data, namespace='/realtime')
    ai_vision._emit_fn = _ai_emit

# Wire drive-command sender so navigate mode can push AI_DRIVE to the Pi
if hasattr(ai_vision, '_drive_command_fn'):
    def _ai_push_drive(cmd_data):
        """Push an AI_DRIVE command into the instant queue for the Pi."""
        global _instant_command_seq
        with thread_lock:
            _instant_command_seq += 1
            seq = _instant_command_seq
            _instant_queue.append({
                'command_type': cmd_data.get('command', 'AI_DRIVE'),
                'payload': cmd_data.get('payload', {}),
                'device_id': app.config.get('DEFAULT_DEVICE_ID', 'robot_01'),
                'seq': seq,
                'timestamp': datetime.utcnow().isoformat(),
            })
        logger.info("AI_DRIVE queued: %s seq=%d",
                    cmd_data.get('payload', {}).get('direction', '?'), seq)
    ai_vision._drive_command_fn = _ai_push_drive

# Helper to set latest camera frame (bytes)
def _store_camera_frame_bytes(frame_bytes: bytes, timestamp: Optional[str] = None, device_id: Optional[str] = None):
    """Store raw jpeg bytes and precompute base64 for backward compatibility"""
    try:
        frame_b64 = base64.b64encode(frame_bytes).decode('utf-8')
    except Exception:
        frame_b64 = ''
    data = {
        'frame_bytes': frame_bytes,
        'frame_b64': frame_b64,
        'timestamp': timestamp or datetime.utcnow().isoformat(),
        'device_id': device_id or app.config.get('DEFAULT_DEVICE_ID', 'robot_01')
    }
    with thread_lock:
        latest_data['camera_frame'] = data
    return data

def _sanitize_for_json(value):
    """Best-effort conversion to JSON-serializable primitives."""
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {key: _sanitize_for_json(val) for key, val in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_sanitize_for_json(item) for item in value]
    return str(value)


def _snapshot_backup_state() -> dict:
    """Return a JSON-friendly copy of the current backup state."""
    return {
        'active': backup_state.get('active', False),
        'device_id': backup_state.get('device_id'),
        'last_update': backup_state['last_update'].isoformat() if backup_state.get('last_update') else None,
        'last_location': backup_state.get('last_location'),
        'reason': backup_state.get('reason'),
        'last_command': backup_state.get('last_command'),
        'enabled': app.config.get('ZIGBEE_ENABLED', False),
        'supportsCommands': app.config.get('ZIGBEE_FORWARD_COMMANDS', False)
    }


def set_backup_state(
    active: bool,
    *,
    location: Optional[dict] = None,
    reason: Optional[str] = None,
    device_id: Optional[str] = None,
    command: Optional[dict] = None,
    emit_event: bool = True
) -> dict:
    """Update backup activity tracking and optionally emit websocket updates."""

    previous_active = backup_state.get('active', False)
    backup_state['active'] = active

    if device_id:
        backup_state['device_id'] = device_id

    if location:
        backup_state['last_location'] = location.copy()
        backup_state['last_update'] = datetime.utcnow()
    elif active and not backup_state.get('last_update'):
        backup_state['last_update'] = datetime.utcnow()
    elif not active:
        backup_state['last_update'] = datetime.utcnow()

    if reason:
        backup_state['reason'] = reason

    if command is not None:
        command = _sanitize_for_json(command)
        if isinstance(command, dict):
            try:
                backup_state['last_command'] = json.loads(json.dumps(command))
            except TypeError:
                backup_state['last_command'] = command.copy()
        else:
            backup_state['last_command'] = command
    elif not active:
        backup_state['last_command'] = None

    snapshot = _snapshot_backup_state()

    with thread_lock:
        latest_data['backup'] = snapshot

    if emit_event and (previous_active != active or location is not None or reason is not None or command is not None):
        socketio.emit('backup_update', snapshot, namespace='/realtime')

    return snapshot


def process_backup_location(data: dict) -> dict:
    """Persist and broadcast a location update received via the backup link."""

    if not data:
        raise ValueError('Missing location payload')

    if 'latitude' not in data or 'longitude' not in data:
        raise ValueError('Latitude and longitude are required')

    device_id = data.get('device_id') or app.config.get('ZIGBEE_DEVICE_ID', 'robot_backup')

    timestamp_raw = data.get('timestamp')
    if timestamp_raw:
        try:
            timestamp = datetime.fromisoformat(timestamp_raw)
        except ValueError as exc:
            raise ValueError(f'Invalid timestamp: {timestamp_raw}') from exc
    else:
        timestamp = datetime.utcnow()

    location_record = GPSLocation(
        timestamp=timestamp,
        latitude=float(data['latitude']),
        longitude=float(data['longitude']),
        altitude=float(data.get('altitude', 0) or 0),
        speed=float(data.get('speed', 0) or 0),
        heading=float(data.get('heading', 0) or 0),
        satellites=int(data.get('satellites', 0) or 0),
        device_id=device_id,
        source='backup'
    )

    db.session.add(location_record)
    db.session.commit()

    payload = {
        'latitude': location_record.latitude,
        'longitude': location_record.longitude,
        'altitude': location_record.altitude,
        'speed': location_record.speed,
        'heading': location_record.heading,
        'satellites': location_record.satellites,
        'timestamp': location_record.timestamp.isoformat(),
        'device_id': location_record.device_id,
        'source': 'backup'
    }

    with thread_lock:
        latest_data['gps'] = payload

    socketio.emit('gps_update', payload, namespace='/realtime')
    set_backup_state(True, location=payload, device_id=device_id, reason=data.get('reason', 'zigbee_update'))

    logger.info("ZigBee backup location processed for %s", device_id)
    return payload


def handle_zigbee_message(message: dict) -> None:
    """Callback for inbound ZigBee frames."""
    if not isinstance(message, dict):
        logger.debug("Ignoring non-dict ZigBee frame: %r", message)
        return

    frame_type = message.get('type')
    if frame_type in {'gps', 'location'}:
        try:
            process_backup_location(message)
        except Exception as exc:  # pragma: no cover - defensive, hardware dependent
            logger.error("Failed to process ZigBee location: %s", exc)
    elif frame_type == 'status':
        device_id = message.get('device_id')
        if message.get('state') == 'error':
            set_backup_state(True, reason='zigbee_status_error', device_id=device_id)
        elif message.get('state') == 'recover':
            set_backup_state(False, reason='zigbee_status_recover', device_id=device_id)
        elif message.get('state') == 'hello':
            set_backup_state(True, reason='zigbee_hello', device_id=device_id, command=backup_state.get('last_command'))
    elif frame_type in {'joystick', 'control'}:
        direction = (message.get('direction') or message.get('command') or '').lower()
        raw_payload = message.get('payload') or {}
        speed_val = message.get('speed') or raw_payload.get('speed')
        try:
            speed = float(speed_val) if speed_val is not None else None
        except (TypeError, ValueError):
            speed = None

        command_payload = {
            'direction': direction or None,
            'speed': speed
        }

        if any(k in raw_payload for k in ('x', 'y', 'z')):
            command_payload['axes'] = {
                'x': raw_payload.get('x'),
                'y': raw_payload.get('y'),
                'z': raw_payload.get('z')
            }
        if raw_payload:
            command_payload['raw'] = raw_payload

        set_backup_state(True, reason='zigbee_joystick', device_id=message.get('device_id'), command=command_payload)


def initialize_zigbee() -> None:
    """Start the ZigBee bridge if enabled and available."""
    global zigbee_bridge

    if zigbee_bridge is not None:
        return

    if not app.config.get('ZIGBEE_ENABLED', False):
        logger.info("ZigBee backup link disabled in configuration")
        return

    if ZigbeeBridge is None:
        logger.warning("ZigBee bridge module not available; install pyserial to enable backup link")
        return

    bridge = ZigbeeBridge(
        port=app.config.get('ZIGBEE_PORT', '/dev/ttyUSB1'),
        baudrate=app.config.get('ZIGBEE_BAUDRATE', 115200),
        device_id=app.config.get('ZIGBEE_DEVICE_ID', 'robot_backup'),
        on_message=handle_zigbee_message
    )

    if bridge.start():
        zigbee_bridge = bridge
    else:
        zigbee_bridge = None
        logger.warning("ZigBee bridge failed to start; backup commands will require REST ingestion")


def build_frontend_config():
    """Expose selected server configuration to the frontend"""
    cfg = app.config
    return {
        'apiBaseUrl': cfg.get('API_BASE_URL', ''),
        'deviceId': cfg.get('DEFAULT_DEVICE_ID', 'robot_01'),
        'supportedDevices': cfg.get('SUPPORTED_DEVICE_IDS', []),
        'map': {
            'provider': cfg.get('MAP_PROVIDER'),
            'tileUrl': cfg.get('MAP_TILE_URL'),
            'attribution': cfg.get('MAP_ATTRIBUTION'),
            'apiKey': cfg.get('MAP_API_KEY'),
            'defaultCenter': cfg.get('DEFAULT_MAP_CENTER'),
            'defaultZoom': cfg.get('DEFAULT_MAP_ZOOM')
        },
        'camera': {
            'streamMode': cfg.get('CAMERA_STREAM_MODE', 'relay'),
            'streamUrl': cfg.get('CAMERA_STREAM_URL'),
            'allowFallback': cfg.get('CAMERA_ALLOW_FALLBACK', True),
            'frameRate': cfg.get('CAMERA_FRAME_RATE', 10),
            'maxFrameSize': cfg.get('CAMERA_MAX_FRAME_SIZE', 2 * 1024 * 1024),
            'mjpegEnabled': cfg.get('CAMERA_MJPEG_ENABLED', True)
        },
        'commands': {
            'retryInterval': cfg.get('COMMAND_RETRY_INTERVAL', 2)
        },
        'backup': {
            'enabled': cfg.get('ZIGBEE_ENABLED', False),
            'supportsCommands': cfg.get('ZIGBEE_FORWARD_COMMANDS', False),
            'deviceId': cfg.get('ZIGBEE_DEVICE_ID'),
            'ttl': cfg.get('BACKUP_LOCATION_TTL', 30)
        }
    }


# ============= WEB ROUTES =============

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html', dashboard_config=build_frontend_config())


@app.route('/api/health')
def health_check():
    """Health check endpoint"""
    return jsonify({'status': 'ok', 'timestamp': datetime.now().isoformat()})


# ============= SENSOR DATA ENDPOINTS =============

@app.route('/api/sensor_data', methods=['POST'])
def receive_sensor_data():
    """
    Receive sensor data from Raspberry Pi
    Expected JSON format:
    {
        "timestamp": "2025-11-04T12:00:00",
        "temperature": 25.5,
        "humidity": 60.0,
        "mq2": 150,
        "mq135": 200,
        "mq7": 100,
        "device_id": "robot_01"
    }
    """
    global sensor_data_enabled
    
    try:
        # Check if sensor data collection is enabled
        if not sensor_data_enabled:
            return jsonify({'status': 'paused', 'message': 'Sensor data collection is currently paused'}), 200
        
        data = request.get_json()
        logger.info(f"Received sensor data: {data}")
        
        # Calculate PPM concentrations from raw ADC values
        mq2_raw = data.get('mq2', 0) or 0
        mq135_raw = data.get('mq135', 0) or 0
        mq7_raw = data.get('mq7', 0) or 0
        
        gas_data = GasCalibration.get_all_concentrations(mq2_raw, mq135_raw, mq7_raw)
        
        # Merge raw data with calculated PPM values
        enhanced_data = {
            **data,  # Original data (raw values for cloud upload)
            **gas_data  # Calculated PPM and voltage values
        }
        
        # Store in database (raw values only)
        sensor_reading = SensorReading(
            timestamp=datetime.fromisoformat(data.get('timestamp', datetime.now().isoformat())),
            temperature=data.get('temperature'),
            humidity=data.get('humidity'),
            mq2=mq2_raw,
            mq135=mq135_raw,
            mq7=mq7_raw,
            device_id=data.get('device_id', 'robot_01')
        )
        db.session.add(sensor_reading)
        db.session.commit()
        
        # Update latest data with enhanced values (raw + PPM)
        with thread_lock:
            latest_data['sensors'] = enhanced_data
        
        # Broadcast enhanced data to connected clients via WebSocket
        socketio.emit('sensor_update', enhanced_data, namespace='/realtime')
        
        logger.debug(f"PPM values - Smoke: {gas_data['mq2_smoke_ppm']}, CO2: {gas_data['mq135_co2_ppm']}, CO: {gas_data['mq7_co_ppm']}")
        
        return jsonify({'status': 'success', 'message': 'Sensor data received'}), 200
        
    except Exception as e:
        logger.error(f"Error receiving sensor data: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/sensor_data', methods=['GET'])
def get_sensor_data():
    """
    Get historical sensor data
    Query parameters:
    - limit: number of records (default: 100)
    - device_id: filter by device
    - start_time: ISO format datetime
    - end_time: ISO format datetime
    """
    try:
        limit = request.args.get('limit', 100, type=int)
        device_id = request.args.get('device_id', 'robot_01')
        
        query = SensorReading.query.filter_by(device_id=device_id)
        
        # Apply time filters if provided
        start_time = request.args.get('start_time')
        if start_time:
            query = query.filter(SensorReading.timestamp >= datetime.fromisoformat(start_time))
        
        end_time = request.args.get('end_time')
        if end_time:
            query = query.filter(SensorReading.timestamp <= datetime.fromisoformat(end_time))
        
        readings = query.order_by(SensorReading.timestamp.desc()).limit(limit).all()
        
        data = [{
            'id': r.id,
            'timestamp': r.timestamp.isoformat(),
            'temperature': r.temperature,
            'humidity': r.humidity,
            'mq2': r.mq2,
            'mq135': r.mq135,
            'mq7': r.mq7,
            'device_id': r.device_id
        } for r in readings]
        
        return jsonify({'status': 'success', 'data': data, 'count': len(data)}), 200
        
    except Exception as e:
        logger.error(f"Error retrieving sensor data: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/sensor_data/latest', methods=['GET'])
def get_latest_sensor_data():
    """Get the most recent sensor reading"""
    with thread_lock:
        return jsonify({'status': 'success', 'data': latest_data['sensors']}), 200


# ============= GPS & LOCATION ENDPOINTS =============

@app.route('/api/gps_data', methods=['POST'])
def receive_gps_data():
    """
    Receive GPS location from Raspberry Pi
    Expected JSON format:
    {
        "latitude": 40.7128,
        "longitude": -74.0060,
        "altitude": 10.5,
        "speed": 1.2,
        "heading": 180.0,
        "timestamp": "2025-11-04T12:00:00",
        "device_id": "robot_01"
    }
    """
    try:
        data = request.get_json()
        logger.info(f"Received GPS data: {data}")
        
        # Store in database
        gps_location = GPSLocation(
            timestamp=datetime.fromisoformat(data.get('timestamp', datetime.now().isoformat())),
            latitude=data.get('latitude'),
            longitude=data.get('longitude'),
            altitude=data.get('altitude', 0),
            speed=data.get('speed', 0),
            heading=data.get('heading', 0),
            satellites=data.get('satellites', 0),
            device_id=data.get('device_id', 'robot_01'),
            source='primary'
        )
        db.session.add(gps_location)
        db.session.commit()
        
        # Update latest data
        payload = {
            'latitude': gps_location.latitude,
            'longitude': gps_location.longitude,
            'altitude': gps_location.altitude,
            'speed': gps_location.speed,
            'heading': gps_location.heading,
            'satellites': gps_location.satellites,
            'timestamp': gps_location.timestamp.isoformat(),
            'device_id': gps_location.device_id,
            'source': 'primary'
        }

        with thread_lock:
            latest_data['gps'] = payload
        
        # Broadcast to connected clients
        socketio.emit('gps_update', payload, namespace='/realtime')

        # RPi resumed primary telemetry; disable backup if it was active
        set_backup_state(False, reason='primary_gps', device_id=gps_location.device_id)
        
        return jsonify({'status': 'success', 'message': 'GPS data received'}), 200
        
    except Exception as e:
        logger.error(f"Error receiving GPS data: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/gps_data/latest', methods=['GET'])
def get_latest_gps():
    """Get the most recent GPS location"""
    with thread_lock:
        return jsonify({'status': 'success', 'data': latest_data['gps']}), 200


@app.route('/api/gps_data/track', methods=['GET'])
def get_gps_track():
    """
    Get GPS track history for displaying robot path on map
    Query parameters:
    - limit: number of points (default: 100)
    - device_id: filter by device
    """
    try:
        limit = request.args.get('limit', 100, type=int)
        device_id = request.args.get('device_id', 'robot_01')
        
        locations = GPSLocation.query.filter_by(device_id=device_id)\
            .order_by(GPSLocation.timestamp.desc())\
            .limit(limit).all()
        
        track = [{
            'latitude': loc.latitude,
            'longitude': loc.longitude,
            'timestamp': loc.timestamp.isoformat(),
            'heading': loc.heading
        } for loc in locations]
        
        return jsonify({'status': 'success', 'data': track}), 200
        
    except Exception as e:
        logger.error(f"Error retrieving GPS track: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


# ============= BACKUP LINK (ZIGBEE) =============

@app.route('/api/backup/location', methods=['POST'])
def receive_backup_location():
    """Accept location updates delivered via the ZigBee backup link."""
    try:
        data = request.get_json() or {}
        payload = process_backup_location(data)
        return jsonify({'status': 'success', 'data': payload}), 200
    except Exception as e:
        logger.error(f"Error receiving backup location: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400


@app.route('/api/backup/status', methods=['GET', 'POST'])
def backup_status():
    """Query or set the backup control status."""
    if request.method == 'POST':
        data = request.get_json() or {}
        active = bool(data.get('active', False))
        reason = data.get('reason')
        device_id = data.get('device_id')
        set_backup_state(active, reason=reason, device_id=device_id)
        logger.info("Backup status updated via API: active=%s", active)

    snapshot = _snapshot_backup_state()
    return jsonify({'status': 'success', 'data': snapshot}), 200


@app.route('/api/backup/command', methods=['POST'])
def send_backup_command():
    """Forward a control command over the ZigBee link."""
    if not app.config.get('ZIGBEE_FORWARD_COMMANDS', False):
        return jsonify({'status': 'error', 'message': 'Backup command forwarding disabled'}), 405

    payload = request.get_json() or {}
    command = payload.get('command')
    command_payload = payload.get('payload', {})

    if not command:
        return jsonify({'status': 'error', 'message': 'Missing command type'}), 400

    if zigbee_bridge is None:
        logger.warning("Received backup command %s but ZigBee bridge is not ready", command)
        return jsonify({'status': 'error', 'message': 'ZigBee bridge not available'}), 503

    try:
        zigbee_bridge.send_control(command, command_payload)
        logger.info("Forwarded backup command via ZigBee: %s", command)
        return jsonify({'status': 'success'}), 202
    except Exception as exc:  # pragma: no cover - hardware dependent
        logger.error("Failed to send ZigBee command %s: %s", command, exc)
        return jsonify({'status': 'error', 'message': str(exc)}), 500


# ============= WAYPOINT MANAGEMENT =============

@app.route('/api/waypoints', methods=['GET'])
def get_waypoints():
    """Get all active waypoints"""
    try:
        device_id = request.args.get('device_id', 'robot_01')
        
        waypoints = Waypoint.query.filter_by(device_id=device_id, completed=False)\
            .order_by(Waypoint.sequence).all()
        
        data = [{
            'id': wp.id,
            'sequence': wp.sequence,
            'latitude': wp.latitude,
            'longitude': wp.longitude,
            'description': wp.description,
            'completed': wp.completed
        } for wp in waypoints]
        
        return jsonify({'status': 'success', 'data': data}), 200
        
    except Exception as e:
        logger.error(f"Error retrieving waypoints: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/waypoints', methods=['POST'])
def add_waypoint():
    """
    Add new waypoint(s)
    Expected JSON format:
    {
        "waypoints": [
            {"latitude": 40.7128, "longitude": -74.0060, "description": "Point A"},
            {"latitude": 40.7129, "longitude": -74.0061, "description": "Point B"}
        ],
        "device_id": "robot_01"
    }
    """
    try:
        data = request.get_json()
        waypoints_data = data.get('waypoints', [])
        device_id = data.get('device_id', 'robot_01')
        
        # Get the current maximum sequence number
        max_seq = db.session.query(db.func.max(Waypoint.sequence))\
            .filter_by(device_id=device_id).scalar() or 0
        
        added_waypoints = []
        for idx, wp_data in enumerate(waypoints_data):
            waypoint = Waypoint(
                latitude=wp_data['latitude'],
                longitude=wp_data['longitude'],
                description=wp_data.get('description', f'Waypoint {max_seq + idx + 1}'),
                sequence=max_seq + idx + 1,
                device_id=device_id,
                completed=False
            )
            db.session.add(waypoint)
            added_waypoints.append(waypoint)
        
        db.session.commit()
        
        # Notify robot about new waypoints
        waypoint_list = [{
            'id': wp.id,
            'latitude': wp.latitude,
            'longitude': wp.longitude,
            'sequence': wp.sequence
        } for wp in added_waypoints]
        
        socketio.emit('waypoint_update', {
            'action': 'added',
            'waypoints': waypoint_list
        }, namespace='/realtime')
        
        return jsonify({
            'status': 'success',
            'message': f'{len(added_waypoints)} waypoint(s) added',
            'waypoints': waypoint_list
        }), 201
        
    except Exception as e:
        logger.error(f"Error adding waypoints: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/waypoints/<int:waypoint_id>/complete', methods=['POST'])
def complete_waypoint(waypoint_id):
    """Mark a waypoint as completed"""
    try:
        waypoint = Waypoint.query.get(waypoint_id)
        if not waypoint:
            return jsonify({'status': 'error', 'message': 'Waypoint not found'}), 404
        
        waypoint.completed = True
        waypoint.completed_at = datetime.now()
        db.session.commit()
        
        # Check if this is the last waypoint
        remaining_waypoints = Waypoint.query.filter_by(
            device_id=waypoint.device_id, 
            completed=False
        ).order_by(Waypoint.sequence).first()
        
        message = f"Waypoint #{waypoint.sequence} reached"
        if not remaining_waypoints:
            message += ". This is your destination - start measuring if needed."
        else:
            message += ". This is not your destination - proceeding to next waypoint."
        
        # Broadcast waypoint completion
        socketio.emit('waypoint_completed', {
            'waypoint_id': waypoint_id,
            'sequence': waypoint.sequence,
            'message': message,
            'is_final': remaining_waypoints is None
        }, namespace='/realtime')
        
        return jsonify({
            'status': 'success', 
            'message': message,
            'waypoint': {
                'id': waypoint.id,
                'sequence': waypoint.sequence,
                'completed': True
            }
        }), 200
        
    except Exception as e:
        logger.error(f"Error completing waypoint: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/waypoints/<int:waypoint_id>', methods=['DELETE'])
def delete_waypoint(waypoint_id):
    """Delete a single waypoint by ID"""
    try:
        waypoint = Waypoint.query.get(waypoint_id)
        if not waypoint:
            return jsonify({'status': 'error', 'message': 'Waypoint not found'}), 404

        device_id = waypoint.device_id
        db.session.delete(waypoint)
        db.session.commit()

        # Resequence remaining waypoints
        remaining = Waypoint.query.filter_by(device_id=device_id, completed=False)\
            .order_by(Waypoint.sequence).all()
        for idx, wp in enumerate(remaining, 1):
            wp.sequence = idx
        db.session.commit()

        socketio.emit('waypoint_update', {
            'action': 'deleted',
            'waypoint_id': waypoint_id
        }, namespace='/realtime')

        return jsonify({'status': 'success', 'message': f'Waypoint {waypoint_id} deleted'}), 200

    except Exception as e:
        logger.error(f"Error deleting waypoint: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/waypoints/clear', methods=['POST'])
def clear_waypoints():
    """Clear ALL waypoints for a device (including completed)"""
    try:
        device_id = request.get_json().get('device_id', 'robot_01')

        Waypoint.query.filter_by(device_id=device_id).delete()
        db.session.commit()

        socketio.emit('waypoint_update', {
            'action': 'cleared'
        }, namespace='/realtime')

        return jsonify({'status': 'success', 'message': 'All waypoints cleared'}), 200

    except Exception as e:
        logger.error(f"Error clearing waypoints: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


# Global sensor data control
sensor_data_enabled = True

@app.route('/api/sensor_data/control', methods=['POST'])
def control_sensor_data():
    """Control sensor data collection (pause/start/stop)"""
    global sensor_data_enabled
    
    try:
        data = request.get_json()
        action = data.get('action')  # 'pause', 'start', 'stop'
        
        if action not in ['pause', 'start', 'stop']:
            return jsonify({'status': 'error', 'message': 'Invalid action. Use: pause, start, stop'}), 400
        
        if action == 'pause':
            sensor_data_enabled = False
            message = 'Sensor data collection paused'
        elif action == 'start':
            sensor_data_enabled = True
            message = 'Sensor data collection started'
        elif action == 'stop':
            sensor_data_enabled = False
            message = 'Sensor data collection stopped'
        
        # Broadcast the control change
        socketio.emit('sensor_control', {
            'action': action,
            'enabled': sensor_data_enabled,
            'message': message
        }, namespace='/realtime')
        
        return jsonify({
            'status': 'success',
            'action': action,
            'enabled': sensor_data_enabled,
            'message': message
        }), 200
        
    except Exception as e:
        logger.error(f"Error controlling sensor data: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/sensor_data/control', methods=['GET'])
def get_sensor_control_status():
    """Get current sensor data control status"""
    return jsonify({
        'status': 'success',
        'enabled': sensor_data_enabled
    }), 200
    """
    Update robot status
    Expected JSON format:
    {
        "online": true,
        "battery": 85.5,
        "signal_strength": -75,
        "system_info": {"cpu": 45.2, "memory": 60.5},
        "device_id": "robot_01"
    }
    """
    try:
        data = request.get_json()
        device_id = data.get('device_id', 'robot_01')
        
        status = RobotStatus.query.filter_by(device_id=device_id).first()
        
        if not status:
            status = RobotStatus(device_id=device_id)
        
        status.online = data.get('online', True)
        status.battery_level = data.get('battery', 0)
        status.signal_strength = data.get('signal_strength', 0)
        status.last_update = datetime.now()
        status.system_info = json.dumps(data.get('system_info', {}))
        
        db.session.add(status)
        db.session.commit()
        
        # Update latest data
        with thread_lock:
            latest_data['status'] = {
                'online': status.online,
                'battery': status.battery_level,
                'signal_strength': status.signal_strength,
                'last_update': status.last_update.isoformat(),
                'fallback_active': backup_state.get('active', False)
            }
        
        # Broadcast status update
        socketio.emit('status_update', latest_data['status'], namespace='/realtime')
        
        return jsonify({'status': 'success', 'message': 'Status updated'}), 200
        
    except Exception as e:
        logger.error(f"Error updating status: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/status', methods=['GET', 'POST'])
def get_status():
    """Get or update current robot status"""
    try:
        if request.method == 'POST':
            # Handle status update
            data = request.get_json()
            if not data:
                return jsonify({'status': 'error', 'message': 'No data provided'}), 400
            
            device_id = data.get('device_id', 'robot_01')
            
            # Update or create status record
            status = RobotStatus.query.filter_by(device_id=device_id).first()
            if not status:
                status = RobotStatus(device_id=device_id)
                db.session.add(status)
            
            # Update status fields
            status.online = data.get('online', True)
            status.battery_level = data.get('battery', 0)
            status.signal_strength = data.get('signal_strength', 0)
            status.system_info = json.dumps(data.get('system_info', {}))
            status.last_update = datetime.utcnow()
            
            db.session.commit()
            
            # Emit WebSocket update
            socketio.emit('status_update', {
                'device_id': device_id,
                'online': status.online,
                'battery': status.battery_level,
                'signal_strength': status.signal_strength,
                'last_update': status.last_update.isoformat(),
                'system_info': json.loads(status.system_info) if status.system_info else {}
            }, namespace='/realtime')
            
            return jsonify({'status': 'success'}), 200
        
        else:
            # Handle GET request (existing logic)
            device_id = request.args.get('device_id', 'robot_01')
            status = RobotStatus.query.filter_by(device_id=device_id).first()
            
            if not status:
                return jsonify({
                    'status': 'success',
                    'data': {
                        'online': False,
                        'battery': 0,
                        'last_update': None
                    }
                }), 200
            
            data = {
                'online': status.online,
                'battery': status.battery_level,
                'signal_strength': status.signal_strength,
                'last_update': status.last_update.isoformat() if status.last_update else None,
                'system_info': json.loads(status.system_info) if status.system_info else {},
                'fallback_active': backup_state.get('active', False)
            }
            
            return jsonify({'status': 'success', 'data': data}), 200
        
    except Exception as e:
        logger.error(f"Error handling status: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


# ============= ROBOT COMMAND QUEUE =============

@app.route('/api/commands', methods=['POST'])
def create_command():
    """Create a new robot control command"""
    try:
        data = request.get_json() or {}
        command_type = data.get('command')
        if not command_type:
            return jsonify({'status': 'error', 'message': 'Missing command type'}), 400

        device_id = data.get('device_id') or app.config.get('DEFAULT_DEVICE_ID', 'robot_01')
        payload = data.get('payload', {})

        command = RobotCommand(
            device_id=device_id,
            command_type=command_type,
            payload=json.dumps(payload) if isinstance(payload, (dict, list)) else payload
        )
        db.session.add(command)
        db.session.commit()

        command_data = command.to_dict()
        socketio.emit('command_update', {
            'event': 'created',
            'command': command_data
        }, namespace='/realtime')

        return jsonify({'status': 'success', 'command': command_data}), 201

    except Exception as e:
        logger.error(f"Error creating command: {e}")
        db.session.rollback()
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/commands/pending', methods=['GET'])
def get_pending_commands():
    """Return pending commands for a device"""
    device_id = request.args.get('device_id', app.config.get('DEFAULT_DEVICE_ID', 'robot_01'))
    limit = request.args.get('limit', 20, type=int)

    commands = RobotCommand.query \
        .filter_by(device_id=device_id, status='pending') \
        .order_by(RobotCommand.created_at.asc()) \
        .limit(limit).all()

    if not commands:
        return jsonify({'status': 'success', 'commands': []}), 200

    now = datetime.utcnow()
    for command in commands:
        command.status = 'sent'
        command.updated_at = now

    db.session.commit()

    return jsonify({
        'status': 'success',
        'commands': [cmd.to_dict() for cmd in commands]
    }), 200


@app.route('/api/commands/<int:command_id>/ack', methods=['POST'])
def acknowledge_command(command_id: int):
    """Mark command as completed/failed by device"""
    command = RobotCommand.query.get_or_404(command_id)
    data = request.get_json() or {}
    status = data.get('status', 'completed')

    if status not in {'completed', 'failed'}:
        return jsonify({'status': 'error', 'message': 'Invalid status'}), 400

    command.status = status
    command.error_message = data.get('error_message')
    command.processed_at = datetime.utcnow()
    command.updated_at = datetime.utcnow()
    db.session.commit()

    socketio.emit('command_update', {
        'event': 'acknowledged',
        'command': command.to_dict()
    }, namespace='/realtime')

    return jsonify({'status': 'success'}), 200


# ============= FAST MANUAL COMMAND CHANNEL =============
# These endpoints bypass the DB queue for latency-critical commands
# like manual drive arrows.  The browser pushes commands via WebSocket
# or HTTP POST; the Pi polls at ~100ms via GET.

@app.route('/api/commands/instant', methods=['POST'])
def set_instant_command():
    """Enqueue an instant command for the Pi to pick up immediately."""
    global _instant_command_seq
    try:
        data = request.get_json() or {}
        command_type = data.get('command')
        if not command_type:
            return jsonify({'status': 'error', 'message': 'Missing command'}), 400

        with thread_lock:
            _instant_command_seq += 1
            seq = _instant_command_seq
            _instant_queue.append({
                'command_type': command_type,
                'payload': data.get('payload', {}),
                'device_id': data.get('device_id') or app.config.get('DEFAULT_DEVICE_ID', 'robot_01'),
                'seq': seq,
                'timestamp': datetime.utcnow().isoformat()
            })

        logger.info("Instant command queued (POST): %s seq=%d", command_type, seq)

        # Track base navigation state for AI auto-drive gating
        if command_type == 'NAV_START' or command_type == 'NAV_RESUME':
            ai_vision.set_base_nav('waypoint')
        elif command_type in ('NAV_STOP', 'NAV_PAUSE'):
            ai_vision.set_base_nav('none')
        elif command_type == 'MANUAL_DRIVE':
            direction = data.get('payload', {}).get('direction', '')
            if direction in ('forward', 'reverse'):
                ai_vision.set_base_nav('manual')
            elif direction in ('stop', 'brake', ''):
                if ai_vision.base_nav_mode == 'manual':
                    ai_vision.set_base_nav('none')

        return jsonify({'status': 'success', 'seq': seq}), 200
    except Exception as e:
        logger.error("Error in instant command: %s", e)
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/commands/instant', methods=['GET'])
def get_instant_command():
    """Pi polls this at high frequency.
    Returns ALL queued commands (drains the queue) so nothing is lost."""
    commands = []
    with thread_lock:
        while _instant_queue:
            commands.append(_instant_queue.popleft())
    if commands:
        return jsonify({'status': 'success', 'commands': commands}), 200
    return jsonify({'status': 'success', 'commands': []}), 200


# ============= CAMERA STREAM =============

@app.route('/api/camera/frame', methods=['POST'])
def receive_camera_frame():
    """
    Receive camera frame from Raspberry Pi (legacy JSON base64)
    Expected JSON format:
    {
        "frame": "base64_encoded_image",
        "timestamp": "2025-11-04T12:00:00",
        "device_id": "robot_01"
    }
    """
    try:
        if app.config.get('CAMERA_STREAM_MODE') == 'direct' and not app.config.get('CAMERA_ALLOW_FALLBACK', True):
            return jsonify({'status': 'ignored', 'message': 'Direct camera stream enabled'}), 202

        data = request.get_json() or {}
        frame_b64 = data.get('frame', '')
        timestamp = data.get('timestamp')
        device_id = data.get('device_id')

        if not frame_b64 or not isinstance(frame_b64, str):
            return jsonify({'status': 'error', 'message': 'Missing or invalid frame'}), 400

        # Basic validation on size
        max_bytes = app.config.get('CAMERA_MAX_FRAME_SIZE', 2 * 1024 * 1024)
        max_b64_len = int(max_bytes * 4 / 3) + 32
        if len(frame_b64) > max_b64_len:
            logger.warning("Rejected oversized base64 camera frame (len=%d, max=%d)", len(frame_b64), max_b64_len)
            return jsonify({'status': 'error', 'message': 'Frame too large'}), 413

        try:
            frame_bytes = base64.b64decode(frame_b64)
        except Exception:
            return jsonify({'status': 'error', 'message': 'Invalid base64 frame'}), 400

        _store_camera_frame_bytes(frame_bytes, timestamp=timestamp, device_id=device_id)

        # Broadcast frame to connected clients (we keep emitting base64 for backward compatibility)
        socketio.emit('camera_frame', {
            'frame': frame_b64,
            'timestamp': timestamp
        }, namespace='/realtime')

        return jsonify({'status': 'success'}), 200

    except Exception as e:
        logger.exception(f"Error receiving camera frame: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/camera/frame_binary', methods=['POST'])
def receive_camera_frame_binary():
    """
    Receive binary JPEG frame from Raspberry Pi
    - Content-Type: image/jpeg
    - optional query params: device_id, timestamp
    """
    try:
        if app.config.get('CAMERA_STREAM_MODE') == 'direct' and not app.config.get('CAMERA_ALLOW_FALLBACK', True):
            return jsonify({'status': 'ignored', 'message': 'Direct camera stream enabled'}), 202

        # Read raw bytes
        frame_bytes = request.get_data()
        if not frame_bytes:
            return jsonify({'status': 'error', 'message': 'No frame bytes received'}), 400

        max_bytes = app.config.get('CAMERA_MAX_FRAME_SIZE', 2 * 1024 * 1024)
        if len(frame_bytes) > max_bytes:
            logger.warning("Rejected oversized binary camera frame (len=%d, max=%d)", len(frame_bytes), max_bytes)
            return jsonify({'status': 'error', 'message': 'Frame too large'}), 413

        device_id = request.args.get('device_id') or request.form.get('device_id')
        timestamp = request.args.get('timestamp') or request.form.get('timestamp') or datetime.utcnow().isoformat()

        data = _store_camera_frame_bytes(frame_bytes, timestamp=timestamp, device_id=device_id)

        # Emit minimal socket event (we keep legacy frame emission optional)
        try:
            socketio.emit('camera_frame', {
                'frame': data['frame_b64'],
                'timestamp': data['timestamp']
            }, namespace='/realtime')
        except Exception:
            # If socket emission fails, we still accepted the frame
            logger.debug("Socket emit for camera frame failed")

        return jsonify({'status': 'success'}), 200

    except Exception as e:
        logger.exception("Error receiving binary camera frame: %s", e)
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/event', methods=['POST'])
def receive_robot_event():
    """Receive robot events (e.g., obstacle detected) and broadcast via WebSocket."""
    try:
        data = request.get_json(silent=True) or {}
        event = {
            'type': data.get('type') or 'UNKNOWN',
            'device_id': data.get('device_id') or 'robot_01',
            'timestamp': data.get('timestamp') or datetime.utcnow().isoformat(),
            'payload': {k: v for k, v in data.items() if k not in ('type','device_id','timestamp')}
        }
        with thread_lock:
            # Cache latest nav status for full_update snapshots
            if event['type'] == 'NAV_STATUS':
                latest_data['nav_status'] = event['payload']
                # If nav reached IDLE or COMPLETE, clear waypoint base nav
                nav_state = event['payload'].get('nav', {}).get('state', '')
                if nav_state in ('IDLE', 'COMPLETE'):
                    if ai_vision.base_nav_mode == 'waypoint':
                        ai_vision.set_base_nav('none')
            else:
                # Only store non-NAV_STATUS events in the ring buffer
                # to avoid drowning out real events at 2 Hz
                recent_events.append(event)
                if len(recent_events) > MAX_EVENTS:
                    del recent_events[0]
        try:
            socketio.emit('robot_event', event, namespace='/realtime')
        except Exception:
            logger.debug("Failed to emit robot_event")
        return jsonify({'status': 'success', 'event': event}), 200
    except Exception as exc:
        logger.exception("Error receiving robot event: %s", exc)
        return jsonify({'status': 'error', 'message': str(exc)}), 500


@app.route('/api/camera/latest')
def get_latest_frame():
    """Get the most recent camera frame (legacy JSON returning base64)"""
    with thread_lock:
        frame_data = latest_data.get('camera_frame')
        if frame_data:
            return jsonify({'status': 'success', 'data': {
                'frame': frame_data.get('frame_b64'),
                'timestamp': frame_data.get('timestamp'),
                'device_id': frame_data.get('device_id')
            }}), 200
        else:
            return jsonify({'status': 'error', 'message': 'No frame available'}), 404


@app.route('/camera/mjpeg')
def mjpeg_stream():
    """
    Serve an MJPEG multipart stream of the latest frames.
    Clients can point an <img> at /camera/mjpeg and receive live frames.
    """
    if not app.config.get('CAMERA_MJPEG_ENABLED', True):
        return jsonify({'status': 'error', 'message': 'MJPEG disabled'}), 404

    boundary = "--frame"

    def generate():
        last_ts = None
        while True:
            with thread_lock:
                frame_entry = latest_data.get('camera_frame')
            if frame_entry and frame_entry.get('frame_bytes'):
                ts = frame_entry.get('timestamp')
                if ts != last_ts:
                    last_ts = ts
                    frame_bytes = frame_entry['frame_bytes']
                    try:
                        yield (b"%b\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" %
                               (boundary.encode('utf-8'), len(frame_bytes)))
                        yield frame_bytes
                        yield b"\r\n"
                    except GeneratorExit:
                        break
                    except Exception as e:
                        logger.debug("Error yielding MJPEG frame: %s", e)
            # small sleep to avoid busy-looping; respects server performance
            time.sleep(0.03)

    resp = Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')
    # Reduce buffering/caching so the feed stays as close to real-time as possible.
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    resp.headers['Expires'] = '0'
    # If running behind a proxy like nginx, this helps disable response buffering.
    resp.headers['X-Accel-Buffering'] = 'no'
    return resp


# ============= AI VISION API =============

@app.route('/api/ai/status', methods=['GET'])
def ai_status():
    """Current AI Vision engine status."""
    return jsonify(ai_vision.get_status())


@app.route('/api/ai/enable', methods=['POST'])
def ai_enable():
    """Enable / disable continuous AI analysis."""
    data = request.get_json(silent=True) or {}
    enabled = bool(data.get('enabled', False))
    ai_vision.set_enabled(enabled)
    return jsonify({'status': 'ok', 'enabled': ai_vision.enabled})


@app.route('/api/ai/mode', methods=['POST'])
def ai_set_mode():
    """Change analysis mode: scene | obstacles | direction | terrain | detect | custom."""
    data = request.get_json(silent=True) or {}
    mode = data.get('mode', 'scene')
    ai_vision.set_mode(
        mode,
        detect_target=data.get('detect_target'),
        custom_prompt=data.get('custom_prompt'),
    )
    return jsonify({'status': 'ok', 'mode': mode})


@app.route('/api/ai/interval', methods=['POST'])
def ai_set_interval():
    """Set seconds between analyses (1–30)."""
    data = request.get_json(silent=True) or {}
    seconds = float(data.get('interval', 3.0))
    ai_vision.set_interval(seconds)
    return jsonify({'status': 'ok', 'interval': ai_vision._interval})


@app.route('/api/ai/query', methods=['POST'])
def ai_query():
    """One-shot manual query with optional custom prompt."""
    if not ai_vision.is_ready:
        return jsonify({'status': 'error', 'message': 'Model not loaded'}), 503
    data = request.get_json(silent=True) or {}
    prompt = data.get('prompt', 'Describe what you see.')
    frame = _get_latest_frame_bytes()
    if not frame:
        return jsonify({'status': 'error', 'message': 'No camera frame available'}), 404
    result = ai_vision.query_once(frame, prompt)
    if result:
        socketio.emit('ai_vision_update', result, namespace='/realtime')
        return jsonify(result)
    return jsonify({'status': 'error', 'message': 'Inference failed'}), 500


@app.route('/api/ai/detect', methods=['POST'])
def ai_detect():
    """One-shot object detection."""
    if not ai_vision.is_ready:
        return jsonify({'status': 'error', 'message': 'Model not loaded'}), 503
    data = request.get_json(silent=True) or {}
    target = data.get('target', 'obstacle')
    frame = _get_latest_frame_bytes()
    if not frame:
        return jsonify({'status': 'error', 'message': 'No camera frame available'}), 404
    result = ai_vision.detect_once(frame, target)
    if result:
        socketio.emit('ai_vision_update', result, namespace='/realtime')
        return jsonify(result)
    return jsonify({'status': 'error', 'message': 'Detection failed'}), 500


@app.route('/api/ai/load', methods=['POST'])
def ai_load_model():
    """Trigger model download & load (runs in background)."""
    ai_vision.load_model()
    return jsonify({'status': 'ok', 'model_status': ai_vision.status})


@app.route('/api/ai/unload', methods=['POST'])
def ai_unload_model():
    """Unload model and free memory."""
    ai_vision.unload_model()
    return jsonify({'status': 'ok', 'model_status': ai_vision.status})


@app.route('/api/ai/drive', methods=['POST'])
def ai_drive_toggle():
    """Toggle auto-drive: in navigate mode, AI sends drive commands to the Pi.
    RULE: Only works when a base navigation method is already active."""
    data = request.get_json(silent=True) or {}
    enabled = bool(data.get('enabled', False))
    ai_vision.set_auto_drive(enabled)
    return jsonify({
        'status': 'ok',
        'auto_drive': ai_vision.auto_drive,
        'base_nav_mode': ai_vision.base_nav_mode,
        'mode': ai_vision._mode,
    })


@app.route('/api/ai/base_nav', methods=['POST'])
def ai_set_base_nav():
    """Explicitly set the base navigation mode (for JS-side tracking)."""
    data = request.get_json(silent=True) or {}
    mode = data.get('mode', 'none')
    ai_vision.set_base_nav(mode)
    return jsonify({
        'status': 'ok',
        'base_nav_mode': ai_vision.base_nav_mode,
        'auto_drive': ai_vision.auto_drive,
    })


# ============= WEBSOCKET EVENTS =============

@socketio.on('connect', namespace='/realtime')
def handle_connect():
    """Handle client connection"""
    logger.info('Client connected to WebSocket')
    emit('connection_response', {'status': 'connected'})


@socketio.on('disconnect', namespace='/realtime')
def handle_disconnect():
    """Handle client disconnection"""
    logger.info('Client disconnected from WebSocket')


@socketio.on('request_update', namespace='/realtime')
def handle_update_request(data):
    """Handle request for latest data"""
    with thread_lock:
        camera_entry = latest_data.get('camera_frame') or {}
        emit('full_update', {
            'sensors': latest_data['sensors'],
            'gps': latest_data['gps'],
            'status': latest_data['status'],
            'camera': {
                'timestamp': camera_entry.get('timestamp'),
                # We do not include large frame payloads here to keep event light
            },
            'backup': latest_data.get('backup'),
            'nav_status': latest_data.get('nav_status'),
            'ai_vision': ai_vision.get_status()
        })


@socketio.on('instant_command', namespace='/realtime')
def handle_instant_command(data):
    """Handle ALL dashboard→Pi commands via WebSocket.
    Commands are appended to a queue so nothing is lost between polls."""
    global _instant_command_seq
    if not data or not data.get('command'):
        return
    with thread_lock:
        _instant_command_seq += 1
        seq = _instant_command_seq
        _instant_queue.append({
            'command_type': data['command'],
            'payload': data.get('payload', {}),
            'device_id': data.get('device_id') or app.config.get('DEFAULT_DEVICE_ID', 'robot_01'),
            'seq': seq,
            'timestamp': datetime.utcnow().isoformat()
        })
    logger.info("Instant command queued (WS): %s seq=%d  (queue depth: %d)",
                data['command'], seq, len(_instant_queue))

    # Track base navigation state for AI auto-drive gating.
    # Auto-drive only works when a base navigation method is active.
    cmd = data['command']
    if cmd == 'NAV_START' or cmd == 'NAV_RESUME':
        ai_vision.set_base_nav('waypoint')
    elif cmd in ('NAV_STOP', 'NAV_PAUSE'):
        ai_vision.set_base_nav('none')
    elif cmd == 'MANUAL_DRIVE':
        direction = (data.get('payload') or {}).get('direction', '')
        if direction in ('forward', 'reverse'):
            ai_vision.set_base_nav('manual')
        elif direction in ('stop', 'brake', ''):
            # Only clear if currently in manual mode (don't clear waypoint nav)
            if ai_vision.base_nav_mode == 'manual':
                ai_vision.set_base_nav('none')


# ============= DATABASE INITIALIZATION =============

def initialize_app():
    """Initialize database and ZigBee bridge"""
    with app.app_context():
        db.create_all()
        logger.info("Database tables created")
        initialize_zigbee()


# ============= MAIN =============

if __name__ == '__main__':
    # Initialize app components
    initialize_app()
    
    logger.info("Starting Environmental Monitoring Robot Dashboard...")
    logger.info(f"Dashboard URL: http://0.0.0.0:{app.config['PORT']}")
    
    socketio.run(
        app,
        host='0.0.0.0',
        port=app.config['PORT'],
        debug=app.config['DEBUG']
    )