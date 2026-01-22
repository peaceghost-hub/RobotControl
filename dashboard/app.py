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

from datetime import datetime
import json
import logging
from threading import Lock
import base64
from typing import Optional
import time

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
    }
}

backup_state = {
    'active': False,
    'device_id': None,
    'last_update': None,
    'last_location': None,
    'reason': None,
    'last_command': None
}

zigbee_bridge = None

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
    try:
        data = request.get_json()
        logger.info(f"Received sensor data: {data}")
        
        # Store in database
        sensor_reading = SensorReading(
            timestamp=datetime.fromisoformat(data.get('timestamp', datetime.now().isoformat())),
            temperature=data.get('temperature'),
            humidity=data.get('humidity'),
            mq2=data.get('mq2'),
            mq135=data.get('mq135'),
            mq7=data.get('mq7'),
            device_id=data.get('device_id', 'robot_01')
        )
        db.session.add(sensor_reading)
        db.session.commit()
        
        # Update latest data
        with thread_lock:
            latest_data['sensors'] = data
        
        # Broadcast to connected clients via WebSocket
        socketio.emit('sensor_update', data, namespace='/realtime')
        
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


@app.route('/api/waypoints/<int:waypoint_id>', methods=['DELETE'])
def delete_waypoint(waypoint_id):
    """Delete a specific waypoint"""
    try:
        waypoint = Waypoint.query.get(waypoint_id)
        if not waypoint:
            return jsonify({'status': 'error', 'message': 'Waypoint not found'}), 404
        
        db.session.delete(waypoint)
        db.session.commit()
        
        socketio.emit('waypoint_update', {
            'action': 'deleted',
            'waypoint_id': waypoint_id
        }, namespace='/realtime')
        
        return jsonify({'status': 'success', 'message': 'Waypoint deleted'}), 200
        
    except Exception as e:
        logger.error(f"Error deleting waypoint: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/waypoints/clear', methods=['POST'])
def clear_waypoints():
    """Clear all waypoints for a device"""
    try:
        device_id = request.get_json().get('device_id', 'robot_01')
        
        Waypoint.query.filter_by(device_id=device_id, completed=False).delete()
        db.session.commit()
        
        socketio.emit('waypoint_update', {
            'action': 'cleared'
        }, namespace='/realtime')
        
        return jsonify({'status': 'success', 'message': 'All waypoints cleared'}), 200
        
    except Exception as e:
        logger.error(f"Error clearing waypoints: {str(e)}")
        return jsonify({'status': 'error', 'message': str(e)}), 500


# ============= ROBOT STATUS =============

@app.route('/api/status', methods=['POST'])
def update_status():
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


@app.route('/api/status', methods=['GET'])
def get_status():
    """Get current robot status"""
    try:
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
        logger.error(f"Error retrieving status: {str(e)}")
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

    return Response(stream_with_context(generate()), mimetype='multipart/x-mixed-replace; boundary=frame')


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
            'backup': latest_data.get('backup')
        })


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