"""
Database Models for Environmental Monitoring Robot Dashboard
"""

try:
    from flask_sqlalchemy import SQLAlchemy
    SQLALCHEMY_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Flask-SQLAlchemy not available. Install with: pip install -r requirements.txt")
    print(f"Error: {e}")
    SQLALCHEMY_AVAILABLE = False
    # Define dummy SQLAlchemy classes
    class Column:
        def __init__(self, *args, **kwargs): pass
    class Integer:
        pass
    class Float:
        pass
    class String:
        def __init__(self, *args, **kwargs): pass
    class DateTime:
        pass
    class Text:
        pass
    class Boolean:
        pass
    class Model:
        Column = Column
        Integer = Integer
        Float = Float
        String = String
        DateTime = DateTime
        Text = Text
        Boolean = Boolean
    class SQLAlchemy:
        def __init__(self, *args, **kwargs): pass
        def init_app(self, *args, **kwargs): pass
        Model = Model
        Column = Column
        Integer = Integer
        Float = Float
        String = String
        DateTime = DateTime
        Text = Text
        Boolean = Boolean

from datetime import datetime
import json

db = SQLAlchemy()


class SensorReading(db.Model):
    """Store environmental sensor readings"""
    __tablename__ = 'sensor_readings'
    
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow, index=True)
    device_id = db.Column(db.String(50), nullable=False, default='robot_01', index=True)
    
    # Temperature and Humidity (DHT11/DHT22)
    temperature = db.Column(db.Float)  # Celsius
    humidity = db.Column(db.Float)  # Percentage
    
    # Gas Sensors (MQ Series)
    mq2 = db.Column(db.Integer)  # Smoke, LPG, CO
    mq135 = db.Column(db.Integer)  # Air quality (CO2, NH3, NOx)
    mq7 = db.Column(db.Integer)  # Carbon Monoxide
    mq3 = db.Column(db.Integer)  # Alcohol, Benzene
    mq4 = db.Column(db.Integer)  # Methane, CNG
    mq8 = db.Column(db.Integer)  # Hydrogen
    mq9 = db.Column(db.Integer)  # CO, Flammable gases
    
    def __repr__(self):
        return f'<SensorReading {self.device_id} at {self.timestamp}>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'timestamp': self.timestamp.isoformat(),
            'device_id': self.device_id,
            'temperature': self.temperature,
            'humidity': self.humidity,
            'mq2': self.mq2,
            'mq135': self.mq135,
            'mq7': self.mq7,
            'mq3': self.mq3,
            'mq4': self.mq4,
            'mq8': self.mq8,
            'mq9': self.mq9
        }


class GPSLocation(db.Model):
    """Store GPS location history"""
    __tablename__ = 'gps_locations'
    
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow, index=True)
    device_id = db.Column(db.String(50), nullable=False, default='robot_01', index=True)
    
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    altitude = db.Column(db.Float, default=0)  # Meters
    speed = db.Column(db.Float, default=0)  # m/s
    heading = db.Column(db.Float, default=0)  # Degrees (0-360)
    satellites = db.Column(db.Integer, default=0)
    source = db.Column(db.String(20), default='primary')  # primary, backup, manual
    
    def __repr__(self):
        return f'<GPSLocation {self.device_id} ({self.latitude}, {self.longitude})>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'timestamp': self.timestamp.isoformat(),
            'device_id': self.device_id,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'speed': self.speed,
            'heading': self.heading,
            'satellites': self.satellites,
            'source': self.source
        }


class Waypoint(db.Model):
    """Store navigation waypoints"""
    __tablename__ = 'waypoints'
    
    id = db.Column(db.Integer, primary_key=True)
    device_id = db.Column(db.String(50), nullable=False, default='robot_01', index=True)
    
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    sequence = db.Column(db.Integer, nullable=False)  # Order of waypoints
    description = db.Column(db.String(200))
    
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    completed = db.Column(db.Boolean, default=False)
    completed_at = db.Column(db.DateTime)
    
    def __repr__(self):
        return f'<Waypoint {self.id} ({self.latitude}, {self.longitude}) seq={self.sequence}>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'device_id': self.device_id,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'sequence': self.sequence,
            'description': self.description,
            'created_at': self.created_at.isoformat(),
            'completed': self.completed,
            'completed_at': self.completed_at.isoformat() if self.completed_at else None
        }


class RobotStatus(db.Model):
    """Store robot system status"""
    __tablename__ = 'robot_status'
    
    id = db.Column(db.Integer, primary_key=True)
    device_id = db.Column(db.String(50), nullable=False, unique=True, default='robot_01')
    
    online = db.Column(db.Boolean, default=False)
    battery_level = db.Column(db.Float, default=0)  # Percentage
    signal_strength = db.Column(db.Integer, default=0)  # dBm
    
    last_update = db.Column(db.DateTime, default=datetime.utcnow)
    system_info = db.Column(db.Text)  # JSON string with additional info
    
    def __repr__(self):
        return f'<RobotStatus {self.device_id} online={self.online}>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'device_id': self.device_id,
            'online': self.online,
            'battery_level': self.battery_level,
            'signal_strength': self.signal_strength,
            'last_update': self.last_update.isoformat(),
            'system_info': self.system_info
        }


class EventLog(db.Model):
    """Store system events and alerts"""
    __tablename__ = 'event_logs'
    
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow, index=True)
    device_id = db.Column(db.String(50), nullable=False, default='robot_01', index=True)
    
    event_type = db.Column(db.String(50), nullable=False)  # 'info', 'warning', 'error', 'alert'
    event_category = db.Column(db.String(50))  # 'navigation', 'sensor', 'system', 'communication'
    message = db.Column(db.Text, nullable=False)
    details = db.Column(db.Text)  # JSON string with additional details
    
    def __repr__(self):
        return f'<EventLog {self.event_type}: {self.message}>'
    
    def to_dict(self):
        return {
            'id': self.id,
            'timestamp': self.timestamp.isoformat(),
            'device_id': self.device_id,
            'event_type': self.event_type,
            'event_category': self.event_category,
            'message': self.message,
            'details': self.details
        }


class RobotCommand(db.Model):
    """Queue of outbound robot control commands"""
    __tablename__ = 'robot_commands'

    id = db.Column(db.Integer, primary_key=True)
    device_id = db.Column(db.String(50), nullable=False, index=True, default='robot_01')
    command_type = db.Column(db.String(50), nullable=False)
    payload = db.Column(db.Text)  # JSON encoded payload
    status = db.Column(db.String(20), nullable=False, default='pending')  # pending, sent, completed, failed
    error_message = db.Column(db.Text)

    created_at = db.Column(db.DateTime, default=datetime.utcnow, nullable=False)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    processed_at = db.Column(db.DateTime)

    def __repr__(self):
        return f'<RobotCommand {self.device_id} {self.command_type} status={self.status}>'

    def to_dict(self):
        try:
            payload_data = json.loads(self.payload) if self.payload else {}
        except json.JSONDecodeError:
            payload_data = self.payload or {}
        return {
            'id': self.id,
            'device_id': self.device_id,
            'command_type': self.command_type,
            'payload': payload_data,
            'status': self.status,
            'error_message': self.error_message,
            'created_at': self.created_at.isoformat(),
            'updated_at': self.updated_at.isoformat() if self.updated_at else None,
            'processed_at': self.processed_at.isoformat() if self.processed_at else None
        }
