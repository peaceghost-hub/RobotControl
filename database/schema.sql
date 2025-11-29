-- Database Schema for Environmental Monitoring Robot
-- SQLite/MySQL compatible

-- Sensor Readings Table
CREATE TABLE IF NOT EXISTS sensor_readings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    device_id VARCHAR(50) NOT NULL DEFAULT 'robot_01',
    temperature REAL,
    humidity REAL,
    mq2 INTEGER,
    mq135 INTEGER,
    mq7 INTEGER,
    mq3 INTEGER,
    mq4 INTEGER,
    mq8 INTEGER,
    mq9 INTEGER,
    INDEX idx_timestamp (timestamp),
    INDEX idx_device (device_id)
);

-- GPS Locations Table
CREATE TABLE IF NOT EXISTS gps_locations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    device_id VARCHAR(50) NOT NULL DEFAULT 'robot_01',
    latitude REAL NOT NULL,
    longitude REAL NOT NULL,
    altitude REAL DEFAULT 0,
    speed REAL DEFAULT 0,
    heading REAL DEFAULT 0,
    satellites INTEGER DEFAULT 0,
    INDEX idx_timestamp (timestamp),
    INDEX idx_device (device_id)
);

-- Waypoints Table
CREATE TABLE IF NOT EXISTS waypoints (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id VARCHAR(50) NOT NULL DEFAULT 'robot_01',
    latitude REAL NOT NULL,
    longitude REAL NOT NULL,
    sequence INTEGER NOT NULL,
    description VARCHAR(200),
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    completed BOOLEAN DEFAULT FALSE,
    completed_at DATETIME,
    INDEX idx_device (device_id),
    INDEX idx_completed (completed)
);

-- Robot Status Table
CREATE TABLE IF NOT EXISTS robot_status (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id VARCHAR(50) NOT NULL UNIQUE DEFAULT 'robot_01',
    online BOOLEAN DEFAULT FALSE,
    battery_level REAL DEFAULT 0,
    signal_strength INTEGER DEFAULT 0,
    last_update DATETIME DEFAULT CURRENT_TIMESTAMP,
    system_info TEXT
);

-- Event Logs Table
CREATE TABLE IF NOT EXISTS event_logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    device_id VARCHAR(50) NOT NULL DEFAULT 'robot_01',
    event_type VARCHAR(50) NOT NULL,
    event_category VARCHAR(50),
    message TEXT NOT NULL,
    details TEXT,
    INDEX idx_timestamp (timestamp),
    INDEX idx_device (device_id),
    INDEX idx_type (event_type)
);

-- Initial robot status entry
INSERT INTO robot_status (device_id, online, battery_level, signal_strength)
VALUES ('robot_01', FALSE, 0, 0)
ON CONFLICT(device_id) DO NOTHING;
