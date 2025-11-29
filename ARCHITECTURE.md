# System Architecture
## ARM-Based Environmental Monitoring and Aide Robot

---

## Overview

This document describes the detailed system architecture of the environmental monitoring robot project, including hardware components, software modules, communication protocols, and data flow.

---

## System Components

### 1. Web Dashboard (Control Center)
**Technology:** Flask (Python), HTML/CSS/JavaScript, WebSocket, SQLite  
**Location:** Local PC or Cloud Server  
**Purpose:** Real-time monitoring and control interface

**Key Features:**
- Real-time sensor data visualization
- Live camera feed display
- GPS tracking and map interface
- Waypoint management
- Robot status monitoring
- Historical data analysis

### 2. Raspberry Pi 3 Model B (Main Controller)
**Operating System:** Raspbian OS  
**Language:** Python 3.7+  
**Purpose:** Environmental sensing and communication hub

**Responsibilities:**
- Environmental sensor data acquisition
- GSM/Internet connectivity
- Camera capture and streaming
- Data processing and transmission
- Arduino coordination
- API communication with dashboard

### 3. Arduino Mega 2560 (Navigation Controller)
**Language:** C/C++ (Arduino)  
**Purpose:** Autonomous navigation system

**Responsibilities:**
- GPS waypoint navigation
- Compass-based heading control
- Obstacle detection and avoidance
- Motor control
- Serial communication with Raspberry Pi

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    WEB DASHBOARD                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐│
│  │ Sensors  │  │  Camera  │  │   Map    │  │ Control ││
│  │ Display  │  │   Feed   │  │ & GPS    │  │  Panel  ││
│  └──────────┘  └──────────┘  └──────────┘  └─────────┘│
└───────────────────────┬─────────────────────────────────┘
                        │ HTTP/WebSocket
                        │ (Internet via GSM)
                        ↓
┌─────────────────────────────────────────────────────────┐
│              RASPBERRY PI 3 MODEL B                      │
│  ┌──────────────────────────────────────────────────┐  │
│  │ Main Control Loop (main.py)                      │  │
│  │  ↕️                    ↕️                    ↕️     │  │
│  │ Sensors      Communication      Camera         │  │
│  │ Manager         Modules         Stream         │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐       │
│  │  DHT22     │  │  MQ-2      │  │  MQ-135    │       │
│  │  Temp/Hum  │  │  Smoke     │  │  CO2       │       │
│  └────────────┘  └────────────┘  └────────────┘       │
│                                                          │
│  ┌────────────┐  ┌────────────┐                        │
│  │  MQ-7      │  │ Pi Camera  │                        │
│  │  CO        │  │  Module    │                        │
│  └────────────┘  └────────────┘                        │
│                                                          │
│  ┌────────────────────────────────┐                    │
│  │     GSM Module (SIM800L)       │                    │
│  │     Internet Connection        │                    │
│  └────────────────────────────────┘                    │
└───────────────────┬─────────────────────────────────────┘
                    │ Serial UART
                    │ (JSON Protocol)
                    ↓
┌─────────────────────────────────────────────────────────┐
│              ARDUINO MEGA 2560                           │
│  ┌──────────────────────────────────────────────────┐  │
│  │ Navigation System (robot_navigation.ino)         │  │
│  │  ↕️                    ↕️                    ↕️     │  │
│  │ GPS Handler   Compass Handler   Motor Control   │  │
│  └──────────────────────────────────────────────────┘  │
│                                                          │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐       │
│  │  NEO-6M    │  │ HMC5883L   │  │  HC-SR04   │       │
│  │  GPS       │  │ Compass    │  │ Ultrasonic │       │
│  └────────────┘  └────────────┘  └────────────┘       │
│                                                          │
│  ┌────────────────────────────────┐                    │
│  │      L298N Motor Driver        │                    │
│  │      ├─ Left DC Motor          │                    │
│  │      └─ Right DC Motor         │                    │
│  └────────────────────────────────┘                    │
└─────────────────────────────────────────────────────────┘
```

---

## Communication Protocols

### 1. Dashboard ↔ Raspberry Pi

**Protocol:** HTTP REST API + WebSocket  
**Transport:** Internet (via GSM module)  
**Format:** JSON

**Endpoints:**

| Method | Endpoint | Purpose |
|--------|----------|---------|
| POST | `/api/sensor_data` | Send sensor readings |
| POST | `/api/gps_data` | Send GPS location |
| POST | `/api/status` | Send robot status |
| POST | `/api/camera/frame` | Send camera frame |
| GET | `/api/waypoints` | Get waypoints to navigate |

**WebSocket Events:**
- `sensor_update` - Real-time sensor data
- `gps_update` - Real-time GPS position
- `status_update` - Robot status changes
- `camera_frame` - Video frames
- `waypoint_update` - Waypoint list changes

### 2. Raspberry Pi ↔ Arduino

**Protocol:** Serial UART  
**Baud Rate:** 9600  
**Format:** JSON (newline-delimited)

**Command Structure:**
```json
{
    "cmd": "COMMAND_NAME",
    "data": { ... }
}
```

**Response Structure:**
```json
{
    "type": "response|gps|status|error",
    "data": { ... },
    "status": "ok|error",
    "message": "Optional message"
}
```

**Commands:**
- `GET_GPS` - Request GPS data
- `SET_WAYPOINTS` - Send waypoints
- `NAV_CONTROL` - Navigation control (START/STOP/PAUSE/RESUME)
- `GET_STATUS` - Request Arduino status
- `CALIBRATE_COMPASS` - Calibrate compass

---

## Software Architecture

### Dashboard Application

**Framework:** Flask (Python Web Framework)  
**Real-time:** Flask-SocketIO  
**Database:** SQLAlchemy ORM with SQLite

**Structure:**
```
dashboard/
├── app.py              # Main Flask application
├── database.py         # Database models
├── config.py           # Configuration
├── templates/          # HTML templates
│   └── index.html      # Main dashboard page
└── static/             # Static assets
    ├── css/
    │   └── style.css   # Dashboard styling
    └── js/
        ├── main.js     # Main JavaScript logic
        ├── map.js      # Map and GPS handling
        ├── sensors.js  # Sensor data handling
        └── camera.js   # Camera feed handling
```

### Raspberry Pi Application

**Language:** Python 3.7+  
**Architecture:** Multi-threaded with separate loops

**Structure:**
```
raspberry_pi/
├── main.py                    # Main control loop
├── config.json                # Configuration
├── sensors/
│   ├── sensor_manager.py      # Sensor coordinator
│   ├── dht_sensor.py          # Temperature/humidity
│   └── mq_sensors.py          # Gas sensors
├── communication/
│   ├── api_client.py          # Dashboard API client
│   ├── gsm_module.py          # GSM communication
│   └── serial_comm.py         # Arduino serial comm
├── camera/
│   └── camera_stream.py       # Camera capture
└── utils/
    ├── logger.py              # Logging setup
    └── data_formatter.py      # Data formatting
```

**Threads:**
1. **Sensor Loop** - Read sensors every 5 seconds
2. **GPS Loop** - Request GPS from Arduino every 2 seconds
3. **Status Loop** - Send status updates every 10 seconds
4. **Camera Loop** - Capture and stream frames at 5 FPS
5. **Waypoint Loop** - Check for new waypoints every 5 seconds

### Arduino Application

**Language:** C++ (Arduino)  
**Architecture:** Single-threaded event loop

**Structure:**
```
arduino_mega/robot_navigation/
├── robot_navigation.ino       # Main sketch
├── gps_handler.h/.cpp         # GPS module
├── compass_handler.h/.cpp     # Compass/magnetometer
├── navigation.h/.cpp          # Navigation logic
├── motor_control.h/.cpp       # Motor control
└── obstacle_avoidance.h/.cpp  # Ultrasonic sensor
```

**Loop Flow:**
```
1. Update GPS
2. Update Compass
3. Check for obstacles
4. Execute navigation (if active)
5. Handle serial commands
6. Send periodic status
```

---

## Data Models

### Sensor Reading
```python
{
    "id": Integer (Primary Key),
    "timestamp": DateTime,
    "device_id": String,
    "temperature": Float,      # Celsius
    "humidity": Float,         # Percentage
    "mq2": Integer,            # Smoke/LPG
    "mq135": Integer,          # Air quality
    "mq7": Integer             # Carbon monoxide
}
```

### GPS Location
```python
{
    "id": Integer (Primary Key),
    "timestamp": DateTime,
    "device_id": String,
    "latitude": Float,
    "longitude": Float,
    "altitude": Float,         # Meters
    "speed": Float,            # m/s
    "heading": Float,          # Degrees (0-360)
    "satellites": Integer
}
```

### Waypoint
```python
{
    "id": Integer (Primary Key),
    "device_id": String,
    "latitude": Float,
    "longitude": Float,
    "sequence": Integer,       # Order
    "description": String,
    "created_at": DateTime,
    "completed": Boolean,
    "completed_at": DateTime
}
```

### Robot Status
```python
{
    "id": Integer (Primary Key),
    "device_id": String,
    "online": Boolean,
    "battery_level": Float,    # Percentage
    "signal_strength": Integer, # dBm
    "last_update": DateTime,
    "system_info": JSON        # CPU, memory, temp
}
```

---

## Navigation Algorithm

### Waypoint Following

1. **Load Waypoints** - Receive list from dashboard
2. **Select Current** - Start with waypoint #1
3. **Calculate Distance** - Haversine formula to current waypoint
4. **Calculate Bearing** - Target direction to waypoint
5. **Get Heading** - Current direction from compass
6. **Adjust Motors** - PID-like control to follow bearing
7. **Check Arrival** - If distance < 5m, mark complete
8. **Next Waypoint** - Move to next in sequence

### Obstacle Avoidance

1. **Continuous Scanning** - Ultrasonic sensor checks every 200ms
2. **Detection** - If distance < 30cm
3. **Stop Motors** - Immediate halt
4. **Avoidance Maneuver:**
   - Turn right 1 second
   - Move forward 1 second
   - Turn left 1 second
5. **Resume** - Return to waypoint navigation

---

## Power Management

### Power Distribution

```
12V Battery ──┬─→ L298N Motor Driver ─→ DC Motors
              │
              └─→ 5V Regulator ──┬─→ Raspberry Pi
                                  ├─→ Arduino Mega (via USB)
                                  ├─→ Sensors
                                  └─→ GSM Module
```

### Battery Monitoring

- Voltage divider circuit on Arduino analog pin
- Sends battery percentage to Raspberry Pi
- Low battery alert at 20%

---

## Security Considerations

1. **API Authentication:** Add API keys for production
2. **HTTPS:** Use SSL/TLS for encrypted communication
3. **SSH:** Secure Raspberry Pi access
4. **Firewall:** Restrict unnecessary ports
5. **Data Validation:** Sanitize all inputs

---

## Scalability

### Multi-Robot Support

The system supports multiple robots:
- Each robot has unique `device_id`
- Dashboard filters data by `device_id`
- Database indexes on `device_id` for performance

### ESP32 Integration

To add ESP32 boards:
1. Add `device_id` to config
2. Implement same API endpoints
3. Dashboard automatically supports via `device_id`

---

## Future Enhancements

1. **Machine Learning:** Anomaly detection in sensor data
2. **Path Planning:** A* algorithm for optimal routes
3. **Swarm Control:** Multiple robots coordination
4. **Voice Control:** Integration with speech recognition
5. **Mobile App:** Native iOS/Android application
6. **Edge Computing:** On-device AI processing

---

## Performance Metrics

- **Sensor Update Rate:** 5 seconds (configurable)
- **GPS Update Rate:** 2 seconds
- **Camera Frame Rate:** 5 FPS (configurable up to 30 FPS)
- **Navigation Update:** 50ms (20 Hz)
- **Waypoint Accuracy:** ±5 meters
- **Obstacle Detection:** 2-400 cm range

---

## Conclusion

This architecture provides a robust, scalable foundation for an environmental monitoring robot system suitable for a telecommunications engineering undergraduate project. The modular design allows for easy expansion and modification of individual components.
