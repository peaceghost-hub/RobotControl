# ARM-Based Environmental Monitoring and Aide Robot

**University Project - Telecommunications Engineering**

## Project Overview

This project implements an autonomous environmental monitoring robot using ARM-based embedded systems. The robot monitors environmental parameters using gas sensors (MQ series) and temperature/humidity sensors (DHT11/DHT22), while autonomously navigating using GPS waypoints.

## System Architecture

### Hardware Components

#### Main Board: Raspberry Pi 3 Model B
- **Environmental Sensors:**
  - MQ Series Gas Sensors (MQ-2, MQ-135, etc.)
  - DHT11/DHT22 Temperature & Humidity Sensor
- **Communication:**
  - GSM Module (SIM7600E LTE with integrated GPS) for internet connectivity
  - Serial communication with Arduino Mega
- **Camera:**
  - Pi Camera Module for live video feed

#### Navigation Board: Arduino Mega 2560
- **Navigation Sensors:**
  - GPS Module (NEO-6M or similar)
  - HMC5883L Compass Module
  - HC-SR04 Ultrasonic Sensor (Obstacle Avoidance)
- **Motor Control:**
  - L298N Motor Driver (or similar)
  - DC Motors with encoders

### Software Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Web Dashboard                            │
│  (Flask Backend + HTML/CSS/JS Frontend)                     │
│  - Sensor Data Display                                       │
│  - Live Camera Feed                                          │
│  - GPS Map & Robot Tracking                                  │
│  - Waypoint Control                                          │
│  - Status Monitoring                                         │
└─────────────────────────────────────────────────────────────┘
                            ↕ (HTTP/WebSocket + GSM/WiFi)
┌─────────────────────────────────────────────────────────────┐
│              Raspberry Pi 3 Model B (Python)                │
│  - Sensor Data Collection                                    │
│  - GSM Communication                                         │
│  - Camera Streaming                                          │
│  - Data Transmission to Dashboard                           │
│  - Command Forwarding to Arduino                            │
└─────────────────────────────────────────────────────────────┘
                            ↕ (Serial UART)
┌─────────────────────────────────────────────────────────────┐
│              Arduino Mega 2560 (C++)                        │
│  - Waypoint Navigation                                       │
│  - GPS Location Tracking                                     │
│  - Compass-based Heading Control                            │
│  - Obstacle Avoidance                                        │
│  - Motor Control                                             │
└─────────────────────────────────────────────────────────────┘
```

## Features

### 1. Environmental Monitoring
- Real-time gas concentration monitoring (CO, CO2, smoke, etc.)
- Temperature and humidity tracking
- Data logging with timestamps
- Historical data analysis

### 2. Remote Dashboard
- Web-based control interface
- Real-time sensor data visualization
- Live camera feed
- Interactive map with robot location
- Waypoint management system

### 3. Autonomous Navigation
- GPS waypoint following
- Compass-based heading correction
- Obstacle detection and avoidance
- Real-time position reporting

### 4. Multi-Board Support
- Expandable to ESP32 and other microcontrollers
- Modular sensor integration
- Scalable architecture

### 5. Status Monitoring
- Robot online/offline status
- Battery/power level monitoring
- Connection quality indicators
- System health diagnostics

## Directory Structure

```
RobotControl/
├── README.md                          # This file
├── ARCHITECTURE.md                    # Detailed system architecture
├── SETUP_GUIDE.md                    # Complete setup instructions
│
├── dashboard/                         # Web Dashboard
│   ├── app.py                        # Flask server
│   ├── requirements.txt              # Python dependencies
│   ├── config.py                     # Configuration settings
│   ├── database.py                   # Database models
│   ├── static/                       # Static files
│   │   ├── css/
│   │   │   └── style.css
│   │   ├── js/
│   │   │   ├── main.js
│   │   │   ├── map.js
│   │   │   ├── sensors.js
│   │   │   └── camera.js
│   │   └── images/
│   ├── templates/                    # HTML templates
│   │   └── index.html
│   └── utils/                        # Utility modules
│       ├── sensor_handler.py
│       └── websocket_handler.py
│
├── raspberry_pi/                      # Raspberry Pi Code
│   ├── main.py                       # Main control loop
│   ├── requirements.txt              # Python dependencies
│   ├── config.json                   # Configuration file
│   ├── sensors/                      # Sensor modules
│   │   ├── mq_sensors.py            # Gas sensors
│   │   ├── dht_sensor.py            # Temperature/humidity
│   │   └── sensor_manager.py        # Sensor coordination
│   ├── communication/                # Communication modules
│   │   ├── gsm_module.py            # GSM communication
│   │   ├── serial_comm.py           # Arduino serial comm
│   │   └── api_client.py            # Dashboard API client
│   ├── camera/                       # Camera module
│   │   └── camera_stream.py
│   └── utils/                        # Utilities
│       ├── logger.py
│       └── data_formatter.py
│
├── arduino_mega/                      # Arduino Mega Code
│   ├── robot_navigation/             # Main Arduino sketch
│   │   ├── robot_navigation.ino     # Main file
│   │   ├── gps_handler.h/.cpp       # GPS module
│   │   ├── compass_handler.h/.cpp   # Compass module
│   │   ├── navigation.h/.cpp        # Navigation logic
│   │   ├── motor_control.h/.cpp     # Motor control
│   │   └── obstacle_avoidance.h/.cpp # Ultrasonic sensor
│   └── libraries/                    # Required libraries
│       └── README.md                 # Library installation guide
│
├── database/                          # Database files
│   ├── schema.sql                    # Database schema
│   └── init_db.py                    # Database initialization
│
├── docs/                              # Documentation
│   ├── circuit_diagrams/             # Wiring diagrams
│   │   ├── raspberry_pi_wiring.md
│   │   └── arduino_mega_wiring.md
│   ├── api_documentation.md          # API endpoints
│   └── troubleshooting.md            # Common issues
│
├── tests/                             # Test scripts
│   ├── test_sensors.py
│   ├── test_communication.py
│   └── test_navigation.py
│
└── scripts/                           # Utility scripts
    ├── install_dependencies.sh       # Automated setup
    ├── start_dashboard.sh            # Start web server
    └── start_robot.sh                # Start robot systems
```

## Quick Start

### Prerequisites
- Raspberry Pi 3 Model B with Raspbian OS
- Arduino Mega 2560 with Arduino IDE
- Python 3.7+ on server/local machine
- GSM module with active SIM card

### Installation

1. **Clone the repository:**
   ```bash
   cd /home/thewizard/RobotControl
   ```

2. **Set up the dashboard:**
   ```bash
   cd dashboard
   pip install -r requirements.txt
   python app.py
   ```

3. **Set up Raspberry Pi:**
   ```bash
   cd raspberry_pi
   pip install -r requirements.txt
   # Edit config.json with your settings
   python main.py
   ```

4. **Upload Arduino code:**
   - Open `arduino_mega/robot_navigation/robot_navigation.ino` in Arduino IDE
   - Install required libraries
   - Upload to Arduino Mega 2560

## Configuration

### Dashboard Configuration
Edit `dashboard/config.py` with your settings:
- Database path
- Server port
- API keys

### Raspberry Pi Configuration
Edit `raspberry_pi/config.json`:
- Dashboard API endpoint
- GSM APN settings
- Sensor GPIO pins
- Arduino serial port

### Arduino Configuration
Edit pin definitions in `robot_navigation.ino`:
- GPS serial pins
- Compass I2C address
- Motor control pins
- Ultrasonic sensor pins

## Usage

1. **Start the dashboard server** (on PC or cloud server)
2. **Power on Raspberry Pi** (will auto-connect to dashboard)
3. **Power on Arduino Mega** (connected to Raspberry Pi via USB)
4. **Access dashboard** at `http://localhost:5000`
5. **Monitor sensors** in real-time
6. **Send waypoints** via the map interface
7. **View live camera feed** during navigation

## API Endpoints

- `POST /api/sensor_data` - Receive sensor readings
- `POST /api/gps_data` - Receive GPS coordinates
- `GET /api/waypoints` - Get current waypoints
- `POST /api/waypoints` - Send new waypoints
- `GET /api/status` - Get robot status
- `WebSocket /stream` - Camera feed stream

## Contributing

This is an undergraduate university project. Feel free to fork and modify for your own educational purposes.

## License

Educational Use Only - University Project

## Author

Telecommunications Engineering Student

## Acknowledgments

- University Department of Telecommunications Engineering
- Open-source libraries and communities
- Arduino and Raspberry Pi foundations

## Project Status

🚧 **Under Development** 🚧

This project is being built step-by-step as part of a university telecommunications engineering course.

---

For detailed setup instructions, see [SETUP_GUIDE.md](SETUP_GUIDE.md)  
For system architecture details, see [ARCHITECTURE.md](ARCHITECTURE.md)
