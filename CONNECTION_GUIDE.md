# Robot Control System - Device Connection Guide

This guide explains how to connect your Raspberry Pi or ESP32 devices to the Environmental Monitoring Dashboard.

## Overview

The dashboard provides REST APIs and WebSocket connections for real-time communication. Your devices can:

- **Send Data**: Sensor readings, GPS location, camera frames, system status
- **Receive Commands**: Movement commands, configuration changes, control instructions
- **Real-time Updates**: Live data streaming via WebSocket

## Network Requirements

- **Dashboard IP**: Find your dashboard's IP address (run `ip addr show` on dashboard machine)
- **Same Network**: Devices and dashboard must be on the same WiFi network
- **Port 5000**: Dashboard runs on port 5000 by default

---

## 🔴 Raspberry Pi Setup

### 1. Hardware Requirements

- Raspberry Pi 3B/4/Zero W (with WiFi)
- Sensors: DHT22 (temperature/humidity), MQ-2/MQ-135/MQ-7 (gas sensors)
- GPS module (optional, UART connection)
- Pi Camera (optional)
- ADC module for analog sensors (ADS1115)

### 2. Software Setup

Run the setup script on your Raspberry Pi:

```bash
# Download and run setup script
wget https://raw.githubusercontent.com/your-repo/setup_rpi.sh
chmod +x setup_rpi.sh
./setup_rpi.sh
```

Or manually install dependencies:

```bash
sudo apt update
sudo apt install -y python3-pip python3-picamera python3-opencv
pip3 install requests psutil Adafruit-DHT Adafruit-ADS1x15 RPi.GPIO pyserial
```

### 3. Configuration

Copy `robot_controller.py` to your Raspberry Pi and edit the configuration:

```python
# Update these values in robot_controller.py
DASHBOARD_IP = "192.168.1.100"  # Your dashboard's IP address
DEVICE_ID = "robot_01"          # Unique ID for this robot
```

### 4. Hardware Wiring

```python
# GPIO Pin Configuration (adjust as needed)
DHT_PIN = 4                    # GPIO 4 for DHT sensor
MQ_PINS = {
    'mq2': 0,                  # ADC channel 0
    'mq135': 1,                # ADC channel 1
    'mq7': 2                   # ADC channel 2
}
```

### 5. Running the Controller

```bash
# Test run
python3 robot_controller.py

# Run in background
nohup python3 robot_controller.py &

# Or as a service
sudo systemctl enable robot-controller
sudo systemctl start robot-controller
```

---

## 🔵 ESP32 Setup (MicroPython)

### 1. Hardware Requirements

- ESP32 development board
- Sensors (analog/digital)
- Optional: GPS module, motors, LEDs

### 2. Software Setup

1. **Install MicroPython** on your ESP32:
   ```bash
   # Using esptool
   pip install esptool
   esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
   esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 esp32-20220117-v1.18.bin
   ```

2. **Install required libraries**:
   ```bash
   # Use Thonny IDE or similar to upload libraries
   # Required: urequests.py, umqtt, etc.
   ```

### 3. Configuration

Upload `esp32_micropython_controller.py` to your ESP32 and update:

```python
# WiFi settings
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

# Dashboard settings
DASHBOARD_URL = "http://192.168.1.100:5000"
DEVICE_ID = "esp32_01"
```

### 4. Running

The script runs automatically on ESP32 boot. Monitor via serial:

```bash
screen /dev/ttyUSB0 115200
```

---

## 🟡 ESP32 Setup (Arduino IDE)

### 1. Hardware Requirements

- ESP32 development board
- Sensors connected to ADC pins

### 2. Software Setup

1. **Install Arduino IDE** and ESP32 board support
2. **Install required libraries**:
   - WiFi
   - HTTPClient
   - ArduinoJson
   - Wire (for I2C)

### 3. Configuration

Open `esp32_arduino_controller.ino` and update:

```cpp
// WiFi settings
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Dashboard settings
const char* DASHBOARD_URL = "http://192.168.1.100:5000";
const char* DEVICE_ID = "esp32_01";
```

### 4. Upload and Run

1. Select ESP32 board in Arduino IDE
2. Set correct COM port
3. Upload the sketch
4. Monitor serial output

---

## 🟣 Arduino Mega Setup

### 1. Hardware Requirements

- Arduino Mega 2560 microcontroller
- NEO-6M GPS module (UART/Serial1, 9600 baud)
- HMC5883L 3-axis compass (I2C)
- HC-SR04 ultrasonic sensor (servo-mounted)
- KY-032 infrared obstacle sensor
- SG90 or MG90S servo motor
- L298N motor driver
- Wireless module (select one):
  - **ZigBee**: XBee module (Serial2, 57600 baud)
  - **LoRa**: RFM95W / SX1276 module (SPI)
  - **Bluetooth**: HC-05 or HM-10 (Serial3)
- Passive buzzer
- Raspberry Pi 3B (I2C master, pins 20 & 21)

### 2. Pin Configuration

#### Arduino Mega 2560 Pin Assignments

```
DIGITAL PINS:
  Pin 2   - KY-032 DO (Digital Output) ← NEW: Infrared obstacle detection
  Pin 7   - SERVO (Ultrasonic scan servo control, PWM)
  Pin 10  - BUZZER (Passive buzzer)
  Pin 30  - HC-SR04 TRIG (Ultrasonic trigger)
  Pin 31  - HC-SR04 ECHO (Ultrasonic echo)
  Pin 14  - Serial3 TX (Bluetooth module)
  Pin 15  - Serial3 RX (Bluetooth module)
  Pin 16  - Serial2 TX (ZigBee module)
  Pin 17  - Serial2 RX (ZigBee module)
  Pin 18  - Serial1 TX (GPS module)
  Pin 19  - Serial1 RX (GPS module)
  Pin 50  - SPI MISO (LoRa if selected)
  Pin 51  - SPI MOSI (LoRa if selected)
  Pin 52  - SPI SCK (LoRa if selected)

ANALOG PINS:
  A0  - KY-032 AO (Analog Output) ← NEW: Distance proxy (0-1023)
  A14 - (Available)
  A15 - (Available)

I2C PINS:
  Pin 20 (SDA) - I2C to Raspberry Pi
  Pin 21 (SCL) - I2C to Raspberry Pi
```

### 3. Wiring Diagrams

#### KY-032 Infrared Obstacle Sensor (NEW)

```
KY-032 Module:
┌─────────────┐
│  VCC  GND   │
│  DO   AO    │
└─────────────┘
    │    │    │    │
    │    │    │    └──→ A0 (Analog Input)
    │    │    └───────→ Pin 2 (Digital Input)
    │    └────────────→ GND
    └─────────────────→ +5V

Digital Output (DO):
  • HIGH (5V) when obstacle detected at close range
  • LOW (0V) when no obstacle
  • Can be used as binary detection signal

Analog Output (AO):
  • 0-1023 ADC value (via Pin A0)
  • Higher value = closer object
  • Adjustable sensitivity via potentiometer on module
  • Detection range: ~2-30cm (adjustable)

Connection Summary:
  VCC   → +5V (Arduino Mega)
  GND   → GND (Arduino Mega)
  DO    → Pin 2 (Digital Input, HIGH when obstacle)
  AO    → A0 (Analog Input, higher value = closer)
```

#### HC-SR04 Ultrasonic Sensor (Servo-Mounted)

```
HC-SR04 Module (on servo):
┌──────────────┐
│ VCC  GND     │
│ TRIG ECHO    │
└──────────────┘
   │    │   │   │
   │    │   │   └──→ Pin 9 (Pulse Input)
   │    │   └──────→ Pin 8 (Trigger Output)
   │    └──────────→ GND
   └───────────────→ +5V

Connection Summary:
  VCC   → +5V (Arduino Mega)
  GND   → GND (Arduino Mega)
  TRIG  → Pin 8 (Digital Output, 10µs pulse triggers measurement)
  ECHO  → Pin 9 (Digital Input, measures echo pulse duration)
```

#### SG90 Servo Motor

```
Servo Motor:
┌──────────────┐
  │ Brown Red Orange │
│ GND   VCC   PWM  │
└──────────────┘
  │    │    │
  │    │    └──→ Pin 7 (PWM, 1000-2000µs pulse width)
  │    └───────→ +5V (through power supply if >2A)
  └────────────→ GND

Connection Summary:
  GND   → GND (Arduino Mega)
  VCC   → +5V (Arduino Mega or dedicated power)
  PWM   → Pin 7 (PWM output)

Servo Calibration:
  57°  (Left sweep limit)
  110° (Front-facing center)
  163° (Right sweep limit)
```

#### Wireless Module Selection

**Option A: ZigBee (XBee Module)**
```
XBee Module (on Serial2):
  VCC   → +5V
  GND   → GND
  DOUT  → Pin 16 (Serial2 RX)
  DIN   → Pin 17 (Serial2 TX)
  Baud: 57600
```

**Option B: LoRa (RFM95W/SX1276)**
```
RFM95W Module (SPI):
  VCC   → +3.3V (with 100nF capacitor)
  GND   → GND
  SCK   → Pin 52 (SPI Clock)
  MOSI  → Pin 51 (SPI Data Out)
  MISO  → Pin 50 (SPI Data In)
  NSS   → Pin 9  (Chip Select)
  RST   → Pin 8  (Reset)
  DIO0  → Pin 2  (Interrupt, if using)
```

**Option C: Bluetooth (HC-05/HM-10)**
```
HC-05 Module (on Serial3):
  VCC   → +5V
  GND   → GND
  TX    → Pin 14 (Serial3 RX)
  RX    → Pin 15 (Serial3 TX)
  Baud: 38400 (HC-05) or 9600 (HM-10)
```

#### I2C Devices and Bus Separation

```
Primary I2C (Raspberry Pi Bus 1):
  Raspberry Pi ↔ Arduino Mega (SLAVE) ↔ ADS1115
    Pi GPIO 2 (SDA)  ──→ Mega Pin 20 (SDA)
    Pi GPIO 3 (SCL)  ──→ Mega Pin 21 (SCL)
    ADS1115 SDA/SCL  ──→ Pi GPIO 2/3
    Pull-ups: 4.7kΩ to 3.3V on SDA/SCL
    Devices: 0x08 (Mega), 0x48 (ADS1115)

Secondary I2C (Compass on Mega - Software I2C):
  Compass (QMC5883L/HMC5883L) ↔ Arduino Mega (MASTER)
    Compass SDA ──→ Mega Pin 40 (software SDA)
    Compass SCL ──→ Mega Pin 41 (software SCL)
    NOTE: Compass is NOT on the Raspberry Pi I2C bus.

GPS Module (direct Serial1):
  NEO-6M → Arduino Mega Serial1
    TX → Pin 18 (Serial1 RX)
    RX → Pin 19 (Serial1 TX)
    Baud: 9600
```

Why bus separation?
- Prevents the Mega from ever performing master transactions on the Pi bus.
- Eliminates the “ACK-all-addresses” failure mode when the bus gets stuck.
- Keeps Pi bus stable with only 0x08 (Mega) and 0x48 (ADS1115) visible.

### 4. Software Configuration

Update `globals.h` to select your wireless protocol:

```cpp
// Uncomment ONE of the following wireless protocols:
#define WIRELESS_PROTOCOL_ZIGBEE  1    // Use ZigBee (XBee on Serial2)
// #define WIRELESS_PROTOCOL_LORA      1    // Use LoRa (RFM95W on SPI)
// #define WIRELESS_PROTOCOL_BLE       1    // Use Bluetooth (HC-05 on Serial3)

// Obstacle detection thresholds
#define OBSTACLE_THRESHOLD 20       // cm (HC-SR04)
#define KY032_DETECTION_THRESHOLD 600  // ADC value (KY-032)

// Compass bus (software I2C on Mega)
#define COMPASS_USE_SOFT_I2C 1
#define COMPASS_SDA_PIN 40
#define COMPASS_SCL_PIN 41
```

### 5. Upload to Arduino Mega

1. **Install Arduino IDE** and select:
   - Board: "Arduino Mega 2560"
   - Port: COM port of your Arduino
   - Programmer: "AVRISP mkII" or similar

2. **Upload the sketch**:
   ```
   File → Upload
   ```

3. **Monitor serial output**:
   ```
   Tools → Serial Monitor (115200 baud)
   ```

### 6. ESP8266 CC1101 Remote (Dual Joystick)

For the CC1101 wireless handheld transmitter in
`esp8266_remote/cc1101_remote/cc1101_remote.ino`, use this wiring:

| ESP8266 Pin | Connects To | Notes |
|---|---|---|
| `D1` | CC1101 `GDO0` | Radio status line |
| `D2` | CC1101 `CSN` | SPI chip select |
| `D3` | ADS1115 `SDA` | I2C data |
| `D4` | ADS1115 `SCL` | I2C clock |
| `D5` | CC1101 `SCK` | SPI clock |
| `D6` | CC1101 `MISO` | SPI MISO |
| `D7` | CC1101 `MOSI` | SPI MOSI |
| `D0` | Joystick 1 `SW` | Left-wheel reverse toggle input; wire switch to GND |
| `D8` | Joystick 2 `SW` | Right-wheel reverse toggle input; wire switch to GND |
| `3V3` | CC1101 `VCC`, ADS1115 `VDD`, joystick `VCC` | CC1101 is 3.3V only |
| `GND` | CC1101 `GND`, ADS1115 `GND`, joystick `GND` | Shared ground |

#### ADS1115 Channel Map

| ADS1115 Channel | Connected Control | Purpose |
|---|---|---|
| `A0` | Joystick 1 `VRX` | Reserved / unused for drive |
| `A1` | Joystick 1 `VRY` | Joystick 1 throttle axis |
| `A2` | Joystick 2 `VRX` | Reserved / unused for drive |
| `A3` | Joystick 2 `VRY` | Joystick 2 throttle axis |

#### Remote Control Behavior

- Joystick 1 directly controls the right wheel.
- Joystick 2 directly controls the left wheel.
- Each joystick uses only its `VRY` axis for tank drive.
- `D0` toggles left-wheel reverse trim and `D8` toggles right-wheel reverse trim.
- The ESP8266 transmits direct raw wheel values, not throttle/steer mixing.
- In raw manual joystick mode, obstacle detection triggers scanning first.
- Raw manual auto-avoid only starts after both controls are released while scanning is active.

### 7. Dual-Sensor Obstacle Detection

The system now uses two complementary obstacle sensors:

| Sensor | Type | Range | Speed | Purpose |
|--------|------|-------|-------|---------|
| **HC-SR04** | Ultrasonic | 2-400cm | 30ms/scan | Detailed distance, path planning |
| **KY-032** | Infrared | 2-30cm | <1ms | Immediate warning, collision prevention |

**Detection Logic**:
- **KY-032** (infrared) provides immediate detection when obstacle is very close
- **HC-SR04** (ultrasonic) scans three directions (left, center, right) for path planning
- **Combined**: Robot reacts immediately to KY-032 alert, then scans with HC-SR04 to find clear path
- **Fault Tolerance**: If KY-032 fails, system continues with HC-SR04 only

### 8. Testing Dual-Sensor System

```cpp
// In Arduino Serial Monitor, you should see:
// # Obstacle avoidance with servo + dual sensors initialized
// # - HC-SR04 ultrasonic (pins 30-31, servo-scanned)
// # - KY-032 infrared (pin 2 digital, A0 analog)

// Test obstacle detection:
// 1. Place hand in front of robot
// 2. IR sensor detects immediately (KY-032)
// 3. Ultrasonic scans for clear path
// 4. Robot attempts to navigate around obstacle
```

---

## 📡 API Endpoints

### Data Sending (Device → Dashboard)

| Endpoint | Method | Data Type | Description |
|----------|--------|-----------|-------------|
| `/api/sensor_data` | POST | JSON | Temperature, humidity, gas sensors |
| `/api/gps_data` | POST | JSON | GPS location, speed, heading |
| `/api/camera/frame` | POST | JSON | Base64 encoded camera frames |
| `/api/status` | POST | JSON | Battery, signal, system info |

### Command Receiving (Dashboard → Device)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/commands/pending` | GET | Get pending commands |
| `/api/commands/{id}/ack` | POST | Acknowledge command execution |

### Example API Calls

#### Send Sensor Data
```python
import requests

data = {
    "timestamp": "2025-11-09T12:00:00",
    "device_id": "robot_01",
    "temperature": 25.5,
    "humidity": 60.0,
    "mq2": 150,
    "mq135": 200,
    "mq7": 100
}

response = requests.post("http://192.168.1.100:5000/api/sensor_data", json=data)
```

#### Get Pending Commands
```python
response = requests.get("http://192.168.1.100:5000/api/commands/pending?device_id=robot_01")
commands = response.json()['commands']
```

#### Acknowledge Command
```python
requests.post("http://192.168.1.100:5000/api/commands/123/ack",
              json={"status": "completed"})
```

---

## 🎮 Command System

### Built-in Commands

| Command Type | Payload | Description |
|--------------|---------|-------------|
| `move` | `{"direction": "forward", "speed": 0.5}` | Move robot |
| `stop` | `{}` | Stop all movement |
| `goto_waypoint` | `{"waypoint_id": 1}` | Navigate to waypoint |
| `led_on` | `{}` | Turn on LED (ESP32) |
| `led_off` | `{}` | Turn off LED (ESP32) |

### Custom Commands

You can extend the command system by adding new command types in the `execute_command()` method.

---

## 🔧 Troubleshooting

### Common Issues

1. **"Connection refused"**
   - Check dashboard IP address
   - Ensure dashboard is running on port 5000
   - Verify same network

2. **"WiFi not connected" (ESP32)**
   - Check WiFi credentials
   - Verify ESP32 has WiFi capabilities

3. **"Hardware not available" (RPi)**
   - Install missing libraries: `pip install Adafruit-DHT RPi.GPIO`
   - Check sensor wiring

4. **Camera not working**
   - Enable camera: `sudo raspi-config`
   - Test camera: `raspistill -o test.jpg`

### Debug Mode

Enable debug logging in the controllers:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Testing APIs

Test dashboard connectivity:

```bash
# Test health endpoint
curl http://192.168.1.100:5000/api/health

# Test sensor data send
curl -X POST http://192.168.1.100:5000/api/sensor_data \
  -H "Content-Type: application/json" \
  -d '{"device_id":"test","temperature":25.0}'
```

---

## 🚀 Next Steps

1. **Start Dashboard**: Run `python app.py` on your main machine
2. **Find Dashboard IP**: Note the IP address displayed
3. **Configure Device**: Update IP in your device controller
4. **Test Connection**: Run device controller and check dashboard
5. **Add Features**: Extend sensors, commands, or functionality as needed

Your robot control system is now ready for deployment! 🤖
