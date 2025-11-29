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