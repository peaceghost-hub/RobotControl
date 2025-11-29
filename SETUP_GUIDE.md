# Complete Setup Guide
## ARM-Based Environmental Monitoring and Aide Robot

---

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Software Requirements](#software-requirements)
3. [Dashboard Setup](#dashboard-setup)
4. [Raspberry Pi Setup](#raspberry-pi-setup)
5. [Arduino Mega Setup](#arduino-mega-setup)
6. [Hardware Assembly](#hardware-assembly)
7. [Testing](#testing)
8. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Raspberry Pi 3 Model B Components:
- **Main Board:** Raspberry Pi 3 Model B
- **Sensors:**
  - DHT22 Temperature & Humidity Sensor
  - MQ-2 Gas Sensor (Smoke, LPG)
  - MQ-135 Gas Sensor (Air Quality, CO2)
  - MQ-7 Gas Sensor (Carbon Monoxide)
  - MCP3008 ADC (Analog to Digital Converter)
- **Camera:** Raspberry Pi Camera Module V2
- **Communication:** SIM800L/SIM900 GSM Module
- **Power:** 5V 3A Power Supply

### Arduino Mega 2560 Components:
- **Main Board:** Arduino Mega 2560
- **Navigation Sensors:**
  - NEO-6M GPS Module
  - HMC5883L Compass/Magnetometer
  - HC-SR04 Ultrasonic Sensor
- **Motor Control:**
  - L298N Motor Driver
  - 2x DC Motors with wheels
  - 12V Battery for motors

### Additional Components:
- Jumper wires (Male-Male, Male-Female, Female-Female)
- Breadboard or PCB for prototyping
- USB cable (Arduino to Raspberry Pi)
- Robot chassis
- Power distribution board

---

## Software Requirements

### Dashboard Server (PC/Cloud):
- Python 3.7+
- pip (Python package manager)
- SQLite3 (or MySQL)
- Modern web browser

### Raspberry Pi:
- Raspbian OS (Raspberry Pi OS)
- Python 3.7+
- Git

### Arduino Mega:
- Arduino IDE 1.8.x or 2.x
- USB drivers for Arduino Mega

---

## Dashboard Setup

### 1. Install Python Dependencies

```bash
cd /home/thewizard/RobotControl/dashboard
pip install -r requirements.txt
```

### 2. Configure Dashboard

Edit `config.py`:
```python
DEBUG = False  # Set to False in production
PORT = 5000
SECRET_KEY = 'your-secret-key-here'  # Change this!
```

### 3. Initialize Database

```bash
cd /home/thewizard/RobotControl/database
python init_db.py
```

### 4. Start Dashboard Server

```bash
cd /home/thewizard/RobotControl/dashboard
python app.py
```

The dashboard will be accessible at: `http://localhost:5000`

---

## Raspberry Pi Setup

### 1. Prepare Raspberry Pi

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade -y

# Install system dependencies
sudo apt-get install -y python3-pip python3-dev git
sudo apt-get install -y libatlas-base-dev libjasper-dev
sudo apt-get install -y libqtgui4 libqt4-test
```

### 2. Enable Required Interfaces

```bash
sudo raspi-config
```

Enable:
- **Camera** (Interface Options → Camera)
- **SPI** (Interface Options → SPI)
- **I2C** (Interface Options → I2C)
- **Serial** (Interface Options → Serial) - Disable login shell, enable hardware

### 3. Install Python Dependencies

```bash
cd ~/RobotControl/raspberry_pi
pip3 install -r requirements.txt
```

### 4. Configure Raspberry Pi

Copy and edit configuration:
```bash
cp config.json.example config.json
nano config.json
```

Update:
- `dashboard_api.base_url` → Your dashboard server URL
- `gsm.port` → Your GSM module port (usually `/dev/ttyUSB0`)
- `gsm.apn` → Your mobile carrier's APN
- `arduino.port` → Arduino port (usually `/dev/ttyACM0`)

### 5. Test Sensors

```bash
# Test DHT sensor
python3 -c "from sensors.dht_sensor import DHTSensor; s = DHTSensor(4, 'DHT22'); print(s.read())"

# Test MQ sensors
python3 -c "from sensors.mq_sensors import MQSensors; import json; config = json.load(open('config.json')); s = MQSensors(config['sensors']['mq_sensors'], config['sensors']['adc']); print(s.read_all())"
```

### 6. Auto-Start on Boot (Optional)

Create systemd service:
```bash
sudo nano /etc/systemd/system/robot.service
```

Add:
```ini
[Unit]
Description=Robot Control System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/RobotControl/raspberry_pi
ExecStart=/usr/bin/python3 /home/pi/RobotControl/raspberry_pi/main.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable robot.service
sudo systemctl start robot.service
```

---

## Arduino Mega Setup

### 1. Install Arduino IDE

Download from: https://www.arduino.cc/en/software

### 2. Install Required Libraries

Open Arduino IDE:
1. **Sketch → Include Library → Manage Libraries**
2. Install:
   - **TinyGPSPlus** by Mikal Hart
   - **HMC5883L** by Adafruit
   - **ArduinoJson** by Benoit Blanchon (v6.x)

### 3. Configure Board

1. **Tools → Board → Arduino AVR Boards → Arduino Mega or Mega 2560**
2. **Tools → Processor → ATmega2560 (Mega 2560)**
3. **Tools → Port → Select your Arduino port**

### 4. Update Pin Configuration

Edit `robot_navigation.ino` pin definitions if needed based on your wiring.

### 5. Upload Sketch

1. Open `arduino_mega/robot_navigation/robot_navigation.ino`
2. Click **Upload** button
3. Wait for compilation and upload
4. Open **Serial Monitor** (115200 baud) to verify

---

## Hardware Assembly

### Raspberry Pi Connections:

#### DHT22 Sensor:
```
DHT22 VCC  → Pi 5V (Pin 2)
DHT22 GND  → Pi GND (Pin 6)
DHT22 DATA → Pi GPIO 4 (Pin 7)
```

#### MCP3008 ADC (for MQ Sensors):
```
MCP3008 VDD  → Pi 3.3V (Pin 1)
MCP3008 VREF → Pi 3.3V
MCP3008 AGND → Pi GND
MCP3008 DGND → Pi GND
MCP3008 CLK  → Pi SCLK (Pin 23)
MCP3008 DOUT → Pi MISO (Pin 21)
MCP3008 DIN  → Pi MOSI (Pin 19)
MCP3008 CS   → Pi CE0 (Pin 24)
```

#### MQ Sensors:
```
MQ-2 VCC   → 5V
MQ-2 GND   → GND
MQ-2 AOUT  → MCP3008 CH0

MQ-135 AOUT → MCP3008 CH1
MQ-7 AOUT   → MCP3008 CH2
```

#### GSM Module (SIM800L):
```
SIM800L VCC → Pi 5V (with voltage regulator)
SIM800L GND → Pi GND
SIM800L TX  → Pi RX (GPIO 15, Pin 10)
SIM800L RX  → Pi TX (GPIO 14, Pin 8)
```

#### Pi Camera:
- Connect to Camera CSI port on Raspberry Pi

#### Arduino Connection:
- USB cable from Arduino Mega to Raspberry Pi USB port

### Arduino Mega Connections:

#### GPS Module (NEO-6M):
```
GPS VCC → Arduino 5V
GPS GND → Arduino GND
GPS TX  → Arduino RX1 (Pin 19)
GPS RX  → Arduino TX1 (Pin 18)
```

#### Compass (HMC5883L):
```
HMC5883L VCC → Arduino 5V
HMC5883L GND → Arduino GND
HMC5883L SCL → Arduino SCL (Pin 21)
HMC5883L SDA → Arduino SDA (Pin 20)
```

#### Ultrasonic Sensor (HC-SR04):
```
HC-SR04 VCC  → Arduino 5V
HC-SR04 GND  → Arduino GND
HC-SR04 TRIG → Arduino Pin 30
HC-SR04 ECHO → Arduino Pin 31
```

#### Motor Driver (L298N):
```
L298N IN1 → Arduino Pin 22 (Left Motor)
L298N IN2 → Arduino Pin 23 (Left Motor)
L298N ENA → Arduino Pin 5 (Left Motor PWM)

L298N IN3 → Arduino Pin 24 (Right Motor)
L298N IN4 → Arduino Pin 25 (Right Motor)
L298N ENB → Arduino Pin 6 (Right Motor PWM)

L298N +12V → 12V Battery
L298N GND  → Common Ground
L298N +5V  → Not connected (use Arduino 5V)
```

---

## Testing

### 1. Test Dashboard
```bash
cd dashboard
python app.py
# Open browser: http://localhost:5000
```

### 2. Test Raspberry Pi
```bash
cd raspberry_pi
python3 main.py
# Check logs for sensor readings
```

### 3. Test Arduino
- Open Serial Monitor in Arduino IDE
- Send test command: `{"cmd":"GET_GPS"}`
- Verify GPS response

### 4. End-to-End Test
1. Start dashboard
2. Start Raspberry Pi (wait for GSM connection)
3. Check dashboard shows "Online"
4. Send waypoint from dashboard
5. Verify robot receives and navigates

---

## Troubleshooting

### Dashboard Issues:

**Problem:** Can't access dashboard  
**Solution:** Check firewall, ensure port 5000 is open

**Problem:** Database errors  
**Solution:** Run `python database/init_db.py`

### Raspberry Pi Issues:

**Problem:** Sensors not reading  
**Solution:** 
- Check SPI/I2C enabled in `raspi-config`
- Verify wiring connections
- Test with simple scripts

**Problem:** GSM not connecting  
**Solution:**
- Check SIM card is inserted and active
- Verify APN settings in `config.json`
- Check GSM module power supply (needs 2A)

**Problem:** Camera not working  
**Solution:**
- Enable camera in `raspi-config`
- Check ribbon cable connection
- Run `vcgencmd get_camera` to verify

### Arduino Issues:

**Problem:** GPS no fix  
**Solution:**
- Test outdoors with clear sky view
- Wait 1-2 minutes for cold start
- Check GPS LED blinking

**Problem:** Compass readings erratic  
**Solution:**
- Calibrate compass (rotate 360° several times)
- Keep away from magnetic interference
- Verify I2C connections

**Problem:** Motors not responding  
**Solution:**
- Check L298N connections
- Verify 12V battery charged
- Test motors directly with battery

---

## Next Steps

1. **Calibration:** Calibrate all sensors in clean environment
2. **Field Testing:** Test in controlled outdoor area
3. **Fine-tuning:** Adjust PID parameters for navigation
4. **Safety:** Add emergency stop mechanism
5. **Documentation:** Record all modifications and results

---

## Support

For issues and questions:
- Check documentation in `docs/` folder
- Review `troubleshooting.md`
- Consult datasheets for components

---

**Good luck with your project! 🤖🎓**
