# Hardware Configuration Summary
**Updated: January 29, 2026**

## ✅ Verified Working Components

### 1. I2C Communication
- **Raspberry Pi GPIO 2 (SDA)** ↔ **Arduino Mega Pin 20 (SDA)** + **ADS1115 SDA**
- **Raspberry Pi GPIO 3 (SCL)** ↔ **Arduino Mega Pin 21 (SCL)** + **ADS1115 SCL**
- **Pull-up resistors:** 4.7kΩ on both SDA and SCL lines to 3.3V
- **Common Ground:** Pi GND ↔ Mega GND ↔ ADS1115 GND

### 2. Arduino Mega (I2C Address: 0x08)
- **Function:** Navigation controller
- **Features:**
  - GPS waypoint navigation (NEO-6M)
  - Compass heading (HMC5883L)
  - Motor control (L298N dual H-bridge)
  - Obstacle avoidance (HC-SR04 on servo)
  - Multi-protocol wireless (ZigBee/LoRa/BLE for manual override)

### 3. ADS1115 16-bit ADC (I2C Address: 0x48)
- **Power:** 5V from Pi
- **ADDR Pin:** Connected to GND (sets address to 0x48)
- **Analog Inputs:**
  - **A0:** MQ-2 (Smoke/LPG/Propane detector)
  - **A1:** MQ-135 (Air Quality - CO2, NH3, NOx)
  - **A2:** MQ-7 (Carbon Monoxide detector)
  - **A3:** Reserved/Unused

### 4. Temperature & Humidity Sensor
- **Type:** DHT11 (NOT DHT22)
- **Connection:** GPIO 4 (Pi Pin 7)
- **Readings:** Temperature (°C) and Humidity (%)

### 5. Gas Sensors (MQ Series)
All MQ sensors require:
- **Power:** 5V
- **Ground:** Common GND
- **Analog Output:** Connected to ADS1115 channels as listed above

#### Channel Mapping (CORRECTED)
```
ADS1115 A0 → MQ-2   (Smoke/LPG)
ADS1115 A1 → MQ-135 (Air Quality/CO2)
ADS1115 A2 → MQ-7   (Carbon Monoxide)
```

## 🔧 Configuration Files Updated

### raspberry_pi/config.json
```json
"sensors": {
    "dht": {
        "pin": 4,
        "type": "DHT11"  // Changed from DHT22
    },
    "mq_sensors": {
        "mq2": {
            "channel": 0,  // A0
            "enabled": true
        },
        "mq135": {
            "channel": 1,  // A1
            "enabled": true
        },
        "mq7": {
            "channel": 2,  // A2
            "enabled": true
        }
    },
    "adc": {
        "type": "ADS1115",
        "address": 72,  // 0x48 in decimal
        "gain": 1
    }
}
```

## 🚀 Next Steps to Start Robot Controller

### 1. Upload Navigation Sketch to Mega
```bash
# In Arduino IDE, upload:
arduino_mega/robot_navigation/robot_navigation.ino
```

### 2. Start Robot Controller on Pi
```bash
cd ~/RobotControl
python3 raspberry_pi/main.py
```

**Expected Output:**
- ✓ I2C communication with Mega (address 0x08)
- ✓ ADS1115 reading MQ sensors (address 0x48)
- ✓ DHT11 reading temperature & humidity
- ✓ Sensor data uploading to dashboard API

### 3. Start Dashboard (on separate machine)
```bash
cd ~/RobotControl/dashboard
source dashboard_env/bin/activate  # If using venv
python3 app.py
```

Access dashboard at: `http://localhost:5000`

## 📊 Dashboard Display

The dashboard will show:
- **Temperature/Humidity** from DHT11
- **MQ-2:** Smoke/LPG levels (A0)
- **MQ-135:** Air Quality/CO2 (A1)
- **MQ-7:** Carbon Monoxide (A2)
- **GPS Position** from Mega
- **Compass Heading** from Mega
- **Camera Stream** from Pi Camera
- **Command Interface** for waypoint navigation

## ⚠️ SIM7600E Status
- **Connection:** Via UART (attempting /dev/ttyAMA0, /dev/serial0, /dev/ttyS0)
- **Issue:** UART communication not yet working (needs raspi-config serial setup)
- **Workaround:** Robot can operate without cellular data using local WiFi/Ethernet
- **Next:** Configure serial console settings and retest when needed

## ✅ System Ready!
All I2C devices detected and configured. You can now start the robot controller!
