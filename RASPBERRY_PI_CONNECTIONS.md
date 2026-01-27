# Raspberry Pi 3B Complete Pin Connections

## Overview
This document provides the complete pin connection reference for your Raspberry Pi 3B with the actual hardware you're using: **ADS1115 ADC** and **SIM7600E LTE Module**.

---

## Raspberry Pi 3B GPIO Pinout Reference

```
                    3.3V  [1]  [2]  5V
      (I2C SDA)  GPIO 2  [3]  [4]  5V
      (I2C SCL)  GPIO 3  [5]  [6]  GND
                 GPIO 4  [7]  [8]  GPIO 14 (UART TX)
                    GND  [9]  [10] GPIO 15 (UART RX)
                GPIO 17  [11] [12] GPIO 18
                GPIO 27  [13] [14] GND
                GPIO 22  [15] [16] GPIO 23
                   3.3V  [17] [18] GPIO 24
     (SPI MOSI) GPIO 10  [19] [20] GND
     (SPI MISO) GPIO 9   [21] [22] GPIO 25
     (SPI SCLK) GPIO 11  [23] [24] GPIO 8 (SPI CE0)
                    GND  [25] [26] GPIO 7 (SPI CE1)
      (I2C SDA)  GPIO 0  [27] [28] GPIO 1 (I2C SCL)
                 GPIO 5  [29] [30] GND
                 GPIO 6  [31] [32] GPIO 12
                GPIO 13  [33] [34] GND
                GPIO 19  [35] [36] GPIO 16
                GPIO 26  [37] [38] GPIO 20
                    GND  [39] [40] GPIO 21
```

---

## Complete Connection Table

### I2C Bus Devices

| Device | VDD/VCC | GND | SDA | SCL | Address | Notes |
|--------|---------|-----|-----|-----|---------|-------|
| **ADS1115 ADC** | Pin 2 (5V) | Pin 6 (GND) | Pin 3 (GPIO 2) | Pin 5 (GPIO 3) | 0x48 | ADDR pin to GND |
| **Arduino Mega** | - | Common GND | Pin 3 (GPIO 2) | Pin 5 (GPIO 3) | 0x08 | I2C slave on Mega SDA/SCL (20/21) |

### Digital Sensors

| Sensor | VCC | GND | Data Pin | GPIO | Pin # | Notes |
|--------|-----|-----|----------|------|-------|-------|
| **DHT22** | Pin 2 (5V) | Pin 6 (GND) | GPIO 4 | 4 | 7 | Temperature & Humidity |

### Communication Modules

| Module | Connection | Pi Pin | GPIO | Notes |
|--------|------------|--------|------|-------|
| **SIM7600E LTE** | VCC | Pin 2/4 (5V) | - | Requires 2A+ power supply |
| | GND | Pin 6/9/14 | - | Common ground |
| | TX | Pin 10 | GPIO 15 | UART RX (if using UART) |
| | RX | Pin 8 | GPIO 14 | UART TX (if using UART) |
| | **USB** | USB Port | - | **Recommended** - use /dev/ttyUSB0 |
| | GPS Antenna | - | - | Connect to GPS port |
| | LTE Antenna | - | - | Connect to main antenna port |

### Analog Sensors (via ADS1115)

| Sensor | VCC | GND | Analog Out | ADS1115 Channel |
|--------|-----|-----|------------|-----------------|
| **MQ-2** (Smoke/LPG) | 5V | GND | AOUT | A0 |
| **MQ-135** (Air Quality) | 5V | GND | AOUT | A1 |
| **MQ-7** (Carbon Monoxide) | 5V | GND | AOUT | A2 |
| *Reserved* | - | - | - | A3 |

### Camera

| Device | Connection | Notes |
|--------|------------|-------|
| **Pi Camera V2** | CSI Connector | Ribbon cable to camera port |

---

## Detailed Device Wiring

### 1. ADS1115 16-bit ADC (I2C)

**Pinout:**
```
ADS1115 Module
┌─────────────┐
│ VDD     SCL │
│ GND     SDA │
│ A0      ADDR│
│ A1      ALRT│
│ A2          │
│ A3          │
└─────────────┘
```

**Connections:**
```
ADS1115 VDD   → Raspberry Pi Pin 2 (5V)
ADS1115 GND   → Raspberry Pi Pin 6 (GND)
ADS1115 SCL   → Raspberry Pi Pin 5 (GPIO 3, I2C SCL)
ADS1115 SDA   → Raspberry Pi Pin 3 (GPIO 2, I2C SDA)
ADS1115 ADDR  → GND (sets I2C address to 0x48)
ADS1115 ALRT  → Not connected (optional alert pin)
```

**I2C Address:** 0x48 (ADDR pin to GND)

**Test Command:**
```bash
sudo i2cdetect -y 1
# Should show device at 0x48
```

---

### 2. SIM7600E LTE Module with GPS

**Recommended Connection: USB**
```
SIM7600E USB Port → Raspberry Pi USB Port
Device Path: /dev/ttyUSB0
Baudrate: 115200

Power: External 5V/2A+ power supply (module is power-hungry)
GPS Antenna: Connect to GPS antenna port
LTE Antenna: Connect to main antenna port
```

**Alternative: UART Connection**
```
SIM7600E PWRKEY → GPIO 17 (Pin 11) - optional for power control
SIM7600E VCC    → Pin 2 (5V) - must provide 2A+ current
SIM7600E GND    → Pin 6 (GND)
SIM7600E TX     → Pin 10 (GPIO 15, UART RX)
SIM7600E RX     → Pin 8 (GPIO 14, UART TX)
```

**Notes:**
- **USB connection is strongly recommended** for power stability
- Module draws up to 2A during transmission
- Requires active SIM card with LTE data plan
- Built-in GPS requires clear sky view for fix

**Test Commands:**
```bash
# Check if module detected
ls -l /dev/ttyUSB*

# Test AT commands
sudo apt-get install minicom
sudo minicom -D /dev/ttyUSB0 -b 115200
# Type: AT
# Should respond: OK
```

---

### 3. DHT22 Temperature & Humidity Sensor

**Pinout (3-pin module):**
```
DHT22 Module
┌───────┐
│ VCC   │
│ DATA  │
│ GND   │
└───────┘
```

**Connections:**
```
DHT22 VCC  → Raspberry Pi Pin 2 (5V)
DHT22 DATA → Raspberry Pi Pin 7 (GPIO 4)
DHT22 GND  → Raspberry Pi Pin 6 (GND)
```

**Test Command:**
```bash
python3 -c "from sensors.dht_sensor import DHTSensor; s = DHTSensor(4, 'DHT22'); print(s.read())"
```

---

### 4. MQ Gas Sensors

All MQ sensors connect to **ADS1115 analog inputs**:

**MQ Sensor Pinout (typical):**
```
MQ Sensor Module
┌─────────────┐
│ VCC    DOUT │ (digital out - not used)
│ GND    AOUT │ (analog out - use this)
└─────────────┘
```

**Power Connections (all MQ sensors):**
```
MQ-x VCC → External 5V power supply (shared)
MQ-x GND → Common ground (shared with Pi)
```

**Analog Output Connections:**
```
MQ-2 AOUT   → ADS1115 A0 (smoke, LPG, propane)
MQ-135 AOUT → ADS1115 A1 (air quality, CO2, benzene)
MQ-7 AOUT   → ADS1115 A2 (carbon monoxide)
```

**Important Notes:**
- MQ sensors require 24-48 hour burn-in time for accuracy
- Keep sensors well-ventilated during calibration
- Each sensor draws ~150mA, total ~450mA

---

### 5. Arduino Mega 2560 (I2C Slave)

**Connection:**
```
Raspberry Pi → Arduino Mega (I2C)
Pi Pin 3 (GPIO 2, SDA) → Arduino Pin 20 (SDA)
Pi Pin 5 (GPIO 3, SCL) → Arduino Pin 21 (SCL)
Pi GND                 → Arduino GND

USB Connection:
Arduino USB Port → Raspberry Pi USB Port
Device: /dev/ttyACM0
```

**I2C Address:** 0x08 (configured in Arduino code)

**Notes:**
- Arduino acts as I2C slave, receives commands from Pi
- USB provides power and serial communication
- Common ground is essential for I2C communication

**Test I2C Connection:**
```bash
sudo i2cdetect -y 1
# Should show devices at 0x08 (Arduino) and 0x48 (ADS1115)
```

---

### 6. Pi Camera Module V2

**Connection:**
```
Camera Ribbon Cable → Raspberry Pi CSI Connector
(Located between HDMI and audio jack)
```

**Enable Camera:**
```bash
sudo raspi-config
# Interface Options → Camera → Enable
sudo reboot
```

**Test Camera:**
```bash
raspistill -o test.jpg
vcgencmd get_camera
# Should show: supported=1 detected=1
```

---

## Power Requirements

### Power Budget

| Device | Voltage | Current | Notes |
|--------|---------|---------|-------|
| Raspberry Pi 3B | 5V | 700mA | Idle, up to 1.4A under load |
| SIM7600E | 5V | 300mA idle, 2A transmit | Use external supply |
| DHT22 | 5V | 2.5mA | Negligible |
| MQ Sensors (×3) | 5V | 450mA total | ~150mA each |
| ADS1115 | 5V | 1mA | Negligible |
| Arduino Mega | 5V | 50mA | Via USB from Pi |
| Camera | 3.3V | 250mA | From Pi 3.3V rail |

**Total:** ~2.5A at 5V (excluding SIM7600E transmit peaks)

**Recommended Power Supply:**
- **Raspberry Pi:** 5V/3A official power supply
- **SIM7600E:** Separate 5V/2A+ power supply (or powered USB hub)
- **MQ Sensors:** Can share Pi power if using 3A supply

---

## Configuration File Reference

**raspberry_pi/config.json:**
```json
{
    "sensors": {
        "dht": {
            "pin": 4,
            "type": "DHT22"
        },
        "adc": {
            "type": "ADS1115",
            "address": 72,
            "gain": 1
        },
        "mq_sensors": {
            "mq2_channel": 0,
            "mq135_channel": 1,
            "mq7_channel": 2
        }
    },
    "sim7600e": {
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "apn": "your-carrier-apn"
    },
    "arduino": {
        "port": "/dev/ttyACM0",
        "baudrate": 115200,
        "i2c_address": 8
    }
}
```

---

## Quick Verification Commands

```bash
# Check I2C devices
sudo i2cdetect -y 1
# Expected: 0x08 (Arduino), 0x48 (ADS1115)

# Check USB serial devices
ls -l /dev/tty{ACM,USB}*
# Expected: /dev/ttyACM0 (Arduino), /dev/ttyUSB0 (SIM7600E)

# Test GPIO
gpio readall

# Check camera
vcgencmd get_camera

# Test SIM7600E
python3 -c "from communication.sim7600e_gps import SIM7600E; s = SIM7600E('/dev/ttyUSB0', 115200); print(s.get_signal_strength())"
```

---

## Common Issues & Solutions

### Issue: I2C devices not detected
**Solution:**
```bash
# Enable I2C interface
sudo raspi-config → Interface Options → I2C → Enable
sudo reboot

# Install I2C tools
sudo apt-get install i2c-tools

# Check bus
sudo i2cdetect -y 1
```

### Issue: SIM7600E not responding
**Solution:**
- Check USB connection: `lsusb` should show Qualcomm device
- Verify power supply is adequate (2A+)
- Check antenna connections
- Ensure SIM card is inserted and active

### Issue: ADS1115 not reading
**Solution:**
- Verify I2C address: `sudo i2cdetect -y 1`
- Check ADDR pin is connected to GND
- Install Python library: `pip3 install adafruit-ads1x15`
- Test with simple script

### Issue: Arduino I2C communication fails
**Solution:**
- Verify common ground connection
- Check I2C pull-up resistors (usually on modules)
- Verify Arduino I2C address (0x08)
- Test with `i2cdetect` and `i2cget`

---

## Pin Summary Table

| GPIO | Pin # | Connected To | Function |
|------|-------|--------------|----------|
| GPIO 2 | 3 | ADS1115 SDA, Arduino SDA | I2C Data |
| GPIO 3 | 5 | ADS1115 SCL, Arduino SCL | I2C Clock |
| GPIO 4 | 7 | DHT22 DATA | Temperature/Humidity |
| GPIO 14 | 8 | SIM7600E RX (optional) | UART TX |
| GPIO 15 | 10 | SIM7600E TX (optional) | UART RX |
| 5V | 2, 4 | ADS1115, DHT22, MQ sensors | Power |
| GND | 6, 9, 14, 20, 25, 30, 34, 39 | All devices | Common Ground |

**Free GPIOs for Future Use:**
- GPIO 17, 27, 22, 23, 24, 25, 5, 6, 12, 13, 19, 16, 26, 20, 21
- SPI pins (GPIO 8, 9, 10, 11) now available since not using MCP3008

---

## Safety Notes

⚠️ **Important:**
- Always connect GND first, then VCC
- Double-check polarity before powering on
- Use proper power supply ratings
- Ensure adequate cooling for Pi and SIM7600E
- Keep MQ sensors away from flammable materials during burn-in
- SIM7600E can get hot during LTE transmission

---

**Last Updated:** December 2024  
**Hardware:** ADS1115, SIM7600E LTE, DHT22, MQ sensors  
**Platform:** Raspberry Pi 3 Model B
