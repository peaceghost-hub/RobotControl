# Quick Reference Card
## ARM Environmental Monitoring Robot

---

## 📱 Dashboard Access
```
URL: http://localhost:5000
or
URL: http://your-server-ip:5000
```

---

## 🚀 Quick Commands

### Start Dashboard
```bash
cd dashboard
python app.py
```

### Start Raspberry Pi
```bash
cd raspberry_pi
python3 main.py
```

### Upload Arduino Code
```
1. Open Arduino IDE
2. File → Open → arduino_mega/robot_navigation/robot_navigation.ino
3. Tools → Board → Arduino Mega 2560
4. Tools → Port → Select USB port
5. Upload (Ctrl+U)
```

---

## 📍 Pin Connections

### Raspberry Pi GPIO
| Component | Pin | GPIO |
|-----------|-----|------|
| DHT22 Data | 7 | GPIO 4 |
| MCP3008 SCLK | 23 | GPIO 11 |
| MCP3008 MISO | 21 | GPIO 9 |
| MCP3008 MOSI | 19 | GPIO 10 |
| MCP3008 CS | 24 | GPIO 8 |

### Arduino Mega
| Component | Pin |
|-----------|-----|
| GPS TX | RX1 (19) |
| GPS RX | TX1 (18) |
| Compass SDA | SDA (20) |
| Compass SCL | SCL (21) |
| Ultrasonic TRIG | 30 |
| Ultrasonic ECHO | 31 |
| Motor Left EN | 5 |
| Motor Left IN1 | 22 |
| Motor Left IN2 | 23 |
| Motor Right EN | 6 |
| Motor Right IN1 | 24 |
| Motor Right IN2 | 25 |

---

## 🔧 Configuration Files

### Raspberry Pi: `raspberry_pi/config.json`
```json
{
    "device_id": "robot_01",
    "dashboard_api": {
        "base_url": "http://your-dashboard:5000"
    },
    "gsm": {
        "apn": "your-carrier-apn"
    }
}
```

### Dashboard: `dashboard/config.py`
```python
PORT = 5000
DEBUG = False
SECRET_KEY = 'change-me'
```

---

## 🔍 Debugging

### Check Sensors
```bash
# DHT22
python3 -c "from sensors.dht_sensor import DHTSensor; print(DHTSensor(4, 'DHT22').read())"

# MQ Sensors
python3 -c "from sensors.mq_sensors import MQSensors; import json; print(MQSensors(...).read_all())"
```

### Check GPS (Arduino Serial Monitor)
```json
{"cmd":"GET_GPS"}
```

### Check GSM Signal
```bash
# On Raspberry Pi
python3 -c "from communication.gsm_module import GSMModule; g = GSMModule({...}); print(g.get_signal_strength())"
```

---

## 📊 API Endpoints

| Method | Endpoint | Purpose |
|--------|----------|---------|
| POST | `/api/sensor_data` | Send sensor readings |
| GET | `/api/sensor_data` | Get sensor history |
| POST | `/api/gps_data` | Send GPS location |
| GET | `/api/gps_data/track` | Get GPS track |
| POST | `/api/waypoints` | Add waypoints |
| GET | `/api/waypoints` | Get waypoints |
| DELETE | `/api/waypoints/{id}` | Delete waypoint |
| POST | `/api/status` | Update robot status |
| GET | `/api/status` | Get robot status |
| POST | `/api/camera/frame` | Send camera frame |
| GET | `/api/health` | Health check |

---

## 🛑 Emergency Stop

### Stop Robot Immediately
```bash
# On Raspberry Pi terminal
Ctrl+C

# Or send via Arduino Serial Monitor
{"cmd":"NAV_CONTROL","data":{"action":"STOP"}}
```

### Stop Motors Only
```bash
# Arduino Serial Monitor
{"cmd":"NAV_CONTROL","data":{"action":"STOP"}}
```

---

## 🔋 Battery Monitoring

### Check Battery Level
```bash
# Dashboard: Robot Status panel shows battery %
# Safe level: > 20%
# Warning level: < 20%
# Critical level: < 10%
```

---

## 📡 GSM Module

### Common AT Commands
```
AT                 # Test connection
AT+CSQ             # Check signal strength
AT+CIPSTATUS       # Check GPRS status
AT+CIFSR           # Get IP address
```

---

## 🗺️ Waypoint Format

### Add via Dashboard
```
1. Click on map
2. Enter coordinates
3. Or use "Add Waypoint" button
```

### Add via API
```json
POST /api/waypoints
{
    "waypoints": [
        {
            "latitude": 40.7128,
            "longitude": -74.0060,
            "description": "Point A"
        }
    ],
    "device_id": "robot_01"
}
```

---

## 📸 Camera Settings

### Adjust in `raspberry_pi/config.json`
```json
"camera": {
    "resolution": [640, 480],
    "fps": 5,
    "quality": 75,
    "rotation": 0
}
```

---

## ⚠️ Troubleshooting Quick Fixes

### Dashboard not accessible
```bash
# Check if running
ps aux | grep "python.*app.py"

# Check port
netstat -tuln | grep 5000

# Restart
cd dashboard && python app.py
```

### Sensors not reading
```bash
# Enable SPI/I2C
sudo raspi-config

# Check connections
i2cdetect -y 1  # Should show device addresses
```

### GPS no fix
```
- Go outdoors
- Wait 2-5 minutes
- Check satellites: Should be > 4
```

### Motors not moving
```
- Check 12V battery charged
- Verify L298N connections
- Test motors with battery directly
```

---

## 📞 Status Codes

### Robot Status
- ✅ **Online** - Connected to dashboard
- ⚠️ **Low Battery** - < 20%
- 🔴 **Offline** - No communication
- 🟢 **Navigating** - Following waypoints
- 🟡 **Paused** - Stopped temporarily

### GPS Status
- ✅ **Valid** - Good GPS fix (≥ 4 satellites)
- ⚠️ **Searching** - Looking for satellites
- 🔴 **No Fix** - No GPS signal

---

## 📚 Quick Links

- **Main Docs:** README.md
- **Setup Guide:** SETUP_GUIDE.md
- **Architecture:** ARCHITECTURE.md
- **Project Summary:** PROJECT_SUMMARY.md

---

## 🆘 Emergency Contacts

**Technical Support:**
- Check documentation first
- Review logs: `raspberry_pi/robot.log`
- Arduino Serial Monitor for debugging

---

## 💾 Data Export

### From Dashboard
```
1. Go to "Sensor Data History" panel
2. Click "Export Data" button
3. Downloads CSV file
```

### From Database Directly
```bash
sqlite3 database/robot_monitor.db
.mode csv
.output sensor_data.csv
SELECT * FROM sensor_readings;
.quit
```

---

## 🔐 Security Notes

- Change `SECRET_KEY` in `dashboard/config.py`
- Use HTTPS in production
- Enable firewall on dashboard server
- Secure SSH access to Raspberry Pi
- Don't expose API publicly without authentication

---

**Keep this card handy during operation!** 📋
