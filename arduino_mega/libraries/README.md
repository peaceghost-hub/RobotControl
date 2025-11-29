# Arduino Libraries Required

This sketch requires the following Arduino libraries to be installed via the Arduino IDE Library Manager:

## Required Libraries:

1. **TinyGPSPlus** (by Mikal Hart)
   - For GPS parsing
   - Library Manager: Search "TinyGPSPlus"

2. **HMC5883L** (by Adafruit or QMC5883LCompass)
   - For compass/magnetometer
   - Library Manager: Search "HMC5883L" or "QMC5883L"

3. **ArduinoJson** (by Benoit Blanchon)
   - For JSON communication
   - Library Manager: Search "ArduinoJson"
   - Version: 6.x

4. **Wire** (Built-in)
   - For I2C communication
   - Pre-installed with Arduino IDE

## Installation Instructions:

1. Open Arduino IDE
2. Go to: Sketch > Include Library > Manage Libraries...
3. Search for each library name
4. Click "Install"

## Alternative Installation (Manual):

If libraries are not available in Library Manager:

1. **TinyGPSPlus**: https://github.com/mikalhart/TinyGPSPlus
2. **HMC5883L**: https://github.com/jarzebski/Arduino-HMC5883L
3. **ArduinoJson**: https://github.com/bblanchon/ArduinoJson

Download and extract to: `Documents/Arduino/libraries/`

## Hardware Connections:

### GPS Module (NEO-6M):
- VCC -> 5V
- GND -> GND
- TX -> Arduino RX1 (Pin 19)
- RX -> Arduino TX1 (Pin 18)

### Compass (HMC5883L):
- VCC -> 5V
- GND -> GND
- SCL -> SCL (Pin 21)
- SDA -> SDA (Pin 20)

### Ultrasonic Sensor (HC-SR04):
- VCC -> 5V
- GND -> GND
- TRIG -> Pin 30
- ECHO -> Pin 31

### Motor Driver (L298N):
See motor_control.h for pin definitions

### Communication with Raspberry Pi:
- USB Serial connection (automatically configured)
