# Arduino Mega 2560 - Pin Reference Card

## Quick Pin Lookup

### Digital Pins (0-53)

| Pin | Usage | Device | Notes |
|-----|-------|--------|-------|
| 0 | RX0 | Serial0 (Debug) | Reserved |
| 1 | TX0 | Serial0 (Debug) | Reserved |
| 2 | **KY-032 DO** | **IR Obstacle Sensor** | **NEW: Digital input, HIGH = obstacle** |
| 3 | Available | - | Can be used for future sensors |
| 4 | Available | - | Can be used for future sensors |
| 5 | Available | - | Can be used for future sensors |
| 6 | Available | - | Can be used for future sensors |
| 7 | Available | - | Can be used for future sensors |
| 8 | TRIG | HC-SR04 Ultrasonic | Trigger output (10µs pulse) |
| 9 | ECHO | HC-SR04 Ultrasonic | Echo input (measures pulse width) |
| 10 | BUZZER | Audio Output | Passive buzzer (PWM compatible) |
| 11 | SERVO | SG90 Servo Motor | PWM control (1000-2000µs) |
| 12 | Available | - | Can be used for future sensors |
| 13 | LED | Built-in | Arduino internal LED |
| 14 | RX3 | Serial3 (BLE) | Bluetooth RX |
| 15 | TX3 | Serial3 (BLE) | Bluetooth TX |
| 16 | TX2 | Serial2 (ZigBee) | ZigBee TX |
| 17 | RX2 | Serial2 (ZigBee) | ZigBee RX |
| 18 | TX1 | Serial1 (GPS) | GPS TX |
| 19 | RX1 | Serial1 (GPS) | GPS RX |
| 20 | SDA | I2C | Compass + Raspberry Pi |
| 21 | SCL | I2C | Compass + Raspberry Pi |
| 22-49 | Various | - | Available for expansion |
| 50 | MISO | SPI (LoRa) | LoRa SPI Data In |
| 51 | MOSI | SPI (LoRa) | LoRa SPI Data Out |
| 52 | SCK | SPI (LoRa) | LoRa SPI Clock |
| 53 | CS | SPI (LoRa) | LoRa Chip Select |

### Analog Pins (A0-A15)

| Pin | Usage | Device | Notes |
|-----|-------|--------|-------|
| **A0** | **KY-032 AO** | **IR Obstacle Sensor** | **NEW: Analog input (0-1023)** |
| A1 | Available | - | Can be used for sensors |
| A2 | Available | - | Can be used for sensors |
| A3 | Available | - | Can be used for sensors |
| A4 | Available | - | Can be used for sensors |
| A5 | Available | - | Can be used for sensors |
| A6 | Available | - | Can be used for sensors |
| A7 | Available | - | Can be used for sensors |
| A8 | Available | - | Can be used for sensors |
| A9 | Available | - | Can be used for sensors |
| A10 | Available | - | Can be used for sensors |
| A11 | Available | - | Can be used for sensors |
| A12 | Available | - | Can be used for sensors |
| A13 | Available | - | Can be used for sensors |
| A14 | Available | - | Can be used for sensors |
| A15 | Available | - | Can be used for sensors |

---

## Current System Configuration

### Sensor Devices

```
HC-SR04 Ultrasonic (Servo-Mounted):
  ├─ VCC  → +5V
  ├─ GND  → GND
  ├─ TRIG → Pin 8 (digital output)
  └─ ECHO → Pin 9 (digital input)

SG90 Servo Motor:
  ├─ VCC  → +5V
  ├─ GND  → GND
  └─ PWM  → Pin 11 (PWM output)

KY-032 IR Obstacle Sensor:
  ├─ VCC  → +5V
  ├─ GND  → GND
  ├─ DO   → Pin 2 (digital input)
  └─ AO   → A0 (analog input)

Passive Buzzer:
  ├─ +    → Pin 10
  └─ -    → GND (via 470Ω resistor recommended)
```

### Communication Interfaces

```
Serial0 (UART): Debug/Programming
  ├─ RX0 (Pin 0)  ← Computer
  └─ TX0 (Pin 1)  → Computer
  Baud: 115200

Serial1 (UART): GPS Module
  ├─ RX1 (Pin 19) ← NEO-6M TX
  ├─ TX1 (Pin 18) → NEO-6M RX (usually not used)
  └─ Baud: 9600

Serial2 (UART): ZigBee (if selected)
  ├─ RX2 (Pin 17) ← XBee TX
  ├─ TX2 (Pin 16) → XBee RX
  └─ Baud: 57600

Serial3 (UART): Bluetooth (if selected)
  ├─ RX3 (Pin 15) ← HC-05 TX
  ├─ TX3 (Pin 14) → HC-05 RX
  └─ Baud: 38400 (HC-05) or 9600 (HM-10)

I2C (Two-wire):
  ├─ SDA (Pin 20) ↔ HMC5883L Compass
  ├─ SCL (Pin 21) ↔ HMC5883L Compass
  └─ Also connected to Raspberry Pi I2C

SPI (if LoRa selected):
  ├─ SCK  (Pin 52) → RFM95W Clock
  ├─ MOSI (Pin 51) → RFM95W Data Out
  ├─ MISO (Pin 50) ← RFM95W Data In
  ├─ NSS  (Pin 53) → RFM95W Chip Select
  └─ RST  (Pin 8)  → RFM95W Reset
```

### Wireless Protocol Selection

**In `globals.h`, uncomment ONE protocol:**

```cpp
#define WIRELESS_PROTOCOL_ZIGBEE  1    // Uses Serial2 (pins 16-17)
// #define WIRELESS_PROTOCOL_LORA  1    // Uses SPI (pins 50-53)
// #define WIRELESS_PROTOCOL_BLE   1    // Uses Serial3 (pins 14-15)
```

| Protocol | Interface | Baud | Pins Used | Range |
|----------|-----------|------|-----------|-------|
| ZigBee | Serial2 | 57600 | 16, 17 | ~100m |
| LoRa | SPI | N/A | 50, 51, 52, 53, 8 | ~10km |
| Bluetooth | Serial3 | 38400 | 14, 15 | ~100m |

---

## Pin Usage Summary

### Total Pins Used: 22/54

```
RESERVED/FIXED:
  ├─ Pins 0-1   (Serial0 debug)
  ├─ Pin 18-19  (Serial1 GPS)
  ├─ Pins 20-21 (I2C compass)
  └─ Pins 50-52 (SPI for LoRa)

REQUIRED SENSORS:
  ├─ Pins 8-9   (HC-SR04 ultrasonic)
  ├─ Pin 11     (Servo motor)
  ├─ Pin 10     (Buzzer)
  ├─ Pin 2      (KY-032 IR DO)
  └─ Pin A0     (KY-032 IR AO)

WIRELESS (select ONE):
  ├─ Pins 16-17 (ZigBee) OR
  ├─ Pins 14-15 (Bluetooth) OR
  └─ Pins 50-53 (LoRa SPI)

AVAILABLE FOR EXPANSION: 32+ pins
  ├─ Pins 3-7, 12-13, 22-49
  ├─ Analog pins A1-A15
  └─ Can add cameras, additional sensors, etc.
```

---

## Adding New Devices

### Available Digital Pins
```
Pins 3, 4, 5, 6, 7, 12, 13, 22-49
(Choose based on PWM needs or special features)
```

### Available Analog Pins
```
A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
(Choose based on number of analog sensors needed)
```

### Example: Adding a Temperature Sensor
```cpp
#define TEMP_SENSOR_PIN A1  // Use any available analog pin
```

### Example: Adding an Additional Servo
```cpp
#define SERVO2_PIN 12  // Use any available PWM pin (3-6, 12-13, 44-46)
```

---

## Power Considerations

### Power Supply Requirements

```
+5V Supply:
  ├─ Arduino Mega (500mA)
  ├─ HC-SR04 ultrasonic (15mA max)
  ├─ KY-032 IR sensor (20mA max)
  ├─ Servo motor (500mA peak, 100mA idle)
  ├─ L298N motor driver (varies with motors)
  └─ Wireless module (100-200mA depending on protocol)
  
Total: 1-2A recommended power supply

+3.3V Supply (if needed):
  ├─ LoRa module (RFM95W): 100mA
  └─ Use Arduino 3.3V pin or separate regulator
```

### Voltage Levels

| Device | Input Voltage | Logic Level |
|--------|---------------|-------------|
| Arduino Mega | 7-12V (USB or barrel jack) | 5V |
| HC-SR04 | 5V | 5V |
| KY-032 | 5V | 5V logic compatible |
| Servo | 5V | 5V PWM |
| I2C (compass) | 5V | 5V |
| GPS (Serial1) | 5V | 5V UART |
| ZigBee (Serial2) | 5V | 5V UART |
| Bluetooth (Serial3) | 5V | 5V UART |
| LoRa (SPI) | 3.3V | 3.3V SPI |

---

## Pin Change Notes

### Changes in This Session (KY-032 Integration)

**Added:**
- Pin 2: KY-032 DO (digital input)
- Pin A0: KY-032 AO (analog input)

**No pins reassigned** - Both new pins were available

**Verification:**
- ✅ Pin 2 not used by other components
- ✅ Pin A0 not used by other components
- ✅ No SPI conflicts with wireless protocols
- ✅ I2C line (pins 20-21) unaffected
- ✅ Serial ports (18-19, 16-17, 14-15) unaffected

---

## Servo Angle Reference

### SG90 Servo Positions (for HC-SR04 Scanning)

```
Servo Pulse Width vs. Angle:

  20°  (RIGHT)  ← 1000 µs
  45°          ← 1250 µs
  90°  (CENTER) ← 1500 µs
  135°         ← 1750 µs
  160° (LEFT)  ← 2000 µs

Code Usage:
  servo.write(20);   // Look right
  servo.write(90);   // Look center
  servo.write(160);  // Look left
```

---

## Configuration Macros

### In `globals.h` or `obstacle_avoidance.h`

```cpp
// Ultrasonic sensor
#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO 9
#define OBSTACLE_THRESHOLD 30  // cm

// Servo motor
#define SERVO_PIN 11
#define SERVO_CENTER 90
#define SERVO_LEFT 160
#define SERVO_RIGHT 20
#define SERVO_DELAY 300  // ms for servo to stabilize

// Infrared sensor (NEW)
#define KY032_DO_PIN 2      // Digital output
#define KY032_AO_PIN A0     // Analog output
#define KY032_DETECTION_THRESHOLD 600  // ADC threshold

// Audio
#define BUZZER_PIN 10

// Wireless protocol (select ONE)
#define WIRELESS_PROTOCOL_ZIGBEE 1
// #define WIRELESS_PROTOCOL_LORA 1
// #define WIRELESS_PROTOCOL_BLE 1
```

---

## Quick Troubleshooting

### "Pin already in use" Compile Error
1. Search codebase for conflicting `#define PIN`
2. Choose available pin from reference above
3. Update pin definition and wiring

### Servo Moving Erratically
1. Check servo power supply (should be 5V/1A minimum)
2. Verify PWM signal on pin 11 (scope to confirm 1-2ms pulses)
3. Check for voltage sag when servo moves (add capacitor to power)

### Sensor Not Detecting
1. Verify correct pins used
2. Check pinMode() configuration
3. Test with Serial.print() to debug values
4. Measure voltage at sensor pins (should be 5V for digital, 0-5V for analog)

### Wireless Module Not Responding
1. Check serial port assignment (Serial1/2/3)
2. Verify baud rate matches module (57600 for ZigBee, 38400 for HC-05)
3. Confirm RX/TX lines not crossed
4. Test with serial monitor to verify communication

---

## Pin Assignment History

| Phase | Component | Pin(s) | Date |
|-------|-----------|--------|------|
| Initial | GPS | Serial1 (18-19) | Dec 10 |
| Initial | Compass | I2C (20-21) | Dec 10 |
| Initial | HC-SR04 | 8-9 | Dec 10 |
| Initial | Servo | 11 | Dec 10 |
| Initial | Buzzer | 10 | Dec 10 |
| Wireless | ZigBee | Serial2 (16-17) | Dec 10 |
| Wireless | Bluetooth | Serial3 (14-15) | Dec 10 |
| Wireless | LoRa | SPI (50-53) | Dec 10 |
| KY-032 | IR Sensor | 2, A0 | Dec 10 ← Latest |

---

## References

- **Arduino Mega Pinout**: https://store.arduino.cc/products/arduino-mega-2560-rev3
- **HC-SR04 Datasheet**: Common ultrasonic sensor specs
- **KY-032 Datasheet**: Infrared obstacle sensor specs
- **SG90 Servo**: Common PWM servo motor specs
- **NEO-6M GPS**: u-blox GPS module
- **HMC5883L Compass**: 3-axis magnetometer

Your complete Arduino Mega pin configuration is now documented! 🎯

