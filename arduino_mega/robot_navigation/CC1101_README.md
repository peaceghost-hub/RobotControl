# CC1101 SPI Wireless Implementation

This directory contains the CC1101 SPI wireless communication implementation for the robot control system.

## Hardware Requirements

### Arduino Mega (Receiver)
- CC1101 Module
- Connections:
  - CS (Pin 53) → Module CSN
  - SCK (Pin 52) → Module SCK
  - MOSI (Pin 51) → Module MOSI
  - MISO (Pin 50) → Module MISO
  - GDO0 (Pin 2) → Module GDO0
  - GDO2 (Pin 3) → Module GDO2
  - GND → GND
  - VCC → 3.3V (IMPORTANT: NOT 5V!)

### ESP8266 (Transmitter/Remote)
- NodeMCU / Wemos D1 Mini + CC1101
- See `esp8266_remote/` directory for wiring and firmware.

## Software Requirements

### Required Arduino Library
- **ELECHOUSE_CC1101** library
- Install via Arduino IDE Library Manager: Search for "ELECHOUSE_CC1101"
- Or download from: https://github.com/ELECHOUSE/CC1101-arduino

### Configuration
The CC1101 modules must use identical configuration:
- Frequency: 433 MHz
- Data Rate: 9.6 kBaud
- Modulation: 2-FSK
- Sync Word: 0xD391 (211, 145)
- CRC: Enabled

## Files

### Arduino Mega (Receiver)
- `cc1101_driver.h/cpp` - CC1101 SPI driver implementation
- `robot_navigation.ino` - Updated to use CC1101 driver

### ESP8266 (Transmitter)
- `esp8266_remote/cc1101_remote/cc1101_remote.ino` - Joystick remote using CC1101 SPI

## Protocol

### Message Structure (Standard)
```cpp
struct WirelessMessage {
  uint8_t type;      // Message type
  uint8_t length;    // Data length
  uint8_t data[64];  // Payload
};
```

### Message Structure (ESP8266 Raw Drive)
The ESP8266 sends a compact 6-byte packet for low-latency control:
```cpp
struct RawMotorPacket {
  int16_t throttle;
  int16_t steer;
  uint8_t flags;
  uint8_t crc;
};
```
The Mega driver automatically detects this packet size and processes it as `MSG_TYPE_RAW_MOTOR`.

## Usage

1. Install ELECHOUSE_CC1101 library in Arduino IDE
2. Upload `esp8266_remote/cc1101_remote/cc1101_remote.ino` to your ESP8266
3. Update `globals.h` on Mega to use `WIRELESS_PROTOCOL_CC1101`
4. Upload `robot_navigation.ino` to Arduino Mega
5. Power on both modules
6. Use joystick on ESP8266 to control Mega remotely

## Troubleshooting

- **No communication**: Check 3.3V power supply (CC1101 modules are NOT 5V tolerant)
- **Library errors**: Ensure ELECHOUSE_CC1101 library is installed
- **Pin conflicts**: Verify SPI pin connections match Arduino specifications
- **Sync Word Mismatch**: Ensure both sides use 0xD391 (211, 145)
