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

### Arduino Uno (Transmitter/Remote)
- CC1101 Module
- Connections:
  - CS (Pin 10) → Module CSN
  - SCK (Pin 13) → Module SCK
  - MOSI (Pin 11) → Module MOSI
  - MISO (Pin 12) → Module MISO
  - GDO0 (Pin 2) → Module GDO0
  - GDO2 (Pin 3) → Module GDO2
  - GND → GND
  - VCC → 3.3V (IMPORTANT: NOT 5V!)

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
- Sync Word: 0xD191
- CRC: Enabled

## Files

### Arduino Mega (Receiver)
- `cc1101_driver.h/cpp` - CC1101 SPI driver implementation
- `robot_navigation.ino` - Updated to use CC1101 driver

### Arduino Uno (Transmitter)
- `cc1101_remote/cc1101_remote.ino` - Joystick remote using CC1101 SPI

## Protocol

### Message Structure
```cpp
struct WirelessMessage {
  uint8_t type;      // Message type
  uint8_t length;    // Data length
  uint8_t data[64];  // Payload
};
```

### Message Types
- `MSG_TYPE_COMMAND` (0x01) - Motor control commands
- `MSG_TYPE_STATUS` (0x02) - Status updates
- `MSG_TYPE_HANDSHAKE` (0x05) - Connection handshake
- `MSG_TYPE_HEARTBEAT` (0x08) - Keep-alive

### Command Types
- `WIRELESS_CMD_MOTOR_FORWARD` (0x10) - Forward with speed
- `WIRELESS_CMD_MOTOR_BACKWARD` (0x11) - Backward with speed
- `WIRELESS_CMD_MOTOR_LEFT` (0x12) - Left turn with speed
- `WIRELESS_CMD_MOTOR_RIGHT` (0x13) - Right turn with speed
- `WIRELESS_CMD_MOTOR_STOP` (0x14) - Stop motors

## Usage

1. Install ELECHOUSE_CC1101 library in Arduino IDE
2. Upload `cc1101_remote.ino` to Arduino Uno
3. Update `globals.h` to use `WIRELESS_PROTOCOL_CC1101`
4. Upload `robot_navigation.ino` to Arduino Mega
5. Power on both modules with 3.3V
6. Use joystick on Uno to control Mega remotely

## Troubleshooting

- **No communication**: Check 3.3V power supply (CC1101 modules are NOT 5V tolerant)
- **Library errors**: Ensure ELECHOUSE_CC1101 library is installed
- **Pin conflicts**: Verify SPI pin connections match Arduino specifications
- **Range issues**: CC1101 has shorter range than LoRa; consider antenna placement