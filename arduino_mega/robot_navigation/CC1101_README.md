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
- ADS1115 ADC
- 2 analog joysticks
- See `esp8266_remote/` directory for wiring and firmware.

#### ESP8266 Transmitter Wiring

| ESP8266 Pin | Connects To | Notes |
|---|---|---|
| `D1` | CC1101 `GDO0` | Interrupt/status line |
| `D2` | CC1101 `CSN` | SPI chip select |
| `D3` | ADS1115 `SDA` | I2C data |
| `D4` | ADS1115 `SCL` | I2C clock |
| `D5` | CC1101 `SCK` | SPI clock |
| `D6` | CC1101 `MISO` | SPI MISO |
| `D7` | CC1101 `MOSI` | SPI MOSI |
| `D8` | Speed joystick `SW` | Reverse toggle input; wire switch to GND |
| `3V3` | CC1101 `VCC`, ADS1115 `VDD`, joystick `VCC` | Keep CC1101 on 3.3V only |
| `GND` | CC1101 `GND`, ADS1115 `GND`, joystick `GND` | Shared ground |

#### ADS1115 Channel Assignment

| ADS1115 Channel | Connected Control | Role in Firmware |
|---|---|---|
| `A0` | Primary joystick `VRX` | Direction steer axis |
| `A1` | Secondary joystick `VRX` | Ignored for motion (reserved) |
| `A2` | Primary joystick `VRY` | Direction forward/reverse axis |
| `A3` | Secondary joystick `VRY` | Accelerator input (north/positive only) |

#### Dual-Joystick Behavior

- Primary joystick sets the direction vector while held (supports angled directions too).
- Secondary joystick drives only from `VRY` on `A3`, positive-north movement only.
- Secondary `VRX` on `A1` is ignored for motion.
- Releasing the primary joystick clears the direction lock immediately.
- The `D8` switch preserves the existing reverse-toggle behavior.
- The transmitted packet format is unchanged: `throttle`, `steer`, `flags`, `crc`.

#### Manual Obstacle Behavior

- During raw manual joystick driving, the Mega scans when an obstacle is detected.
- It does not auto-avoid immediately in raw manual mode.
- Local avoidance only starts after the operator performs the acknowledgement tap sequence:
  `forward -> release -> forward -> release -> forward -> release -> forward`

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
