# Multi-Protocol Wireless Communication System

## Overview

Your robot now supports **three wireless protocols** for remote control and telemetry:
1. **ZigBee (XBee)** - Reliable, proven, long-range (up to 1.6km)
2. **LoRa** - Extremely long-range (up to 15km), low power, excellent for outdoor deployment
3. **Bluetooth** - Direct mobile device control, shorter range (10-100m)

All three protocols use the **same message structure and command format**, making them completely interchangeable.

---

## Quick Start

### 1. Choose Protocol (Edit `globals.h`)

Open `arduino_mega/robot_navigation/globals.h` and uncomment ONE:

```cpp
#define WIRELESS_PROTOCOL_ZIGBEE     // ← Default, most reliable
// #define WIRELESS_PROTOCOL_LORA    // ← Best for long range
// #define WIRELESS_PROTOCOL_BLE     // ← Best for mobile phones
```

### 2. Upload Code

Use the **NEW file** `robot_navigation_wireless.ino` instead of the old one:
- This version supports all three protocols
- Maintains full backward compatibility with ZigBee-specific code
- All existing features work (obstacle avoidance, GPS navigation, etc.)

### 3. Connect Hardware

See section below for your chosen protocol.

### 4. Test

Open Serial Monitor (115200 baud), you should see:
```
# Wireless: ZigBee (XBee) on Serial2 @ 57600 baud
# Wireless initialized: ZigBee (XBee)
```

---

## Protocol Comparison

| Feature | ZigBee (XBee) | LoRa | Bluetooth (BT) |
|---------|---------------|------|---|
| **Range** | 1-2 km | 5-15 km | 10-100 m |
| **Power** | 40-120 mA | 30-120 mA (TX varies) | 40 mA (HC-05) / 10 mA (HM-10) |
| **Baud Rate** | 57600 | SPI | 38400 (HC-05) / 9600 (HM-10) |
| **Latency** | 10-50 ms | 50-100 ms | 10-20 ms |
| **Cost** | $20-60 | $10-30 | $5-15 |
| **Best Use** | Reliable fixed routes | Outdoor exploration | Mobile phone control |
| **Ecosystem** | Mature, many options | Growing, open source | Ubiquitous, easy |
| **Line-of-Sight** | Not required | Recommended | Not required |
| **Obstacle Alerts** | ✅ Real-time | ✅ Real-time | ✅ Real-time |
| **Waypoint Upload** | Via Raspberry Pi + I2C | Via Raspberry Pi + I2C | Via Raspberry Pi + I2C |

---

## Hardware Setup by Protocol

### Option 1: ZigBee (XBee) - RECOMMENDED FOR BEGINNERS

**Module:** Digi XBee Series 1 or Series 2
**Cost:** $30-60 per pair
**Baud:** 57600
**Throughput:** Up to 250 kbps
**Range:** 1-2 km line-of-sight

#### Wiring (Arduino Mega)

```
XBee Breakout Board → Arduino Mega
═══════════════════════════════════
VCC (3.3V)          → 3.3V (via LDO regulator)
GND                 → GND
RX (Module RX)      → TX2 (Pin 16)
TX (Module TX)      → RX2 (Pin 17)
```

**IMPORTANT:** XBee modules are 3.3V only!
- Use level shifter if 5V Arduino
- Or use Arduino with 3.3V native pins

**Power Consideration:**
```
+5V → 100µF Capacitor → GND
     ↓
     LDO Regulator (AMS1117-3.3) → 3.3V line
     ↓
     XBee VCC (with 47µF capacitor to GND)
```

#### Configuration (Using X-CTU Software)

1. Configure both modules identically:
   ```
   Baud Rate: 57600
   Node Identifier: ROBOT / REMOTE (optional)
   Destination High: 0
   Destination Low: 0xFFFF (broadcast) or remote module SH:SL
   PAN ID: 0x1234 (same for all modules)
   ```

2. Test communication:
   - Both modules should show solid LED (ready)
   - Sending data from one appears on Serial Monitor of other

---

### Option 2: LoRa (SX1276/RFM95W) - BEST FOR RANGE

**Module:** HopeRF RFM95W or Semtech SX1276
**Cost:** $10-30 per pair
**Baud:** SPI @ 4 MHz
**Throughput:** Up to 50 kbps
**Range:** 5-15 km line-of-sight (configurable)

#### Wiring (Arduino Mega)

```
LoRa Module → Arduino Mega
═════════════════════════════
VCC         → 3.3V (via LDO regulator)
GND         → GND
NSS         → Pin 9 (CS)
RST         → Pin 8 (Reset)
DIO0        → Pin 2 (optional, for IRQ)
MOSI        → ICSP Pin 4 (Pin 51)
MISO        → ICSP Pin 1 (Pin 50)
SCK         → ICSP Pin 3 (Pin 52)
```

**Power:**
```
+5V → 100µF Capacitor → GND
     ↓
     LDO Regulator (AMS1117-3.3) → 3.3V line
     ↓
     LoRa VCC (with 100µF capacitor to GND, 100mA capable)
```

**Antenna:**
- 1/4 wave dipole antenna for 915 MHz (USA region)
- Length: ~82 mm for 915 MHz
- ~78 mm for 868 MHz (Europe)
- Critical: Antenna orientation and placement affects range

#### Configuration (In Code)

```cpp
// In globals.h, set region frequency:
#define WIRELESS_PROTOCOL_LORA

// In lora_driver.h:
static const uint32_t FREQ_CENTER = 915000000;  // USA
// static const uint32_t FREQ_CENTER = 868000000;  // Europe
// static const uint32_t FREQ_CENTER = 433000000;  // Asia
```

**Spreading Factor** (Higher = longer range, slower):
```cpp
// In lora_driver.h begin() function:
writeReg(REG_MODEM_CONFIG_2, 0x74);  // SF7, adjust as needed
// 0x74 = SF7 (fastest)
// 0x84 = SF8
// 0x94 = SF9
// 0xA4 = SF10
// 0xB4 = SF11
// 0xC4 = SF12 (slowest, longest range)
```

#### Library Installation

Option 1 (Recommended): **RadioHead**
```
Arduino IDE → Sketch → Include Library → Manage Libraries
Search: "RadioHead"
Install by Mike McCauley
```

Option 2: **LMIC** (More complex, better power management)
```
https://github.com/mcci-catena/arduino-lmic
```

---

### Option 3: Bluetooth - BEST FOR MOBILE CONTROL

**Module:** HC-05 (Classic BT) or HM-10 (BLE)
**Cost:** $5-15 per module
**Baud:** 38400 (HC-05) or 9600 (HM-10) fixed
**Throughput:** Up to 1.2 Mbps
**Range:** 10-100m

#### Wiring (Arduino Mega)

```
Bluetooth Module → Arduino Mega
═════════════════════════════════
VCC              → 5V
GND              → GND
RX (Module RX)   → TX3 (Pin 14)
TX (Module TX)   → RX3 (Pin 15) via 5.1k:3.3k level shifter
```

**Level Shifter for TX3:**
```
            5V
            |
         5.1kΩ
    RX ─────┴─────┬─── TX3 (Arduino)
                  |
                 3.3k
                  |
                 GND
```

This creates a voltage divider: 5V × (3.3k/(5.1k+3.3k)) ≈ 1.8V (safe for 3.3V input)

#### HC-05 Configuration

**Default Settings:**
- Baud: 38400
- PIN: 1234
- Name: HC-05

**Change Baud Rate (if needed):**
1. Hold KEY button during power-on
2. LED should blink slowly (AT mode)
3. Send via Serial Monitor: `AT+UART=9600,0,0` (or 38400)
4. Restart module

**Change Device Name:**
1. Enter AT mode (hold KEY during power-on)
2. Send: `AT+NAME=ROBOT` (for example)

#### HM-10 Configuration

**HM-10 is BLE (Low Energy), better battery life:**
```
Baud: 9600 (fixed)
Commands:
  AT+START  → Start BLE advertisement
  AT+STOP   → Stop
  AT+HELP   → Show all commands
```

To use HM-10 instead of HC-05, edit `bluetooth_driver.h`:
```cpp
#define HM10_MODE  // Uncomment this line
```

#### Mobile App Setup

**Android:**
1. Install: "Android Bluetooth Terminal" or "Serial Bluetooth Terminal"
2. Enable Bluetooth
3. Search for "HC-05" or "HM-10"
4. PIN (HC-05): 1234
5. Connect and send commands

**iOS:**
- Requires HM-10 (BLE)
- Use app: "BLE Scanner" + custom logic
- Or use: "Blynk" IoT platform (free tier available)

---

## Command Format (Same for All Protocols)

All messages use this structure:

```
[Message Type byte] [Length byte] [Data bytes...]
```

### Manual Control Commands

**Forward:** `01 05 MCTL,FORWARD,200`
**Backward:** `01 08 MCTL,BACKWARD,150`
**Left:** `01 04 MCTL,LEFT,180`
**Right:** `01 05 MCTL,RIGHT,180`
**Stop:** `01 04 MCTL,STOP`

Or simple text (easier):
```
MCTL,FORWARD,200↵
MCTL,BACKWARD,150↵
MCTL,LEFT,180↵
MCTL,RIGHT,180↵
MCTL,STOP↵
```

### Mode Switching

```
MCTL,MANUAL↵     # Enter manual control
MCTL,AUTO↵       # Return to autonomous
```

### Status Queries

```
STATUS?↵         # Request robot status
```

---

## Example: Arduino Uno Remote

### ZigBee Remote

```cpp
#include <SoftwareSerial.h>

// Software serial for XBee (Arduino Uno)
SoftwareSerial xbee(2, 3);  // RX=2, TX=3

void setup() {
  Serial.begin(9600);      // USB serial
  xbee.begin(57600);       // XBee serial
  
  Serial.println("Waiting for robot...");
  delay(2000);
  
  // Handshake
  xbee.println("PING");
}

void loop() {
  // Send commands from Serial Monitor
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Map keyboard input to robot commands
    if (cmd == "w") {
      xbee.println("MCTL,FORWARD,200");
    } else if (cmd == "s") {
      xbee.println("MCTL,BACKWARD,150");
    } else if (cmd == "a") {
      xbee.println("MCTL,LEFT,180");
    } else if (cmd == "d") {
      xbee.println("MCTL,RIGHT,180");
    } else if (cmd == "x") {
      xbee.println("MCTL,STOP");
    } else {
      xbee.println(cmd);  // Pass through custom commands
    }
  }
  
  // Display robot responses
  if (xbee.available()) {
    String response = xbee.readStringUntil('\n');
    Serial.print("Robot: ");
    Serial.println(response);
  }
}
```

### Bluetooth Remote (HC-05)

```cpp
SoftwareSerial bt(2, 3);  // RX=2, TX=3

void setup() {
  Serial.begin(9600);
  bt.begin(38400);  // HC-05 baud rate
}

void loop() {
  // Keyboard control
  if (Serial.available()) {
    char key = Serial.read();
    
    switch(key) {
      case 'w': bt.println("MCTL,FORWARD,200"); break;
      case 's': bt.println("MCTL,BACKWARD,150"); break;
      case 'a': bt.println("MCTL,LEFT,180"); break;
      case 'd': bt.println("MCTL,RIGHT,180"); break;
      case 'x': bt.println("MCTL,STOP"); break;
    }
  }
  
  // Display responses
  if (bt.available()) {
    Serial.println((char)bt.read());
  }
}
```

### Analog Joystick Control

```cpp
// Joystick: X=A0, Y=A1, Button=D7
void loop() {
  int x = analogRead(A0);  // 0-1023
  int y = analogRead(A1);  // 0-1023
  bool button = !digitalRead(7);
  
  // Deadzone
  if (abs(x - 512) < 100 && abs(y - 512) < 100) {
    xbee.println("MCTL,STOP");
    delay(50);
    return;
  }
  
  // Forward/Backward
  if (y > 600) {
    int speed = map(y, 600, 1023, 100, 255);
    xbee.print("MCTL,FORWARD,");
    xbee.println(speed);
  } else if (y < 400) {
    int speed = map(y, 400, 0, 100, 255);
    xbee.print("MCTL,BACKWARD,");
    xbee.println(speed);
  }
  
  // Left/Right
  if (x > 600) {
    xbee.println("MCTL,RIGHT,180");
  } else if (x < 400) {
    xbee.println("MCTL,LEFT,180");
  }
  
  delay(100);  // Command rate limit
}
```

---

## Troubleshooting

### Module Not Responding

**ZigBee:**
1. Check baud rate: `AT+IPL` command
2. Verify voltage: Should show 3.2-3.4V
3. Test with X-CTU software
4. Check antenna connection (if external)

**LoRa:**
1. Verify SPI pins are correct (MOSI/MISO/SCK)
2. Check CS and Reset pin connections
3. Read version register (should be 0x12): `readReg(0x42)`
4. Verify antenna installed and secure

**Bluetooth:**
1. Scan for device name in phone Bluetooth settings
2. Try PIN 1234 (HC-05 default)
3. Check baud rate (38400 for HC-05, 9600 for HM-10)
4. Verify RX level shifter if needed

### Commands Not Working

1. **Verify handshake:**
   - Serial Monitor should show wireless initialization message
   - Send `PING` from remote, expect response
   
2. **Check buffer:**
   - Messages might be split across packets
   - Ensure `\n` (newline) at end of command
   
3. **Test with dashboard:**
   - Raspberry Pi + I2C still works independently
   - Try manual commands from Serial Monitor

### Weak Signal / Disconnections

**ZigBee:**
- Check antenna (external antennas work better)
- Verify PAN ID matches on both modules
- Reduce baud rate if corruption occurs
- Check for RF interference (WiFi, microwaves)

**LoRa:**
- Higher spreading factor = longer range but slower
- Check antenna orientation
- Verify line-of-sight if possible
- Don't route antenna through metal

**Bluetooth:**
- Move closer (range is 10-100m)
- Remove obstacles between devices
- Avoid RF interference
- HC-05 has better range than HM-10

---

## Switching Protocols

To change protocols:

1. **Edit `globals.h`:**
   ```cpp
   // Comment out current:
   // #define WIRELESS_PROTOCOL_ZIGBEE
   
   // Uncomment new:
   #define WIRELESS_PROTOCOL_LORA
   ```

2. **Rebuild and upload to Mega**

3. **Disconnect old hardware, connect new**

4. **Update remote code** (if using Arduino Uno)

5. **Test via Serial Monitor** (115200 baud)

No changes needed to main navigation or obstacle avoidance code! They work identically with all protocols.

---

## Advanced: Custom Protocol Implementation

To add a new wireless protocol (LoRaWAN, NB-IoT, Sigfox, etc.):

1. Create new file: `my_protocol_driver.h`
2. Inherit from `WirelessInterface`:
   ```cpp
   class MyProtocolDriver : public WirelessInterface {
   public:
     bool begin() override { ... }
     bool send(const WirelessMessage& msg) override { ... }
     bool receive(WirelessMessage& msg) override { ... }
     bool isConnected() const override { ... }
     const char* getProtocolName() const override { return "My Protocol"; }
   };
   ```

3. Add to `robot_navigation_wireless.ino`:
   ```cpp
   #elif defined(WIRELESS_PROTOCOL_MYPROTO)
     #include "my_protocol_driver.h"
     MyProtocolDriver wireless;
   ```

4. Add to `globals.h`:
   ```cpp
   // #define WIRELESS_PROTOCOL_MYPROTO
   ```

All existing code continues to work without modification!

---

## Performance Specifications

### Latency (Command to Motor Response)
- **ZigBee:** 50-150 ms
- **LoRa:** 100-500 ms
- **Bluetooth:** 20-100 ms

### Throughput (Practical)
- **ZigBee:** 100-200 bytes/sec
- **LoRa:** 10-50 bytes/sec
- **Bluetooth:** 1000+ bytes/sec

### Power Consumption (When Transmitting)
- **ZigBee:** 80-120 mA
- **LoRa:** 30-150 mA (varies with spreading factor)
- **Bluetooth HC-05:** 40-80 mA
- **Bluetooth HM-10:** 10-20 mA

### Typical Range (Line-of-Sight)
- **ZigBee:** 1-2 km
- **LoRa:** 5-15 km (configurable)
- **Bluetooth:** 10-100 m

---

## Compatibility with Existing Features

✅ **All features work with all protocols:**
- GPS waypoint navigation
- Servo-based obstacle avoidance
- Intelligent path scanning
- Manual mode with operator alerts
- Fault tolerance (GPS/compass/servo failures)
- I2C communication with Raspberry Pi
- Serial debug output

✅ **No code changes needed** to obstacle_avoidance.cpp, navigation.cpp, gps_handler.h, compass_handler.h, motor_control.h, or obstacle_avoidance.h

✅ **Backward compatible** - old zigbee_* function names still work, mapped to wireless_* implementations

---

## Support Matrix

| Feature | ZigBee | LoRa | Bluetooth |
|---------|--------|------|-----------|
| Manual motor control | ✅ | ✅ | ✅ |
| Mode switching (AUTO/MANUAL) | ✅ | ✅ | ✅ |
| Real-time obstacle alerts | ✅ | ✅ | ✅ |
| GPS/Compass telemetry | ✅ | ✅ | ✅ |
| Waypoint storage (via I2C) | ✅ | ✅ | ✅ |
| Navigation resume | ✅ | ✅ | ✅ |
| Multiple remotes | ✅ | ⚠️ (broadcast) | ✅ (per connection) |
| Encryption | ⚠️ (API mode only) | ❌ | ⚠️ (Bluetooth auth only) |

---

## Next Steps

1. **Choose protocol** (ZigBee recommended for beginners)
2. **Purchase modules** (check links in datasheets section)
3. **Edit globals.h** to select protocol
4. **Use robot_navigation_wireless.ino** instead of old version
5. **Connect hardware** following wiring diagram
6. **Test with Serial Monitor** first
7. **Build remote controller** (Arduino Uno example provided)
8. **Deploy and enjoy!**

Your robot is now equipped for any remote operation scenario!
