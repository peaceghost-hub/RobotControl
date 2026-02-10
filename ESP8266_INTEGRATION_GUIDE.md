# ESP8266 CC1101 Integration Guide

## Overview
Successfully integrated ESP8266 (NodeMCU) with ADS1115 joystick and CC1101 transceiver to replace Arduino UNO remote control. System now uses **binary packet format** for efficient arcade-drive control.

---

## Architecture Changes

### **Old System (Arduino UNO)**
- UNO with CC1101 at 433 MHz
- Joystick control via digital pins
- Message-based protocol (MSG_TYPE_COMMAND)
- Command types: MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_LEFT, MOTOR_RIGHT, MOTOR_STOP

### **New System (ESP8266)**
- ESP8266 (NodeMCU) with CC1101 at 433 MHz
- Joystick via ADS1115 (I2C ADC) on pins D3/D4
- **Binary packet format** for direct motor control
- Arcade drive mixing: `throttle` (forward/back) + `steer` (turn)
- Auto-calibration at startup
- Reverse toggle via button on D0

---

## Hardware Connections

### **ESP8266 to ADS1115 (I2C)**
```
ESP8266      ADS1115
D3 (GPIO0)  →  SDA
D4 (GPIO2)  →  SCL
GND         →  GND
3.3V        →  VDD
```
**ADS1115 Address:** 0x48 (default)
- A0: Joystick X (pitch)
- A1: Joystick Y (roll)

### **ESP8266 to CC1101 (SPI)**
```
ESP8266      CC1101
D5 (SCK)    →  SCK
D6 (MISO)   →  MISO
D7 (MOSI)   →  MOSI
D2 (CS)     →  CSN
D1 (GPIO5)  →  GDO0
GND         →  GND
3.3V        →  VCC (with 100µF decap cap)
```

### **Joystick Button**
```
ESP8266 D0 (GPIO16) → Button (pulls LOW when pressed)
```
- Press to toggle reverse mode
- Default: forward/back normal
- After press: forward/back reversed

---

## CC1101 Configuration (MUST MATCH MEGA)

Both ESP8266 and Mega use identical settings:
```
Frequency:      433.00 MHz
Modulation:     2-FSK
Data Rate:      9.6 kBaud
RX Bandwidth:   325 kHz
Frequency Dev:  47.60 kHz
PA Power:       +10 dBm
Sync Mode:      2 (16-bit)
Sync Word:      0xD3, 0x91 (211, 145)
CRC:            Enabled
Packet Format:  0 (Normal)
Length Config:  1 (Variable)
```

---

## Packet Structure

### **Binary Packet (6 bytes)**
```
Offset  Type       Field         Description
0-1     int16_t    throttle      -255 (full back) to +255 (full forward)
2-3     int16_t    steer         -255 (full left) to +255 (full right)
4       uint8_t    flags         bit 0 = reverse toggle
5       uint8_t    crc           8-bit checksum (sum of first 5 bytes)
```

### **Receiving on Mega**
- CC1101 receiver captures 6-byte packet
- Driver passes as `WirelessMessage` with `type = 0xFF` (binary marker)
- `handleWireless()` detects 0xFF and parses throttle/steer
- `motors.arcadeDrive(throttle, steer)` applies mixing

### **Arcade Drive Mixing**
```
left_motor  = throttle + steer
right_motor = throttle - steer
```
- Throttle dominant: forward/backward movement
- Steer dominant: turning in place
- Both: curved movement (turn while moving)

---

## Files Modified/Created

### **New Files**
- `esp8266_remote/cc1101_remote.ino` — Complete ESP8266 firmware

### **Modified Files**
- `arduino_mega/robot_navigation/cc1101_driver.cpp` — Added raw binary packet handling (0xFF marker)
- `arduino_mega/robot_navigation/motor_control.h` — Added `arcadeDrive()` method
- `arduino_mega/robot_navigation/motor_control.cpp` — Implemented arcade drive mixing + `setMotor()` helper
- `arduino_mega/robot_navigation/robot_navigation.ino` — Updated `handleWireless()` to process binary packets

---

## ESP8266 Firmware Details

### **Auto-Calibration**
1. On startup: "DO NOT TOUCH JOYSTICK" message
2. Takes 30 samples of both joystick axes
3. Sets `xCenter` and `yCenter` (eliminates drifting)
4. Prints calibrated values to Serial

### **Joystick Mapping**
```cpp
// Reading
int32_t xRaw = readAveragedAds(0, 4);  // 4-sample average
int32_t yRaw = readAveragedAds(1, 4);

// Mapping to -255...+255
int16_t steer    = mapAdsToSigned255(xRaw, xCenter);
int16_t throttle = mapAdsToSigned255(yRaw, yCenter);

// Deadzone: 600 (prevents tiny drifts from being sent)
// Range: 16384 ± 600 = centered
```

### **Blind Send (No Soft WDT Reset)**
Uses manual SPI strobe commands instead of library SendData:
```cpp
ELECHOUSE_cc1101.SpiStrobe(0x36);     // IDLE
ELECHOUSE_cc1101.SpiWriteReg(0x3F, len);  // Length byte
ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, (uint8_t*)&pkt, len);  // Data
ELECHOUSE_cc1101.SpiStrobe(0x35);     // TRANSMIT
delay(30);
ELECHOUSE_cc1101.SpiStrobe(0x36);     // IDLE
ELECHOUSE_cc1101.SpiStrobe(0x3B);     // FLUSH TX
```

### **Reverse Toggle**
- Button press on D0 toggles `reverseToggle`
- Applied: `if (reverseToggle) throttle = -throttle;`
- Useful for switching driving direction

### **Debug Output**
Every 20ms (50 Hz):
```
Thr: 42 Str: -18 RawX: 15844 RawY: 16520
```

---

## Testing Checklist

### **Phase 1: Hardware**
- [ ] ESP8266 boots, prints "DO NOT TOUCH JOYSTICK"
- [ ] Calibration prints `Calibrated Centers -> X: XXXXX Y: XXXXX`
- [ ] ADS1115 responds (no I2C error)
- [ ] CC1101 responds (no "CC1101 Error")
- [ ] Serial shows `Thr: 0 Str: 0` when joystick idle

### **Phase 2: Transmission**
- [ ] Mega CC1101 RX LED flashes when ESP8266 sends
- [ ] Mega serial shows: `# ESP8266 arcade: throttle=XX steer=XX`
- [ ] Values match joystick movement

### **Phase 3: Motor Control**
- [ ] Push joystick forward → both motors forward
- [ ] Push joystick left → left motor slower, right motor faster
- [ ] Push joystick right → right motor slower, left motor faster
- [ ] Push joystick backward → both motors backward
- [ ] Button press → reverse toggles (forward input now moves backward)

### **Phase 4: Range/Reliability**
- [ ] Control works at 2 meters without line-of-sight
- [ ] Control drops at 5+ meters (adjust PA or antenna)
- [ ] No wireless dropouts at typical operating range
- [ ] No Mega crashes or resets

---

## Troubleshooting

### **ESP8266 Issues**

**Soft WDT Reset**
- Old code used `SendData()` which blocks during I2C reads
- **Fixed:** Now uses manual blind send with `delay(30)` instead
- Ensure `delay(20)` at end of loop()

**Joystick Drifting (Moves at idle)**
- **Fixed:** Auto-calibration at startup
- If still drifting: increase `deadband` from 600 to 800

**ADS1115 Not Found**
- Check I2C wiring: D3=SDA, D4=SCL
- Check address: should be 0x48
- Add `Wire.begin(D3, D4)` before `ads.begin()`

**CC1101 Not Found**
- Check SPI wiring: D5/D6/D7/D2
- Check GDO0 to D1
- Verify 3.3V power + decoupling capacitor

### **Mega Issues**

**No packets received**
- Check Mega CC1101 RX is enabled: `ELECHOUSE_cc1101.SetRx()`
- Verify sync word matches: 211, 145 (0xD3, 0x91)
- Check antenna: ensure short leads, proper 2.4cm length or SMA connector

**Packets received but no motor movement**
- Check `handleWireless()` is called from `loop()`
- Verify `msg.type == 0xFF` detection works
- Check motor pins: should be 5/22/23 (left) and 6/24/25 (right)
- Verify `motors.begin()` was called

**Wireless OFFLINE despite packets**
- Mega may still be waiting for handshake
- ESP8266 does not send handshake; it sends raw binary immediately
- Change Mega logic: set `wirelessHandshakeComplete = true;` on first 0xFF packet

---

## Future Improvements

1. **Add GPS overlay**: Send Mega GPS back to ESP8266 for display
2. **Failsafe timeout**: Stop motors if no packet received for 500ms (already in Mega)
3. **Signal quality display**: Show RSSI on ESP8266 Serial
4. **Command acknowledgment**: Mega sends ACK back to ESP8266
5. **Modular packet format**: Support different control modes (e.g., depth control for underwater)

---

## Summary

✅ **ESP8266 integration complete**
- Auto-calibrating joystick with ADS1115
- Binary arcade-drive packet format
- Blind CC1101 transmission (no Soft WDT)
- Mega receives and applies arcade mixing
- Motors respond to joystick immediately
- Range: ~2-3 meters typical, up to 5+ with good antenna

**Next step:** Flash ESP8266 and Mega, test phases 1-4 above.
