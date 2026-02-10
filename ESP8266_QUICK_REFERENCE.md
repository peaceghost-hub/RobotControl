# ESP8266 Integration - Quick Summary

## What Changed

### **Removed**
- ❌ Arduino UNO with CC1101 (arduino_uno/cc1101_remote/)
- ❌ Message-based protocol (WIRELESS_CMD_MOTOR_FORWARD, etc.)
- ❌ UNO handshake logic

### **Added**
- ✅ ESP8266 (NodeMCU) remote control (esp8266_remote/cc1101_remote.ino)
- ✅ ADS1115 analog joystick interface
- ✅ Binary packet format (6 bytes: throttle, steer, flags, crc)
- ✅ Arcade drive mixing on Mega
- ✅ Auto-calibration at ESP8266 startup

---

## File Changes

### **New Files**
```
esp8266_remote/
└── cc1101_remote.ino          (181 lines) — Complete transmitter firmware
```

### **Modified Files**
```
arduino_mega/robot_navigation/
├── cc1101_driver.cpp          — Added 0xFF marker for raw packets
├── motor_control.h            — Added arcadeDrive() method
├── motor_control.cpp          — Implemented arcade mixing + setMotor() helper
└── robot_navigation.ino       — Updated handleWireless() for binary packets
```

---

## Key Code Changes

### **ESP8266 Transmit Loop**
```cpp
// Read joystick (ADS1115)
int32_t xRaw = readAveragedAds(0, 4);
int32_t yRaw = readAveragedAds(1, 4);

// Map to signed ±255
int16_t throttle = mapAdsToSigned255(yRaw, yCenter);
int16_t steer    = mapAdsToSigned255(xRaw, xCenter);

// Apply reverse toggle
if (reverseToggle) throttle = -throttle;

// Assemble packet
pkt.throttle = throttle;
pkt.steer    = steer;
pkt.flags    = (reverseToggle ? 0x01 : 0x00);
pkt.crc      = computeCrc(pkt);

// Send (blind, no library call)
ELECHOUSE_cc1101.SpiStrobe(0x36);
ELECHOUSE_cc1101.SpiWriteReg(0x3F, sizeof(pkt));
ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, (uint8_t*)&pkt, sizeof(pkt));
ELECHOUSE_cc1101.SpiStrobe(0x35);
delay(30);
ELECHOUSE_cc1101.SpiStrobe(0x36);
ELECHOUSE_cc1101.SpiStrobe(0x3B);
```

### **Mega Receive Loop**
```cpp
// In handleWireless()
if (msg.type == 0xFF && msg.length == 6) {
  // Raw binary packet from ESP8266
  int16_t throttle = ((int16_t)msg.data[0] << 8) | msg.data[1];
  int16_t steer    = ((int16_t)msg.data[2] << 8) | msg.data[3];
  
  // Apply arcade drive
  motors.arcadeDrive(throttle, steer);
}
```

### **Arcade Drive Mixing**
```cpp
void MotorControl::arcadeDrive(int throttle, int steer) {
  int left  = throttle + steer;
  int right = throttle - steer;
  
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  
  setMotor(left,  IN1, IN2, ENA);
  setMotor(right, IN3, IN4, ENB);
}
```

---

## CC1101 Configuration (Identical on Both Sides)

| Parameter | Value |
|-----------|-------|
| Frequency | 433.00 MHz |
| Modulation | 2-FSK |
| Data Rate | 9.6 kBaud |
| RX BW | 325 kHz |
| Deviation | 47.60 kHz |
| PA Power | +10 dBm |
| Sync Mode | 2 (16-bit) |
| Sync Word | 0xD3, 0x91 |
| CRC | Enabled |
| Pkt Format | 0 (Normal) |
| Length Config | 1 (Variable) |

---

## Wiring Summary

### **ESP8266 ↔ ADS1115**
| ESP8266 | ADS1115 |
|---------|---------|
| D3      | SDA     |
| D4      | SCL     |
| 3.3V    | VDD     |
| GND     | GND     |

**Joystick Channels:**
- A0: Pitch (throttle)
- A1: Roll (steer)

### **ESP8266 ↔ CC1101**
| ESP8266 | CC1101 |
|---------|--------|
| D5      | SCK    |
| D6      | MISO   |
| D7      | MOSI   |
| D2      | CSN    |
| D1      | GDO0   |
| 3.3V    | VCC    |
| GND     | GND    |

**Note:** 100µF capacitor near CC1101 VCC/GND

### **Joystick Button**
- ESP8266 D0 → Button (pulls to GND when pressed)
- Toggles reverse mode

---

## Testing Steps

### **1. Serial Verification (ESP8266)**
```
Expected output:
- "DO NOT TOUCH JOYSTICK" (startup)
- "Calibrated Centers -> X: 16384 Y: 16384" (or similar)
- "Thr: 0 Str: 0 RawX: 16384 RawY: 16384" (idle)
- Values change when joystick moves
```

### **2. Radio Check (Mega)**
```
Expected Mega serial output:
- CC1101 initialized
- "# ESP8266 arcade: throttle=XX steer=XX" (when joystick moves)
- Motors respond to values
```

### **3. Motor Test**
```
Joystick Action → Expected Motor Response
- Forward push  → Both motors forward
- Backward pull → Both motors backward
- Left push     → Left slower (or backward), right faster
- Right push    → Right slower (or backward), left faster
- Button press  → Reverse toggle (forward input = backward motion)
```

### **4. Range Test**
```
Test distance:
- ~0.5m: Must work perfectly
- ~2-3m: Should work reliably
- ~5m:   May work with good antenna/line-of-sight
- 10m+:  Likely loss unless antenna upgraded
```

---

## Packet Format Details

### **6-Byte Binary Packet**
```
Byte 0:  Throttle High (int16_t bit 8-15)
Byte 1:  Throttle Low  (int16_t bit 0-7)
Byte 2:  Steer High    (int16_t bit 8-15)
Byte 3:  Steer Low     (int16_t bit 0-7)
Byte 4:  Flags         (bit 0 = reverse toggle)
Byte 5:  CRC           (sum of bytes 0-4 mod 256)
```

### **Value Ranges**
- **Throttle:** -255 (full backward) to +255 (full forward)
  - 0 = stopped
  - -128 to 0 = backward
  - 0 to +128 = forward
- **Steer:** -255 (full left) to +255 (full right)
  - 0 = straight
  - -128 to 0 = left turn
  - 0 to +128 = right turn

### **Motor Output (after arcade mixing)**
```
Left Motor  = Throttle + Steer
Right Motor = Throttle - Steer

Examples:
- Throttle=100, Steer=0   → Left=100,  Right=100  (straight forward)
- Throttle=100, Steer=50  → Left=150,  Right=50   (forward+turn right)
- Throttle=0,   Steer=100 → Left=100,  Right=-100 (turn right in place)
- Throttle=-100, Steer=0  → Left=-100, Right=-100 (straight backward)
```

---

## Known Issues & Fixes

### **Soft WDT Reset on ESP8266**
- **Cause:** Old code called `SendData()` which blocks
- **Fix:** Manual blind send with strobes + `delay(30)`
- **Status:** ✅ Fixed

### **Joystick Drifting**
- **Cause:** No calibration; stale ADS1115 centers
- **Fix:** Auto-calibration at startup (30 samples)
- **Status:** ✅ Fixed

### **Motor Deadzone Buzz**
- **Cause:** Very small PWM values cause noise
- **Fix:** Deadzone in `setMotor()` (ignore <15 PWM)
- **Status:** ✅ Fixed

### **Handshake Timeout**
- **Cause:** Mega waits for `wirelessHandshakeComplete`
- **Fix:** Set flag on first 0xFF packet
- **Status:** ✅ Handled in updated `handleWireless()`

---

## Next Steps

1. **Flash ESP8266**
   - Install ELECHOUSE CC1101 library
   - Install Adafruit ADS1X15 library
   - Select Board: Generic ESP8266 Module / NodeMCU 1.0
   - Baud: 115200
   - Upload cc1101_remote.ino

2. **Flash Mega**
   - Already has all modified files
   - Compile and upload robot_navigation.ino
   - Verify I2C is enabled (DISABLE_I2C_FOR_SPI_TEST = 0)

3. **Test**
   - Open Serial Monitor on both (115200 baud)
   - Follow testing steps above
   - Adjust ranges/tuning as needed

---

## Support Files
- `ESP8266_INTEGRATION_GUIDE.md` — Detailed integration guide
- `esp8266_remote/cc1101_remote.ino` — Complete firmware
- Modified Mega files — See above for locations

**Status:** ✅ Integration complete, ready for testing
