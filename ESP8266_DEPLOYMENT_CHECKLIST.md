# ESP8266 Integration - Deployment Checklist

**Date:** 10 February 2026  
**Status:** ✅ Integration Complete  
**Previous System:** Arduino UNO CC1101  
**New System:** ESP8266 (NodeMCU) + ADS1115 + CC1101

---

## 📋 Pre-Deployment

### Hardware Assembly
- [ ] ESP8266 NodeMCU board ready
- [ ] ADS1115 ADC module connected to D3/D4 (I2C)
- [ ] CC1101 module connected to D5/D6/D7/D2/D1 (SPI + GDO0)
- [ ] Joystick connected to ADS1115 (A0=X, A1=Y)
- [ ] Reverse button connected to D0 (GPIO16)
- [ ] 100µF capacitor on CC1101 VCC/GND
- [ ] 3.3V supply stable (multimeter check)
- [ ] All wires short and secure (especially SPI)

### Software Prerequisites
- [ ] Arduino IDE installed
- [ ] ESP8266 board support installed (Generic ESP8266 Module / NodeMCU 1.0)
- [ ] **ELECHOUSE_CC1101_SRC_DRV** library installed
- [ ] **Adafruit_ADS1X15** library installed
- [ ] Mega code uploaded (robot_navigation.ino with CC1101 modifications)

### Configuration Check
- [ ] CC1101 frequency: **433.00 MHz** (both ESP8266 and Mega)
- [ ] Sync word: **0xD3, 0x91** (211, 145) matching
- [ ] Data rate: **9.6 kBaud** matching
- [ ] CRC enabled on both
- [ ] Packet length config: **variable (1)** on both

---

## 🚀 Phase 1: ESP8266 Flash & Boot

### Upload Firmware
```
1. Open Arduino IDE
2. File → Open: esp8266_remote/cc1101_remote.ino
3. Tools → Board → Generic ESP8266 Module
4. Tools → Upload Speed → 115200
5. Tools → Port → /dev/ttyUSB0 (or COM port)
6. Sketch → Upload
7. Wait for "Leaving... Hard resetting via RTS pin"
```

### Verify Boot Output
```
Serial Monitor @ 115200 baud should show:
---
ESP8266 Joystick TX - Auto Calibrating...
DO NOT TOUCH JOYSTICK
(wait 500ms)
Calibrated Centers -> X: 16384 Y: 16384
--------------------------------
Thr: 0 Str: 0 RawX: 16384 RawY: 16384
Thr: 0 Str: 0 RawX: 16384 RawY: 16384
...
```

**Expected Issues & Fixes:**

| Issue | Cause | Fix |
|-------|-------|-----|
| "ADS1115 not found!" | I2C wiring or address wrong | Check D3/D4, verify 0x48 address |
| "CC1101 Error" | SPI wiring or power | Check D5/D6/D7/D2, add 100µF cap |
| Soft WDT Reset | Code bug (old blind send) | Should not happen; update library |
| Throttle/Steer not zero when idle | Joystick drift | Recalibrate: press RESET on ESP8266 |

---

## 📊 Phase 2: Joystick Verification

### Test Joystick Movement
Open Serial Monitor on ESP8266 @ 115200, watch for values:

```
Test Input          Expected Output (approx)
Joystick idle       Thr: 0 Str: 0
Push forward        Thr: 100+ Str: 0
Push backward       Thr: -100 Str: 0
Push left           Thr: 0 Str: -100
Push right          Thr: 0 Str: +100
Diagonal fwd-left   Thr: 100 Str: -50
Diagonal bwd-right  Thr: -100 Str: +50
Press button once   (next forward becomes backward)
```

### Validation Criteria
- [ ] Thr and Str are 0 when joystick idle (±5 tolerance)
- [ ] Thr increases/decreases smoothly with Y axis
- [ ] Str increases/decreases smoothly with X axis
- [ ] Button press toggles reverse (watch subsequent forward inputs)
- [ ] No spikes or wild values
- [ ] Output update rate ~50 Hz (every 20ms)

---

## 🔗 Phase 3: Wireless Link Check

### Mega Verification
```
Open Serial Monitor on Mega @ 115200, watch for:
# ========== ARDUINO MEGA NAVIGATION CONTROLLER ==========
...
# Wireless (CC1101): deferred init in loop
...
# ESP8266 arcade: throttle=XX steer=XX
```

### Manual Trigger Test
1. ESP8266 running (Serial shows Thr/Str values)
2. Mega powered on and Serial Monitor open
3. Move joystick on ESP8266
4. **Within 5 seconds**, Mega serial should print:
   ```
   # ESP8266 arcade: throttle=42 steer=18
   # ESP8266 arcade: throttle=45 steer=20
   # ESP8266 arcade: throttle=0 steer=0
   ```

### Validation Criteria
- [ ] Mega receives packets (serial output confirms)
- [ ] Received throttle matches ESP8266 output
- [ ] Received steer matches ESP8266 output
- [ ] No CRC errors or dropped packets
- [ ] Connection established within 5 seconds

**Troubleshooting:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| No "ESP8266 arcade" messages | Mega CC1101 RX not enabled | Ensure `handleWireless()` called in loop |
| Messages say wrong type (not 0xFF) | Binary packet parsing issue | Check cc1101_driver.cpp receive() |
| Messages corrupt | Sync word mismatch | Verify both use 0xD3, 0x91 |
| Packets every 2-3 seconds | Mega not calling handleWireless() frequently | Check loop() timing |

---

## 🎮 Phase 4: Motor Control Test

### Prerequisite
- [ ] Robot on non-conductive surface (so wheels can spin freely)
- [ ] Motors tested separately (forward, backward, both)
- [ ] No obstacles around robot
- [ ] Mega batteries fresh

### Run Motor Test
```
Joystick Move       Expected Motor Response
Push forward        Both wheels forward at increasing speed
Release             Both wheels stop
Push backward       Both wheels backward at increasing speed
Release             Both wheels stop
Push left           Left wheel slower/stop, right wheel forward (turn left in place)
Push right          Right wheel slower/stop, left wheel forward (turn right in place)
Diagonal fwd+left   Both forward but left slower (curved forward-left)
```

### Validation Criteria
- [ ] Forward push (100 throttle) → both motors ~100 PWM forward
- [ ] Backward pull (-100 throttle) → both motors ~100 PWM backward
- [ ] Left push (0 throttle, -100 steer) → left -100, right +100 (spin left)
- [ ] Right push (0 throttle, +100 steer) → left +100, right -100 (spin right)
- [ ] Diagonal motion smooth and curved
- [ ] Motor response immediate (< 50ms)
- [ ] No buzzing or stuttering

### Calculate Motor Outputs
If you see `throttle=60 steer=30`:
```
left  = 60 + 30  = 90   (faster)
right = 60 - 30  = 30   (slower)
→ curves forward-right
```

**Troubleshooting:**

| Issue | Cause | Fix |
|-------|-------|-----|
| Motors don't move | Motor power disconnected | Check L298N GND/VCC |
| Motors move backward on forward input | IN1/IN2 reversed | Swap motor wires |
| One motor moves, other doesn't | Motor fault | Test motor directly |
| Motors move but with lag | Mega not receiving packets | Check wireless link phase 3 |
| Motors buzz but don't move | PWM too low (below 15) | Normal; PWM increases with joystick range |

---

## 📡 Phase 5: Range & Reliability Test

### Test Environment Setup
- [ ] Open area, line-of-sight between ESP8266 and Mega
- [ ] No RF interference (WiFi router away)
- [ ] Measure distance with tape measure
- [ ] Mega serial monitor running

### Distance Tests
```
Distance    Expected                  Result
0.5m        Reliable 100% packets     [ ] Pass [ ] Fail
1m          Reliable 100% packets     [ ] Pass [ ] Fail
2m          Reliable 99% packets      [ ] Pass [ ] Fail
3m          Reliable 95% packets      [ ] Pass [ ] Fail
5m          May drop some packets     [ ] Pass [ ] Fail
10m         Likely disconnects        [ ] Pass [ ] Fail
```

**Success Metric:** At least **2-3 meters reliable operation** with line-of-sight.

### Packet Drop Measurement
From Mega serial, count packets in 10 seconds:
```
Theoretical: 50 Hz × 10s = 500 packets
Observed: _____ packets
Drop rate: (500 - observed) / 500 × 100% = _____%
Target: < 2% drop at 2-3m
```

---

## 🔧 Phase 6: Fine-Tuning

### Joystick Sensitivity
If joystick is too sensitive:
- Increase `deadband` in ESP8266 code from 600 to 800
- Re-flash and test

If joystick is too slow to respond:
- Decrease `deadband` from 600 to 400
- Reduce `samples` in `readAveragedAds()` from 8 to 4

### Motor Speed Tuning
If motors move too fast/slow on same joystick input:
- Adjust joystick mapping in `mapAdsToSigned255()` span calculation
- Or adjust L298N PWM values in Mega `motor_control.cpp`

### Antenna Improvement
If range < 2m:
- Try different antenna orientation (45°, vertical, horizontal)
- Try SMA connector antenna (2.4 cm monopole)
- Move Mega away from metal/water (RF absorbers)
- Add ferrite toroid around CC1101 power leads

---

## 📝 Phase 7: Documentation & Finalization

### Code Comments
- [ ] Add date/time to both sketches: `// Updated: 10 FEB 2026`
- [ ] Document any deviations from standard CC1101 config
- [ ] Note joystick calibration values (xCenter, yCenter)

### Backup Files
```bash
cp esp8266_remote/cc1101_remote.ino backup/esp8266_cc1101_2026-02-10.ino.bak
cp arduino_mega/robot_navigation/robot_navigation.ino backup/robot_nav_2026-02-10.ino.bak
```

### Version Control
- [ ] Commit changes: `git add -A && git commit -m "ESP8266 integration: binary arcade-drive protocol"`
- [ ] Tag release: `git tag -a v1.2-esp8266 -m "Replaced UNO with ESP8266 + ADS1115"`

### Deploy Checklist
- [ ] All 7 phases passed
- [ ] Distance test: 2-3m minimum
- [ ] Motor response: < 50ms latency
- [ ] No packet drops > 2%
- [ ] No motor buzzing
- [ ] No serial errors or warnings

---

## 🚨 Emergency Procedures

### Robot Won't Stop
**Immediate:** Unplug Mega battery  
**Cause:** Wireless stuck command  
**Fix:** Check Mega `handleWireless()` timeout logic

### ESP8266 Won't Boot
**Immediate:** Press RESET button  
**Cause:** Power brown-out or corrupt upload  
**Fix:** Re-upload firmware, ensure stable 3.3V supply

### Motors Moving Backward
**Immediate:** Pull joystick in opposite direction  
**Cause:** Motor wires or IN pins reversed  
**Fix:** Swap motor wires or flip IN1/IN2 in code

### Wireless Dropping Out
**Immediate:** Move closer (< 1m)  
**Cause:** Antenna, RF interference, or low signal  
**Fix:** Improve antenna, move away from WiFi, check CC1101 power

---

## 📞 Support References

### Files Involved
- **ESP8266 Firmware:** `esp8266_remote/cc1101_remote.ino` (181 lines)
- **Mega Receiver:** `arduino_mega/robot_navigation/robot_navigation.ino` (modified)
- **Motor Driver:** `arduino_mega/robot_navigation/motor_control.cpp` (added arcade drive)
- **Wireless Driver:** `arduino_mega/robot_navigation/cc1101_driver.cpp` (modified for 0xFF packets)

### Documentation
- `ESP8266_INTEGRATION_GUIDE.md` — Detailed technical guide
- `ESP8266_QUICK_REFERENCE.md` — Command/config reference
- `ESP8266_DEPLOYMENT_CHECKLIST.md` — This document

### Libraries Required
- `ELECHOUSE_CC1101_SRC_DRV` (GitHub: elechouse/CC1101)
- `Adafruit_ADS1X15` (Arduino Library Manager)
- `Wire.h` (built-in I2C)
- `SPI.h` (built-in SPI)

---

## ✅ Sign-Off

- [ ] All 7 phases completed
- [ ] All validation criteria met
- [ ] Documentation complete
- [ ] Backup files created
- [ ] Version control committed

**Technician Name:** _________________  
**Date:** _________________  
**Time Spent:** _________ hours  
**Status:** ✅ **READY FOR DEPLOYMENT**

---

## 📌 Next Steps

1. **Deploy to field:** Transport robot to operational site
2. **Run acceptance test:** Verify 3m range in field conditions
3. **Train user:** Show how to move joystick, press button
4. **Schedule maintenance:** Check battery, CC1101 antenna weekly
5. **Monitor logs:** Save Mega serial output for first week

---

**End of Checklist — Integration Complete**
