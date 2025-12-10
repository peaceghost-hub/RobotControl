# KY-032 Wiring Quick Start

## Your Pin Assignments

```
Arduino Mega Pin 2     ← KY-032 DO (Digital Output)
Arduino Mega Pin A0    ← KY-032 AO (Analog Output)
Arduino Mega +5V       ← KY-032 VCC
Arduino Mega GND       ← KY-032 GND
```

## KY-032 Module Pinout

Looking at the module from the front:

```
┌─────────────────────┐
│  1    2    3    4   │
│ VCC  GND   DO   AO  │
└─────────────────────┘
  ↓    ↓    ↓    ↓
 +5V  GND  Pin2  A0
```

## Wiring Diagram

```
KY-032 Module:
    
    VCC (Pin 1) ──────→ Arduino +5V (Power)
    
    GND (Pin 2) ──────→ Arduino GND (Ground)
    
    DO  (Pin 3) ──────→ Arduino Pin 2 (Digital Input)
                       └─ HIGH when obstacle detected
                       └─ Can use with digitalRead()
    
    AO  (Pin 4) ──────→ Arduino A0 (Analog Input)
                       └─ 0-1023 ADC value
                       └─ Higher value = closer obstacle
                       └─ Use with analogRead()
```

## Power Considerations

- **VCC**: 5V (Arduino provides on power pin)
- **GND**: Common ground (must be connected)
- **Current**: ~20mA typical
- **Logic Levels**: 5V compatible

## Connection Verification Checklist

After wiring:

```
☐ VCC wire connected to Arduino +5V (red wire typically)
☐ GND wire connected to Arduino GND (black wire typically)
☐ DO wire connected to Arduino Pin 2
☐ AO wire connected to Arduino A0
☐ No loose connections
☐ Wires not frayed or touching other pins
☐ Module oriented correctly (pins on bottom)
```

## Testing Connection

Before running full code:

1. **Power test**:
   - Connect VCC and GND only
   - Module should turn on (red indicator LED if present)

2. **Pin test** (in Arduino sketch):
   ```cpp
   void setup() {
     Serial.begin(115200);
     pinMode(2, INPUT);
   }
   
   void loop() {
     int doValue = digitalRead(2);
     int aoValue = analogRead(A0);
     Serial.print("DO: ");
     Serial.print(doValue);
     Serial.print("  AO: ");
     Serial.println(aoValue);
     delay(100);
   }
   ```

3. **Obstacle test**:
   - Run the sketch above
   - Place your hand in front of KY-032
   - DO should go HIGH (1)
   - AO value should increase (0-1023)

## Troubleshooting Wiring

| Problem | Solution |
|---------|----------|
| No response from sensor | Check VCC/GND connections |
| DO always HIGH | Check if wired correctly (should be LOW at rest) |
| AO always 0 | Verify A0 pin connection |
| Random values | May need filtering or check for noise |
| Sensor not detecting objects | Check sensitivity pot on module, point at obstacle |

## Using in robot_navigation_wireless.ino

Once wired correctly, upload `robot_navigation_wireless.ino` and the system will automatically:

1. Initialize KY-032 on Pin 2 and A0
2. Read sensor values every 200ms
3. Combine with HC-SR04 ultrasonic for dual-sensor detection
4. Send alerts via wireless module (ZigBee/LoRa/Bluetooth)
5. Provide obstacle avoidance

## Next Steps

1. ✓ Wire KY-032 according to diagram above
2. Run the connection test code to verify wiring
3. Upload `robot_navigation_wireless.ino` to Arduino Mega
4. Open Serial Monitor (115200 baud) to see sensor readings
5. Place obstacles and verify detection works

---

Your dual-sensor obstacle detection system is ready to deploy! 🚀

