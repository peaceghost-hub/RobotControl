# Arduino UNO ZigBee Remote

This subfolder contains the handheld ZigBee controller sketch (`zigbee_remote/zigbee_remote.ino`). It sends joystick commands and receives GPS updates from the robot.

## Hardware Summary
- Arduino UNO (5V)
- ZigBee/XBee module (3.3V) + level shifter or shield
- 2-axis joystick module (typically 10k pots)
- Optional push button (using joystick SW pin) to force STOP

## Pin Mapping
| Function        | UNO Pin | Notes |
|-----------------|---------|-------|
| ZigBee RX (UNO reads) | 2 | SoftwareSerial RX |
| ZigBee TX (UNO writes) | 3 | SoftwareSerial TX (level shift to 3.3V) |
| Joystick X      | A0      | Analog 0 |
| Joystick Y      | A1      | Analog 1 |
| Joystick Button | D4      | INPUT_PULLUP (active LOW) |
| Status LED      | 13      | Built-in |

## Frame Protocol (Legacy CSV)
Outgoing:
- `HELLO,<deviceId>` every ~5s
- `JOYSTICK,<direction>,<speed>` throttled (~120ms) on change

Incoming (from robot):
- `GPS,<lat>,<lon>,<speed>,<heading>,<sats>,<timestamp>` echoed to Serial
- `MODE,...` or `HELLO,...` also echoed

Directions emitted: `forward`, `reverse`, `left`, `right`, `stop`
Speed range: 0-255

## Build & Upload
```bash
arduino-cli compile --fqbn arduino:avr:uno arduino_uno/zigbee_remote
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino_uno/zigbee_remote
```
Adjust port as needed (use `arduino-cli board list`).

## Adjustments
- Change `ZIGBEE_BAUD` in sketch if your module uses different baud.
- Tune `JOY_DEADBAND` for your joystick neutrality.
- Increase `SEND_INTERVAL_MS` to reduce traffic.

## Integration Notes
The dashboard's `zigbee_bridge.py` already parses `JOYSTICK` frames, extracting direction and speed. GPS frames forwarded by the robot update backup location.

## Next Enhancements (Optional)
- Add checksum to frames.
- Introduce a STARTUP frame to negotiate baud.
- Map diagonal joystick movement to combined commands if robot firmware supports it.
