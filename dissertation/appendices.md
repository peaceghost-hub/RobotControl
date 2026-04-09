# APPENDICES

---

## Appendix A: Major Source Files in the Final System

This appendix summarises the principal source files that define the final implemented robot. It replaces earlier appendix material that reproduced outdated code excerpts without reflecting the final architecture.

**Table A.1: Major Implementation Files**

| File | Role in Final System |
|---|---|
| `raspberry_pi/main.py` | Main Raspberry Pi controller; manages sensors, dashboard API, command handling, and overall orchestration |
| `raspberry_pi/navigation/nav_controller.py` | Pi-side heading, waypoint, return-home, and navigation state logic |
| `raspberry_pi/communication/i2c_comm.py` | Raspberry Pi to Arduino Mega I2C communication layer |
| `dashboard/app.py` | Flask backend for dashboard APIs, command queuing, and joystick bridge handling |
| `dashboard/joystick_bridge.py` | USB serial bridge that reads ESP8266 joystick frames and forwards them to the dashboard backend |
| `dashboard/static/js/main.js` | Main browser dashboard behaviour and control logic |
| `dashboard/static/js/map.js` | Leaflet/OpenStreetMap map integration used by the dashboard |
| `dashboard/ai_vision.py` | AI vision support module; no longer the normal obstacle-safety authority |
| `arduino_mega/robot_navigation/robot_navigation.ino` | Main Mega firmware; owns motor control states, CC1101 handling, and local reactive obstacle behaviour |
| `arduino_mega/robot_navigation/navigation.cpp` | Mega-side waypoint and local obstacle-avoid support logic |
| `arduino_mega/robot_navigation/obstacle_avoidance.cpp` | Servo-mounted ultrasonic scan logic |
| `esp8266_remote/cc1101_remote/cc1101_remote.ino` | ESP8266 handheld transmitter firmware using dual-joystick tank drive |

---

## Appendix B: Current Hardware Interconnection Summary

The final robot hardware is organised around three cooperating subsystems:

1. **Raspberry Pi subsystem**
   High-level control, dashboard communication, LTE access, storage, camera, and navigation reasoning.

2. **Arduino Mega subsystem**
   Motor interface, servo-mounted ultrasonic scanning, CC1101 receiver, and deterministic control ownership.

3. **ESP8266 handheld controller**
   Dual-joystick tank-drive transmitter used either directly over CC1101 or indirectly through the dashboard USB bridge.

**Table B.1: Core Hardware Roles**

| Hardware | Main Role |
|---|---|
| Raspberry Pi 3 Model B | High-level coordination, dashboard host, LTE, camera, data handling |
| Arduino Mega 2560 | Low-level motor control, local radio reception, servo scan, local obstacle reaction |
| SIM7600E | LTE connectivity and auxiliary Pi-side GPS functionality |
| NEO-6M | GPS source associated with the robot navigation stack |
| DHT11 | Ambient temperature and humidity |
| MQ-2 / MQ-135 / MQ-7 | Gas sensing |
| ADS1115 | Analogue acquisition for MQ sensors and handheld transmitter |
| HC-SR04 on servo | Local directional obstacle scanning |
| CC1101 | Local RF manual-control path |
| ESP8266 controller | Handheld transmitter for local or dashboard-mediated teleoperation |

**Table B.2: Core Mega Pins in the Final System**

| Pin(s) | Function |
|---|---|
| 5, 22, 23 | Left motor PWM and direction |
| 6, 24, 25 | Right motor PWM and direction |
| 7 | Ultrasonic scan servo |
| 10 | Buzzer |
| 18, 19 | GPS serial interface |
| 20, 21 | I2C to Raspberry Pi |
| 30, 31 | HC-SR04 trigger and echo |
| 50, 51, 52, 53 | SPI for CC1101 |
| 2, 3 | CC1101 GDO lines |

---

## Appendix C: Current Control-Mode Summary

The final robot intentionally supports more than one control mode, but each mode has a defined ownership model.

**Table C.1: Control Modes**

| Mode | Path | Owner |
|---|---|---|
| Dashboard manual arrows | Browser -> dashboard -> Pi -> Mega | Raspberry Pi through `STATE_I2C` |
| Dashboard joystick bridge | ESP8266 USB serial -> dashboard -> Pi -> Mega | Raspberry Pi through `STATE_I2C` |
| Direct handheld control | ESP8266 -> CC1101 -> Mega | Mega through `STATE_WIRELESS` |
| Waypoint assistance | Dashboard/Pi navigation -> Mega | Raspberry Pi for intent, Mega for local execution |

When the dashboard USB bridge is active, the ESP8266 transmitter enters a USB bridge mode so that the browser-driven path does not fight direct RF transmission from the same handheld controller.

---

## Appendix D: Current Obstacle-Handling Summary

The final obstacle-handling design is deliberately conservative.

**Table D.1: Mega Local Obstacle Rule**

| Zone | Meaning |
|---|---|
| 90 cm to 61 cm | Detect obstacle and begin scan/planning |
| 60 cm to 31 cm | Commit local avoidance if a clear side is identified |
| 30 cm and below | Immediate failed-avoid stop |

Additional operational notes:

- During waypoint assistance and dashboard-arrow driving, the Mega may perform local auto-avoid under the rule above.
- During raw joystick driving, the Mega does not force an immediate takeover while the operator is still actively commanding the wheels.
- In raw joystick mode, local auto-avoid acknowledgement occurs when both controls are released.
- After one acknowledged raw-manual auto-avoid attempt, the Mega stops and waits for operator control again rather than chaining repeated avoid attempts.
- The Raspberry Pi remains aware of obstacle state but no longer performs the old physical reverse-turn-pass obstacle manoeuvre itself.

---

## Appendix E: Gas Channels and Warning Presentation

The dashboard warning system is now tied to the same calibrated gas channels that are displayed to the operator.

**Table E.1: Gas Channels Presented in the Dashboard**

| Channel | Dashboard Interpretation |
|---|---|
| MQ-2 | Smoke-oriented concentration |
| MQ-135 | CO₂-oriented air-quality concentration |
| MQ-7 | Carbon monoxide concentration |

**Table E.2: Warning Bands**

| Channel | Elevated | Warning | Danger |
|---|---:|---:|---:|
| MQ-2 | 50 ppm | 500 ppm | 5000 ppm |
| MQ-135 | 600 ppm | 1000 ppm | 5000 ppm |
| MQ-7 | 9 ppm | 35 ppm | 200 ppm |

These bands are suitable for operator awareness and low-cost monitoring, but they should not be presented as certified industrial alarm thresholds without further calibration and validation.

---

*End of Appendices*
