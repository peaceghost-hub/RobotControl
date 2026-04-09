# CHAPTER 4: RESULTS

---

## 4.1 Introduction

This chapter presents the results of implementing and integrating the final version of the system. The emphasis of the updated chapter is on the robot as it now exists in hardware and software, rather than on earlier draft assumptions that no longer describe the platform correctly. The results are therefore reported in terms of implemented capability, verified subsystem behaviour, control-path integration, and the practical operating role of the finished robot.

---

## 4.2 Environmental Monitoring Results

### 4.2.1 Environmental Sensor Integration

The final robot successfully integrates the environmental sensor suite on the Raspberry Pi side using a DHT11 sensor together with MQ-2, MQ-135, and MQ-7 gas sensors digitised through an ADS1115 ADC. The important outcome of this integration is not merely that raw sensor values are sampled, but that the dashboard presents operator-facing readings in calibrated PPM form for the gas channels.

The updated implementation therefore improves on earlier low-cost robot patterns that stopped at raw ADC reporting. In the completed dashboard:

- temperature and humidity are displayed continuously;
- MQ-2 is presented as smoke-oriented concentration;
- MQ-135 is presented as a CO₂-oriented air-quality channel;
- MQ-7 is presented as carbon monoxide concentration;
- and warning banners are generated from the same calibrated PPM quantities shown on the sensor cards.

This alignment between displayed value and warning logic is itself a significant implementation result, because earlier mismatches between displayed PPM values and stale raw-threshold warnings were removed during development.

**Table 4.1: Implemented Gas Warning Bands**

| Sensor Channel | Elevated Band | Warning Band | Danger Band |
|---|---:|---:|---:|
| MQ-2 smoke | 50 ppm | 500 ppm | 5000 ppm |
| MQ-135 CO₂ | 600 ppm | 1000 ppm | 5000 ppm |
| MQ-7 CO | 9 ppm | 35 ppm | 200 ppm |

The result is a monitoring interface that is substantially easier to interpret during operation than one based only on raw counts or voltages. The dissertation nevertheless recognises that these values remain low-cost calibrated estimates rather than laboratory-certified measurements.

### 4.2.2 Position, Heading, and Monitoring Outputs

The positioning and heading subsystem is now presented to the operator in a clearer and more diagnostic form than before. The dashboard supports:

- GPS position monitoring on the map;
- heading display using corrected heading and magnetic heading views;
- waypoint and target visualisation;
- manual target locking;
- and return-home support.

The Raspberry Pi side remains responsible for heading computation, correction handling, and high-level coordinate logic, while the Mega remains responsible for low-level motor-side behaviour. This separation made it possible to improve heading interpretation without rewriting the Mega motor ownership logic.

### 4.2.3 Camera and Dashboard Monitoring

The dashboard successfully functions as the practical monitoring surface of the robot. The final interface provides:

- live MJPEG camera streaming;
- Leaflet/OpenStreetMap map visualisation;
- live environmental sensor cards and history;
- compass displays;
- manual control widgets;
- navigation controls;
- joystick bridge status;
- and AI Vision controls.

This result is important because it confirms that the robot is not merely a mobile sensor node: it is an operator-facing environmental monitoring platform whose behaviour can be observed and influenced in real time from a browser.

---

## 4.3 Control and Communication Results

### 4.3.1 Raspberry Pi to Arduino Mega Coordination

One of the clearest implementation results of the project is the direct coordination between the Raspberry Pi and Arduino Mega. In the completed system, the two processors are not simply co-present on the robot; they are linked by an explicit I2C control path with distinct responsibility boundaries:

- the Raspberry Pi manages dashboard communication, data handling, GPS/heading calculations, and high-level navigation intent;
- the Arduino Mega manages local motor execution, local wireless reception, and immediate obstacle reaction.

This direct Pi-to-Mega link is a defining characteristic of the implemented platform and an important distinction from the more loosely described processor relationships discussed in the literature.

### 4.3.2 Control Ownership Results

The Arduino Mega retains the three-state ownership model in the completed robot:

**Table 4.2: Mega Control Ownership States**

| State | Meaning | Typical Owner |
|---|---|---|
| `STATE_I2C` | Motion intent is coming from the Raspberry Pi | Dashboard manual arrows, dashboard USB joystick, waypoint assistance |
| `STATE_WIRELESS` | Motion intent is coming from the local RF handheld controller | ESP8266 + CC1101 direct control |
| `STATE_FAILSAFE` | Safe stop because valid control ownership has been lost | No owner / communication loss |

The practical result of this architecture is that the robot has a deterministic answer to the question "who owns the motors right now?" That clarity is essential for safe operation and for integrating multiple control paths without unpredictable overlap.

### 4.3.3 CC1101 and Dashboard Joystick Results

The local manual-control implementation now centres on CC1101 and the dashboard USB joystick bridge, not on a dual-radio backup design. Two distinct manual-control results were achieved:

1. **Direct local tank drive over CC1101**
   The ESP8266 handheld transmitter now operates in tank-drive form, dedicating one joystick axis to each wheel and transmitting explicit left-wheel and right-wheel speeds.

2. **Dashboard joystick bridge**
   The same handheld controller can be connected by USB to the dashboard machine. In this mode, the dashboard backend reads the transmitted wheel values from serial and forwards them through the Raspberry Pi to the Mega. A USB claim mode suppresses direct CC1101 radio transmission while the dashboard bridge is active, reducing control-path conflict.

**Table 4.3: Implemented Manual Control Paths**

| Control Path | Transport | Motion Style | Practical Role |
|---|---|---|---|
| Dashboard arrows | Browser -> dashboard backend -> Pi -> I2C -> Mega | Discrete directional commands | Simple supervised manual driving |
| Dashboard USB joystick | ESP8266 USB serial -> dashboard backend -> Pi -> I2C -> Mega | Tank drive / direct wheel speeds | Long-range browser-supervised teleoperation |
| Direct CC1101 joystick | ESP8266 -> CC1101 -> Mega | Tank drive / direct wheel speeds | Local short-range manual driving |

The result is a much more practical control architecture than a single short-range app-only interface. It supports both local intervention and browser-mediated teleoperation.

### 4.3.4 Networked Dashboard Access

The inclusion of the SIM7600E module and the browser dashboard produced a major operational result: the robot can now be treated as a remotely supervised platform rather than as a device whose operator must remain within short-range radio distance. In practical terms, this means:

- local radio remains useful for nearby operator intervention;
- but the main human-machine interface can be reached through normal IP networking;
- and LTE/4G connectivity extends practical supervision beyond local control range.

This shift in how the robot is actually used is one of the most important results of the project.

---

## 4.4 Navigation and Obstacle-Handling Results

### 4.4.1 Assisted Waypoint Navigation

The final implementation retains waypoint loading, bearing acquisition, heading correction, return-home functions, and manual target locking on the Raspberry Pi side. Autonomous navigation was therefore not merely proposed in design terms; it was implemented and tested as part of the completed system. However, the result is still best understood as an assisted navigation capability rather than an all-conditions autonomous navigation guarantee.

In the completed system:

- the Raspberry Pi computes bearing, heading error, target distance, and reacquisition logic;
- the Mega executes the motion intent and remains the last local authority for motor safety;
- and the dashboard allows the operator to supervise or interrupt navigation as required.

This means the project still demonstrates autonomous-style behaviours, but the result is best interpreted as supervised assisted navigation rather than fully trusted autonomy. The reason is practical rather than rhetorical: the robot's local obstacle awareness still depends primarily on one servo-mounted ultrasonic sensor, which is useful for low-cost reactive scanning but not sufficient to justify high-trust independent navigation in arbitrary outdoor scenes.

### 4.4.2 Servo-Mounted Ultrasonic Scanning

The obstacle sensor has been restored to a servo-mounted HC-SR04 configuration on the Mega, allowing the robot to scan left, centre, and right without physically rotating the chassis first. This is a meaningful functional result because it improves local directional awareness without requiring more expensive perception hardware.

The implemented thresholds are now:

**Table 4.4: Mega Local Obstacle Thresholds**

| Distance Band | Behaviour |
|---|---|
| 90 cm down to 61 cm | Obstacle detected and scan/planning begins |
| 60 cm down to 31 cm | Local avoidance may be committed if a clear side is identified |
| 30 cm and below | Failed avoidance; immediate stop |

The final logic therefore rejects earlier behaviour in which the robot could continue improvising too close to an obstacle. At the current stage of implementation, the platform intentionally stops rather than pretending confidence when the obstacle has already closed to the fail-stop distance.

### 4.4.3 Separation Between Autonomous and Raw Manual Obstacle Behaviour

An important result of the final system is that obstacle behaviour is no longer identical across all control modes.

For **waypoint assistance and dashboard-arrow driving**:

- the Mega performs local scan-based obstacle handling according to the `90 cm / 60 cm / 30 cm` rule;
- the Raspberry Pi remains aware of obstacle state but no longer performs the old reverse-turn-pass physical manoeuvre itself.

For **raw joystick driving** through either direct CC1101 or the dashboard USB joystick bridge:

- the Mega can begin scanning when an obstacle is detected;
- but it does not immediately take over and auto-avoid while the operator is still actively commanding the wheels;
- acknowledgement occurs only when both controls are released;
- and once acknowledged, the Mega performs one local avoid attempt only, then stops and waits for the operator again.

This is a significant functional refinement because it reduces unwanted takeover during manual teleoperation while still allowing local assistance when the operator explicitly yields control.

### 4.4.4 AI Role After the Obstacle-Handling Redesign

The updated system no longer treats AI Vision as the normal obstacle-avoidance authority. That responsibility now belongs locally to the Mega. The result is a clearer separation of roles:

- the Mega owns reactive local obstacle motion;
- the Raspberry Pi owns high-level navigation intent;
- the dashboard AI tools remain available for visual analysis and experimental driving modes;
- but AI is no longer the normal safety path for obstacle response.

This result is important because it reduces ambiguity about which subsystem is responsible for not hitting an obstacle.

---

## 4.5 Integrated System Results

The completed robot now operates as an integrated supervised environmental monitoring platform whose major subsystems are all connected through the dashboard and the Pi-Mega control path.

**Table 4.5: Integrated System Capability Summary**

| Capability | Status in Final System |
|---|---|
| Calibrated gas display and warning banners | Implemented |
| Live browser dashboard with charts, map, and camera | Implemented |
| Direct Pi-to-Mega coordination | Implemented |
| LTE-backed remote dashboard access | Implemented |
| Local CC1101 manual control | Implemented |
| Dashboard USB joystick teleoperation | Implemented |
| Assisted waypoint navigation | Implemented |
| Servo-mounted local obstacle scan on Mega | Implemented |
| Mega fail-stop obstacle rule at 30 cm | Implemented |
| Fully trustworthy unsupervised autonomy in complex environments | Not claimed |

Taken together, these results show that the final system is strongest as a remotely supervised environmental aide platform. It can sense, stream, map, warn, accept joystick commands, accept dashboard commands, and perform assisted navigation, but it is not presented as a robot that should be trusted to navigate arbitrary outdoor environments blindly from a single ultrasonic sensor.

---

## 4.6 Summary

This chapter has presented the implemented results of the final system revision. The robot now provides calibrated environmental sensing, browser-based monitoring, Pi-to-Mega coordination, CC1101 manual control, dashboard USB joystick control, LTE-backed remote supervision, assisted waypoint navigation, and Mega-side servo-based obstacle handling. Just as importantly, the updated results clarify what the system is not: it is not a multi-radio backup-control platform, and it is not a fully trustworthy autonomous field robot. Those clarifications form the basis of the discussion in Chapter 5.

---

*End of Chapter 4*
