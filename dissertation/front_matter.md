# FRONT MATTER

---

## TITLE PAGE

&nbsp;

&nbsp;

&nbsp;

**ARM-BASED ENVIRONMENTAL MONITORING AND AIDE ROBOT**

&nbsp;

&nbsp;

**A dissertation submitted in partial fulfilment of the requirements for the degree of**

**Bachelor of Engineering in Electronic Engineering**

&nbsp;

&nbsp;

*by*

&nbsp;

**[STUDENT FULL NAME]**

**[Student Number]**

&nbsp;

&nbsp;

**[Department Name]**

**[Faculty Name]**

**[University Name]**

&nbsp;

&nbsp;

**Supervisor: [Supervisor Name and Title]**

&nbsp;

&nbsp;

**March 2026**

---

&nbsp;

## DECLARATION

I, [Student Full Name], declare that this dissertation is my own unaided work. It is submitted in partial fulfilment of the requirements for the degree of Bachelor of Engineering in Electronic Engineering at [University Name]. It has not been submitted before for any degree or examination in any other university.

&nbsp;

&nbsp;

Signed: \_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_

Date: \_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_

---

&nbsp;

## DEDICATION

*To my family, whose patience and encouragement made this work possible.*

*To my supervisor, whose guidance shaped this project from concept to completion.*

---

&nbsp;

## ACKNOWLEDGEMENTS

The author wishes to express sincere gratitude to the following individuals and institutions:

To [Supervisor Name and Title], for expert guidance, constructive criticism, and continued encouragement throughout the duration of this project. The weekly supervision meetings and detailed feedback on drafts were invaluable in shaping both the technical work and its documentation.

To the staff of the [Department Name] at [University Name], for providing access to the electronics laboratory, hardware components, and technical resources required to build and test the robot platform.

To fellow students who participated as test observers during the field validation trials, and whose feedback contributed to the refinement of the navigation and obstacle avoidance subsystems.

To the open-source community whose contributions to Python, Flask, GeographicLib, TinyGPSPlus, Arduino, Leaflet.js, and Chart.js made this project possible within the time and budget constraints of an undergraduate thesis.

Finally, to my family and friends, for their patience and moral support throughout the long hours of hardware debugging, software development, and writing.

---

&nbsp;

## ABSTRACT

This dissertation presents the design, implementation, and validation of an ARM-based Environmental Monitoring and Aide Robot — an autonomous mobile platform capable of navigating GPS-defined waypoints while continuously acquiring, calibrating, and transmitting real-time environmental sensor data. The system employs a dual-processor architecture: a Raspberry Pi 3 Model B running a seven-thread Python 3 application for high-level navigation, network communication, and web-based monitoring; and an Arduino Mega 2560 implementing a non-blocking three-state firmware (STATE_I2C / STATE_WIRELESS / STATE_FAILSAFE) for real-time motor control, GPS parsing, and obstacle sensing.

The sensing suite comprises a DHT11 temperature and humidity sensor, and three MQ-series gas sensors (MQ-2, MQ-135, MQ-7) interfaced via a 16-bit ADS1115 ADC, with all gas concentrations expressed in calibrated parts-per-million (PPM) using a validated log-log power-law model. Navigation employs a NEO-6M GPS receiver and a QMC5883L compass, with GeographicLib Karney WGS-84 geodesic calculations providing sub-metre bearing accuracy. A servo-mounted HC-SR04 ultrasonic sensor and a KY-032 IR proximity sensor provide three-direction obstacle avoidance with a sub-4 µs emergency motor-halt capability. Dual-redundant wireless control is provided by a ZigBee XBee module (primary, ~100 m) and a CC1101 433 MHz radio (secondary, ~200 m).

A browser-based web dashboard, implemented with Flask, Flask-SocketIO, Leaflet.js, and Chart.js, delivers real-time sensor charts, GPS mapping, live MJPEG camera streaming, and remote navigation commands with a measured end-to-end update latency of 17.4 ms — 860 times lower than the minimum update interval of the cloud IoT platform used by the reference study.

System validation confirmed a 100% GPS waypoint arrival success rate (10/10, mean error 2.32 m), 97% obstacle avoidance success (34/35 trials), zero I2C communication failures in 350 transactions, and zero system faults in a complete five-waypoint, 6-minute 43-second autonomous outdoor mission. The total system cost is approximately USD 120–140, within the USD 150 budget constraint. The project directly extends the work of Salman et al. [17] by addressing seven specific gaps identified in the literature: operator platform independence, quantitative gas calibration, directional obstacle avoidance, local data persistence, dual-redundant wireless, extended range, and real-time rather than cloud-delayed monitoring.

**Keywords:** Autonomous robot, environmental monitoring, Raspberry Pi, Arduino Mega, GPS navigation, gas sensor calibration, IoT, obstacle avoidance, web dashboard, dual-redundant wireless.

---

&nbsp;

## TABLE OF CONTENTS

| Section | Page |
|---|---|
| Declaration | ii |
| Dedication | iii |
| Acknowledgements | iv |
| Abstract | v |
| Table of Contents | vi |
| List of Figures | viii |
| List of Tables | x |
| List of Abbreviations | xiii |
| Glossary | xv |
| **Chapter 1: Introduction** | **1** |
| 1.1 Background and Motivation | 1 |
| 1.2 Problem Statement | 3 |
| 1.3 Research Objectives | 4 |
| 1.4 Research Questions | 5 |
| 1.5 Scope and Delimitations | 5 |
| 1.6 Significance of the Study | 6 |
| 1.7 Dissertation Structure | 7 |
| **Chapter 2: Literature Review** | **8** |
| 2.1 Introduction | 8 |
| 2.2 Environmental Monitoring Systems | 8 |
| 2.3 ARM-Based Embedded Platforms for Robotics | 10 |
| 2.4 Environmental Sensors and Gas Concentration Calibration | 12 |
| 2.5 GPS-Based Navigation and Waypoint Following | 14 |
| 2.6 Obstacle Detection and Avoidance | 16 |
| 2.7 IoT Data Management and Dashboard Platforms | 17 |
| 2.8 Wireless Communication for Mobile Robots | 19 |
| 2.9 Summary and Research Gaps | 21 |
| **Chapter 3: Methodology** | **23** |
| 3.1 Introduction | 23 |
| 3.2 Requirements Specification | 23 |
| 3.3 System Architecture Overview | 26 |
| 3.4 Hardware Design | 28 |
| 3.5 Sensor Integration and Calibration | 37 |
| 3.6 Software Architecture | 43 |
| 3.7 Communication Protocols | 50 |
| 3.8 Navigation Algorithm Design | 57 |
| 3.9 Web Dashboard Design | 63 |
| 3.10 Testing and Validation Methodology | 67 |
| 3.11 Summary | 70 |
| **Chapter 4: Results** | **71** |
| 4.1 Introduction | 71 |
| 4.2 Sensor Subsystem Results | 71 |
| 4.3 I2C Communication Protocol Results | 78 |
| 4.4 Navigation Subsystem Results | 82 |
| 4.5 Obstacle Avoidance Results | 88 |
| 4.6 Wireless Communication Results | 92 |
| 4.7 Web Dashboard Performance Results | 96 |
| 4.8 Integrated System Results | 100 |
| 4.9 System Resource Utilisation | 106 |
| 4.10 Summary | 108 |
| **Chapter 5: Discussion** | **109** |
| 5.1 Introduction | 109 |
| 5.2 Achievement of Research Objectives | 109 |
| 5.3 Comparison with Salman et al. [17] and Related Work | 116 |
| 5.4 Answers to Research Questions | 120 |
| 5.5 Limitations | 122 |
| 5.6 Implications | 124 |
| 5.7 Summary | 125 |
| **Chapter 6: Conclusion** | **126** |
| 6.1 Introduction | 126 |
| 6.2 Summary of the Research | 126 |
| 6.3 Achievement of Research Objectives | 128 |
| 6.4 Research Contributions | 130 |
| 6.5 Limitations | 131 |
| 6.6 Recommendations for Future Work | 132 |
| 6.7 Concluding Remarks | 134 |
| **Reference List** | **135** |
| **Appendix A: Raspberry Pi Main Controller Source Code** | **140** |
| **Appendix B: Arduino Mega Navigation Firmware Source Code** | **155** |
| **Appendix C: Navigation Controller Source Code** | **170** |
| **Appendix D: Gas Calibration Module Source Code** | **183** |

---

&nbsp;

## LIST OF FIGURES

| Figure | Caption | Page |
|---|---|---|
| Figure 1.1 | Conceptual overview of the ARM-based Environmental Monitoring and Aide Robot system | 2 |
| Figure 2.1 | Comparison of fixed-station and mobile environmental monitoring approaches | 9 |
| Figure 2.2 | Dual-processor ARM architecture of Salman et al. [17] | 11 |
| Figure 2.3 | MQ-series sensor output characteristic curve (log-log PPM vs Rs/R0) | 13 |
| Figure 2.4 | GPS waypoint navigation geometry showing bearing and arrival radius | 15 |
| Figure 2.5 | Servo-scan obstacle avoidance with three-direction ultrasonic sensing | 17 |
| Figure 2.6 | Comparison of IoT dashboard update latency: local WebSocket vs cloud polling | 18 |
| Figure 2.7 | Wireless range comparison: Bluetooth, ZigBee, CC1101 433 MHz | 20 |
| Figure 3.1 | System architecture block diagram: Raspberry Pi, Arduino Mega, sensors, wireless | 27 |
| Figure 3.2 | Physical robot platform showing component placement and wiring layout | 29 |
| Figure 3.3 | Raspberry Pi 3 Model B GPIO pin assignment diagram | 31 |
| Figure 3.4 | Arduino Mega 2560 pin assignment diagram | 33 |
| Figure 3.5 | ADS1115 ADC schematic showing MQ-sensor voltage divider network | 38 |
| Figure 3.6 | Log-log calibration curve for MQ-135 (CO₂): PPM vs Rs/R0 | 40 |
| Figure 3.7 | Log-log calibration curve for MQ-2 (Smoke): PPM vs Rs/R0 | 41 |
| Figure 3.8 | Log-log calibration curve for MQ-7 (CO): PPM vs Rs/R0 | 42 |
| Figure 3.9 | Seven-thread software architecture of the Raspberry Pi controller | 44 |
| Figure 3.10 | NavController eleven-state machine state transition diagram | 58 |
| Figure 3.11 | Arduino Mega three-state machine: STATE_I2C / STATE_WIRELESS / STATE_FAILSAFE | 62 |
| Figure 3.12 | Web dashboard layout: sensor charts, GPS map, camera stream, navigation controls | 64 |
| Figure 3.13 | I2C three-step atomic waypoint loading sequence diagram | 54 |
| Figure 4.1 | DHT11 temperature measurement over 60-minute test period showing ±0.4°C envelope | 73 |
| Figure 4.2 | MQ-135 CO₂ PPM output compared to NOAA atmospheric reference (≈421 PPM) | 77 |
| Figure 4.3 | GPS track overlaid on Leaflet.js map: five-waypoint mission | 84 |
| Figure 4.4 | Heading error vs time during ACQUIRING_HEADING phase: convergence within 10s | 86 |
| Figure 4.5 | HC-SR04 servo-scan three-direction distance readings during obstacle test | 89 |
| Figure 4.6 | ZigBee packet reception rate vs distance (0–120 m outdoor test) | 93 |
| Figure 4.7 | CC1101 packet reception rate vs distance (0–220 m outdoor test) | 94 |
| Figure 4.8 | Dashboard end-to-end sensor update latency distribution (n=500 samples) | 97 |
| Figure 4.9 | CPU and RAM utilisation during full autonomous mission (6 min 43 s) | 107 |
| Figure 5.1 | Radar chart comparing this system vs Salman et al. [17] across six capability dimensions | 118 |

---

&nbsp;

## LIST OF TABLES

| Table | Caption | Page |
|---|---|---|
| Table 3.1 | System requirements specification (SR-01 to SR-11) | 24 |
| Table 3.2 | Raspberry Pi 3 Model B technical specifications | 30 |
| Table 3.3 | Arduino Mega 2560 technical specifications | 32 |
| Table 3.4 | DHT11 sensor specifications and wiring | 34 |
| Table 3.5 | ADS1115 ADC configuration | 35 |
| Table 3.6 | MQ-2 gas sensor specifications | 35 |
| Table 3.7 | MQ-135 gas sensor specifications | 36 |
| Table 3.8 | MQ-7 gas sensor specifications | 36 |
| Table 3.9 | NEO-6M GPS module specifications | 37 |
| Table 3.10 | QMC5883L compass module specifications | 37 |
| Table 3.11 | I2C command set: 17 commands with opcode, payload, and response | 52 |
| Table 3.12 | NavController state machine: eleven states with entry conditions and actions | 59 |
| Table 3.13 | Navigation tuning parameters and their values | 61 |
| Table 3.14 | Gas calibration constants for five sensor/gas combinations | 39 |
| Table 4.1 | DHT11 temperature accuracy test (n=30, reference: calibrated thermometer) | 72 |
| Table 4.2 | DHT11 humidity accuracy test (n=30, reference: hygrometer) | 72 |
| Table 4.3 | ADS1115 channel voltage verification against reference multimeter | 74 |
| Table 4.4 | MQ-2 PPM output at known gas concentrations (smoke chamber test) | 75 |
| Table 4.5 | MQ-135 CO₂ PPM output under ambient and elevated CO₂ conditions | 76 |
| Table 4.6 | MQ-7 CO PPM output under ambient and simulated CO conditions | 77 |
| Table 4.7 | I2C command transaction latency (n=350): mean, max, and failure rate | 79 |
| Table 4.8 | I2C command latency by command type | 80 |
| Table 4.9 | I2C bus recovery test: 9-clock recovery cycle success rate | 81 |
| Table 4.10 | GPS waypoint arrival test: Run 1 (5 waypoints) | 83 |
| Table 4.11 | GPS waypoint arrival test: Run 2 (5 waypoints) | 83 |
| Table 4.12 | Heading acquisition time by initial heading error | 85 |
| Table 4.13 | Navigation parameter sensitivity: WAYPOINT_RADIUS and HEADING_DEADBAND | 87 |
| Table 4.14 | Return-to-start accuracy test (3 trials) | 88 |
| Table 4.15 | HC-SR04 obstacle detection accuracy at range (0.1 m – 4.0 m) | 89 |
| Table 4.16 | Servo-scan three-direction obstacle avoidance success rate (35 trials) | 90 |
| Table 4.17 | KY-032 IR interrupt response latency (n=20) | 91 |
| Table 4.18 | Failsafe transition time test (10 trials) | 92 |
| Table 4.19 | ZigBee packet reception rate vs distance | 93 |
| Table 4.20 | CC1101 packet reception rate vs distance | 94 |
| Table 4.21 | Wireless link recovery time after deliberate interruption | 95 |
| Table 4.22 | Dashboard end-to-end sensor update latency (n=500) | 97 |
| Table 4.23 | Dashboard REST API endpoint response times | 98 |
| Table 4.24 | Camera MJPEG stream frame rate and latency | 99 |
| Table 4.25 | Offline data buffer replay test | 100 |
| Table 4.26 | Full autonomous mission results: five-waypoint outdoor test | 101 |
| Table 4.27 | Per-waypoint arrival data: full mission | 102 |
| Table 4.28 | Obstacle encounter log: full mission | 103 |
| Table 4.29 | End-to-end data transmission log: full mission (sample) | 104 |
| Table 4.30 | System resource utilisation during full mission | 106 |
| Table 5.1 | Comparative performance: this system vs Salman et al. [17] (20 criteria) | 117 |

---

&nbsp;

## LIST OF ABBREVIATIONS

| Abbreviation | Expansion |
|---|---|
| ADC | Analog-to-Digital Converter |
| ADS1115 | 16-bit, 4-channel ADC by Texas Instruments (I2C) |
| API | Application Programming Interface |
| ARM | Advanced RISC Machine |
| BCM | Broadcom Corporation (used to reference Raspberry Pi chip) |
| CEP | Circular Error Probable |
| CPU | Central Processing Unit |
| DHT11 | Digital Humidity and Temperature sensor (Aosong) |
| FPGA | Field-Programmable Gate Array |
| GPIO | General Purpose Input/Output |
| GPS | Global Positioning System |
| GSM | Global System for Mobile Communications |
| HC-SR04 | Ultrasonic range-finding module |
| HTTP | Hypertext Transfer Protocol |
| I2C | Inter-Integrated Circuit (two-wire serial bus) |
| IoT | Internet of Things |
| IR | Infrared |
| ISR | Interrupt Service Routine |
| JSON | JavaScript Object Notation |
| KY-032 | Infrared obstacle avoidance sensor module |
| LAN | Local Area Network |
| LPG | Liquefied Petroleum Gas |
| LTE | Long-Term Evolution (4G cellular) |
| MJPEG | Motion JPEG (video streaming format) |
| MOS | Metal Oxide Semiconductor |
| MQ-2 | Semiconductor gas sensor for smoke/LPG/CH₄ |
| MQ-7 | Semiconductor gas sensor for carbon monoxide |
| MQ-135 | Semiconductor gas sensor for air quality/CO₂/NH₃ |
| NDIR | Non-Dispersive Infrared (CO₂ reference analyser) |
| NTRIP | Networked Transport of RTCM via Internet Protocol |
| PCB | Printed Circuit Board |
| PPM | Parts Per Million |
| PWM | Pulse Width Modulation |
| QMC5883L | 3-axis magnetic sensor (compass) |
| RAM | Random Access Memory |
| REST | Representational State Transfer |
| RTK | Real-Time Kinematic (GPS correction method) |
| SCL | Serial Clock Line (I2C) |
| SDA | Serial Data Line (I2C) |
| SIM7600E | SIMCom LTE/GPS module |
| SPI | Serial Peripheral Interface |
| SQLite | Serverless SQL database engine |
| UART | Universal Asynchronous Receiver-Transmitter |
| URL | Uniform Resource Locator |
| USB | Universal Serial Bus |
| UWB | Ultra-Wideband |
| WGS-84 | World Geodetic System 1984 (GPS datum) |
| Wi-Fi | Wireless Fidelity (IEEE 802.11) |
| WS | WebSocket |

---

&nbsp;

## GLOSSARY

**Bearing** — The initial compass direction, measured in degrees (0°–360°) clockwise from true north, from the robot's current position to a target waypoint. Computed using GeographicLib Karney WGS-84 geodesic calculations.

**Calibration (gas sensor)** — The process of establishing the relationship between a sensor's raw ADC output and the actual concentration of a target gas, expressed in parts per million (PPM), using a log-log power-law model of the form PPM = a × (Rs/R0)^b.

**Course drift** — A navigation condition in which the robot's actual compass heading deviates from the target bearing by more than the COURSE_DRIFT_THRESH (12°), triggering a re-acquisition of the heading.

**Deadband** — A tolerance window around the target bearing (HEADING_DEADBAND = 5°) within which the heading error is considered acceptable and forward motion is permitted.

**Failsafe** — A deterministic safety state (STATE_FAILSAFE) entered by the Arduino Mega when both the I2C link to the Raspberry Pi and the CC1101 wireless link have been lost for more than 500 ms. In this state, all motor outputs are halted immediately.

**GeographicLib** — A C++/Python library implementing Karney's algorithm for geodesic calculations on the WGS-84 reference ellipsoid, providing sub-millimetre accuracy in bearing and distance calculations.

**Haversine formula** — A spherical trigonometry formula used to calculate great-circle distance between two GPS coordinates, used as a fallback when GeographicLib is unavailable.

**Heading error** — The signed angular difference (–180° to +180°) between the robot's current compass heading and the target bearing to the next waypoint.

**NavController** — The Pi-side Python navigation engine that implements the eleven-state state machine and sends simple motor drive commands to the Arduino Mega over I2C.

**PPM (parts per million)** — A dimensionless unit expressing the concentration of a trace gas in air, defined as one gas molecule per million air molecules. Used for all gas sensor outputs in this project.

**R0** — The baseline resistance of an MQ-series gas sensor measured in clean air at standard conditions. Used in the PPM calibration formula as the normalisation reference.

**Rs** — The instantaneous resistance of an MQ-series gas sensor in the presence of a measured gas. Computed from the output voltage using the load resistor equation.

**Servo scan** — A technique in which the HC-SR04 ultrasonic sensor is mounted on an SG90 servo motor and rotated to three angular positions (left, centre, right) to obtain directional obstacle distance measurements.

**State machine** — A computational model that exists in exactly one of a finite set of states at any given time, transitioning between states in response to defined inputs or conditions. Used in both the NavController (eleven navigation states) and the Arduino Mega firmware (three communication states).

**Superloop** — A firmware architecture for microcontrollers in which a single main loop executes all system tasks in sequence without blocking, using time-stamped non-blocking checks rather than `delay()` or blocking I/O calls.

**UART** — Universal Asynchronous Receiver-Transmitter. A serial communication protocol used for GPS (Serial1 on Mega), debug output (Serial0), and ZigBee (Serial2).

**Waypoint** — A geographic coordinate (latitude, longitude) stored in the system database and loaded into the navigation controller. The robot navigates to each waypoint in sequence during autonomous operation.

**WebSocket** — A full-duplex communication protocol providing a persistent, low-latency connection between the Flask server and browser clients, used to push sensor and GPS updates to the dashboard at sub-20 ms latency.

**WGS-84** — World Geodetic System 1984. The standard reference coordinate system used by GPS, defining the Earth's shape as an oblate spheroid and specifying the datum for all latitude/longitude coordinates.

---

*End of Front Matter*
