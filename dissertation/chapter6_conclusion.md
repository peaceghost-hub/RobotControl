# CHAPTER 6: CONCLUSION

---

## 6.1 Introduction

This chapter presents the concluding remarks of the dissertation. It provides a concise summary of the research undertaken, confirms the extent to which each research objective was met, consolidates the key contributions of the project to the field of embedded robotics and environmental monitoring, identifies the limitations acknowledged in Chapter 5, and presents a structured set of recommendations for future work that would extend the capabilities of the system. The chapter closes with a final reflective statement on the significance of the project.

---

## 6.2 Summary of the Research

This dissertation has presented the design, implementation, and validation of an ARM-based Environmental Monitoring and Aide Robot — an autonomous mobile robotic system capable of navigating GPS-defined waypoints, acquiring real-time environmental sensor data, and transmitting that data to a browser-accessible web dashboard over both local Wi-Fi and LTE cellular networks.

The motivation for the research arose from the well-documented limitations of static environmental monitoring networks — their inability to reposition in response to changing pollution events, and their complete unsuitability for deployment in areas hazardous to human health. The work of Salman, Rahman, Tarek, and Wang [17], who demonstrated a GPS-guided ARM-based environmental monitoring robot using a Raspberry Pi and Arduino Mega, provided the direct technical foundation from which this project was developed. The literature review identified seven specific gaps in that and comparable published systems: short-range Bluetooth-only control, Android-only operator interface, raw rather than calibrated gas sensor output, single-sensor fixed-angle obstacle avoidance, no local data persistence, no GPS redundancy, and limited gas species coverage. Each of these gaps was explicitly addressed in the system design.

The hardware architecture adopted the dual-processor pattern validated by Salman et al. [17], with the Raspberry Pi 3 Model B managing high-level control (navigation mathematics, internet communication, database management, web server) and the Arduino Mega 2560 managing real-time low-level control (motor PWM, GPS serial parsing, compass reading, obstacle sensing, wireless radio interfacing). The two processors communicated via a robust I2C protocol with a 17-command set, three-step atomic waypoint loading, and an automatic bus recovery mechanism.

The sensing suite comprised a DHT11 temperature and humidity sensor, an ADS1115 16-bit ADC, and three MQ-series gas sensors (MQ-2 for smoke/LPG, MQ-135 for CO₂/NH₃/air quality, MQ-7 for carbon monoxide), with all gas sensor outputs converted to calibrated parts-per-million concentrations using a validated log-log power-law model derived from manufacturer datasheets. Navigation employed a NEO-6M GPS receiver and a QMC5883L compass on the Arduino Mega, with a secondary SIM7600E integrated GPS on the Raspberry Pi providing redundancy. Obstacle avoidance combined a servo-mounted HC-SR04 ultrasonic sensor performing three-direction distance scanning with a KY-032 IR proximity sensor providing a sub-4 µs hardware interrupt motor-halt capability.

The Raspberry Pi software was implemented as a seven-thread Python 3 application, with a dedicated 10 Hz navigation state machine (eleven states from IDLE through COMPLETE), GeographicLib Karney WGS-84 geodesic calculations, and offline data buffering for communication resilience. The Arduino firmware implemented a strict non-blocking superloop with a three-state machine (STATE_I2C, STATE_WIRELESS, STATE_FAILSAFE) governing motor ownership and priority between autonomous navigation and manual control. The web dashboard, implemented with Flask, Flask-SocketIO, Leaflet.js, and Chart.js, provided real-time sensor charts, interactive GPS mapping, live MJPEG camera streaming, waypoint management, and remote navigation commands, all updated with a mean end-to-end latency of 17.4 ms via WebSocket.

Testing validated each subsystem individually and in end-to-end combination. The complete system was demonstrated in a full autonomous mission: five GPS waypoints successfully navigated, two obstacles successfully avoided, 280 data events transmitted without loss, and zero system faults, over a 6-minute 43-second outdoor mission.

---

## 6.3 Achievement of Research Objectives

The degree to which each of the six research objectives defined in Chapter 1 was achieved is summarised below.

**OBJ-1 — Dual-Processor Hardware Architecture:** *Fully achieved.* The Raspberry Pi 3 Model B and Arduino Mega 2560 dual-processor architecture was successfully designed, implemented, and validated. The I2C inter-processor link achieved zero communication failures across 350 test transactions with a maximum latency of 1,243 µs, and the Raspberry Pi operated at a mean CPU utilisation of 18.7% during the full mission — confirming that the computational partition is both reliable and well-balanced.

**OBJ-2 — Multi-Sensor Suite and PPM Calibration:** *Fully achieved.* All four sensors (DHT11, MQ-2, MQ-135, MQ-7) were successfully integrated and calibrated. The DHT11 achieved ±0.4°C temperature accuracy and ±1.4% RH humidity accuracy, both within manufacturer tolerances. The gas sensor PPM calibration module produced a CO₂ reading of ~420 PPM under ambient conditions, closely matching the known global atmospheric CO₂ concentration (~421 PPM), providing strong empirical validation of the calibration model.

**OBJ-3 — GPS Waypoint Navigation with Obstacle Avoidance:** *Fully achieved.* The navigation system achieved a 100% waypoint arrival success rate (10/10 arrivals across two runs) with a mean positioning error of 2.32 m within the 3.0 m acceptance radius. The servo-scan dual-sensor obstacle avoidance achieved a 97% success rate across 35 trials. The KY-032 IR interrupt provided a 3.8 µs motor-halt response for close-proximity emergencies. The return-to-start function successfully navigated the robot to within 2.7 m of its origin.

**OBJ-4 — Real-Time Web Dashboard:** *Fully achieved.* The Flask/SocketIO dashboard provided end-to-end sensor update latency of 17.4 ms (mean) — a 860-fold improvement over the ThingSpeak 15-second minimum update interval used by Salman et al. [17]. The dashboard was successfully accessed from three different device types (desktop, laptop, smartphone) without any platform-specific configuration, confirming platform independence. The MJPEG camera stream was delivered at 9.4 FPS in the browser. The offline data buffer successfully replayed 36 queued sensor readings within 4.8 seconds of dashboard reconnection.

**OBJ-5 — Dual-Redundant Wireless Communication:** *Fully achieved.* The ZigBee primary link achieved 97% packet reception at 100 m and the CC1101 secondary link achieved 94% reception at 200 m, providing overlapping dual-redundant coverage up to 100 m and single-channel coverage to 200 m. The three-state failsafe machine transitioned to STATE_FAILSAFE (with immediate motor halt) within 530 ms of dual communication loss, and recovered to active wireless control within 98 ms of link restoration.

**OBJ-6 — Integrated System Validation:** *Fully achieved.* The complete system was validated through unit testing of all subsystems, functional testing of the I2C protocol, compass calibration, ADS1115 verification, and end-to-end autonomous mission testing. The full-mission test demonstrated 5/5 waypoints reached, 2/2 obstacles avoided, zero missed dashboard updates, and zero system faults across a 6-minute 43-second autonomous outdoor mission.

---

## 6.4 Research Contributions

This project makes the following specific contributions to the field of embedded robotics and environmental monitoring:

1. **A complete, open-source, validated implementation** of an ARM-based dual-processor environmental monitoring robot that integrates, within a single coherent system, capabilities that the existing literature — including Salman et al. [17] — addresses only in isolation or incompletely: GPS waypoint navigation, multi-species gas sensing with calibrated PPM output, dual-redundant wireless control with failsafe, real-time browser-based dashboard, and local persistent data storage.

2. **A validated log-log PPM calibration framework** for MQ-2, MQ-135, and MQ-7 sensors, with empirical validation against the known atmospheric CO₂ baseline. This framework is replicable and extensible to other MQ-series sensors, and establishes quantitative PPM reporting as a practical standard for low-cost environmental monitoring systems.

3. **A dual-redundant wireless control architecture** with formally specified state machine semantics and deterministic failsafe behaviour, extending the reliable operational range of the system from the ~10 m Bluetooth range of comparable published systems to ~200 m, and providing a defined safety response to complete communication loss.

4. **A browser-based real-time monitoring dashboard** delivering end-to-end sensor updates at 17.4 ms mean latency — three orders of magnitude faster than cloud IoT platforms subject to minimum update intervals — enabling genuinely real-time situational awareness without proprietary software or mobile app installation.

5. **A servo-scan directional obstacle avoidance system** combining three-direction ultrasonic distance scanning with a parallel IR interrupt for sub-4 µs emergency stopping, demonstrating a practical and cost-effective improvement over the fixed-mount single-sensor approaches prevalent in the literature.

---

## 6.5 Limitations

The following limitations of the current implementation were identified in Chapter 5 and are restated here for completeness:

- GPS positional accuracy is bounded by the NEO-6M receiver's 2.3 m CEP, limiting waypoint arrival precision to approximately 2–3 m under typical satellite conditions.
- Gas sensor absolute accuracy against certified reference instruments was not formally quantified due to the unavailability of calibrated reference gas equipment during testing. Cross-sensitivity to multiple gases and ambient temperature/humidity dependence are inherent characteristics of MOS sensors that were not compensated in the current implementation.
- No battery life characterisation or power management strategy was implemented, limiting understanding of operational endurance.
- The reactive obstacle avoidance strategy has no memory of past obstacles and may exhibit repeated-encounter inefficiencies.
- The system is limited to outdoor operation; GPS and compass-based navigation are not functional in indoor environments.
- The SIM7600E module was operated over USB rather than the native UART interface due to intermittent UART connection issues; native UART operation was not fully validated.
- The Raspberry Pi represents a single point of failure for all high-level functions; no hardware watchdog or software crash recovery mechanism was implemented beyond the Arduino's STATE_FAILSAFE motor halt.

---

## 6.6 Recommendations for Future Work

Based on the findings of this project and the limitations identified above, the following recommendations are made for future development:

**1. RTK GNSS Integration for Sub-Metre Navigation Accuracy**
The adoption of a Real-Time Kinematic (RTK) GNSS receiver — such as the u-blox ZED-F9P — in place of the NEO-6M would reduce positional accuracy from 2.3 m CEP to approximately 1–2 cm with RTK correction signals. This would enable precision environmental sampling at specific survey coordinates and significantly improve the quality of GPS-tagged sensor data for scientific analysis. RTK base station corrections are available over the internet (NTRIP protocol) and could be delivered to the robot via the existing LTE connection.

**2. Temperature-Humidity Compensated Gas Sensor Readings**
Implementing temperature and humidity compensation for the MQ-series sensor readings — by incorporating the correction factors provided in sensor datasheets as functions of ambient temperature and relative humidity — would improve the accuracy and consistency of PPM readings across different environmental conditions. The DHT11 readings already acquired by the system could serve as the input to these compensation equations, requiring only a software update to the `GasCalibration` module.

**3. Absolute Gas Sensor Calibration Against Reference Instruments**
Future validation of the gas sensor calibration model against a certified non-dispersive infrared (NDIR) CO₂ analyser and electrochemical CO sensor — available in environmental monitoring laboratories — would quantify the absolute accuracy of the PPM calibration model and establish measurement uncertainty bounds, increasing the scientific credibility of the collected data.

**4. Solar Power and Energy Harvesting**
As noted by Salman et al. [17] as a future direction in their own work, the integration of solar panels and a charge controller would significantly extend the operational endurance of the robot, enabling multi-day autonomous deployment without battery replacement. Given the robot's outdoor operating environment, solar charging is technically and practically well-suited to this application.

**5. Occupancy Grid Mapping and Deliberative Path Planning**
Replacing the current reactive obstacle avoidance algorithm with a deliberative path planner — using an occupancy grid map built from accumulated HC-SR04 measurements and updated in real time — would eliminate the repeated-encounter inefficiency of the reactive strategy and enable globally optimal path planning around complex obstacle fields. The Raspberry Pi's computational headroom (18.7% mean CPU during full mission) is sufficient to support a lightweight occupancy grid implementation.

**6. Multi-Robot Coordination**
The existing ZigBee wireless infrastructure supports mesh networking, which would enable a fleet of multiple robots to share GPS positions, divide a survey area into zones, and collaboratively monitor a large spatial area more efficiently than a single robot. The dashboard's multi-device support (`SUPPORTED_DEVICE_IDS` configuration) already anticipates this extension; the primary development effort would be in the inter-robot coordination protocol.

**7. Particulate Matter (PM2.5 / PM10) Sensing**
The addition of a laser particle counter sensor (such as the PMS5003 or SDS011, both of which communicate over UART and are supported by Python libraries) would extend the environmental monitoring suite to include fine particulate matter concentration — a critical metric for urban air quality assessment that is absent from the current sensor suite and from most comparable published systems.

**8. Raspberry Pi Hardware Watchdog**
Enabling the BCM2837's hardware watchdog timer via `/dev/watchdog` and feeding it from the main control loop would provide automatic operating system restart on software crash or deadlock, eliminating the single-point-of-failure risk identified in Section 5.5.

**9. Indoor Navigation Capability**
Extending the system for indoor operation would require an alternative localisation technology to GPS. Ultra-wideband (UWB) ranging using a set of fixed anchor nodes (e.g., Decawave DW1000 modules) would provide sub-10 cm indoor positioning accuracy and is available in low-cost modules. This extension would enable the robot to operate in factory floors, warehouses, or other indoor industrial environments where gas hazard monitoring is equally or more relevant than outdoor deployment.

**10. Machine Learning-Based Anomaly Detection**
Incorporating a lightweight anomaly detection model — trained on baseline sensor readings and capable of flagging statistically unusual combinations of gas concentrations — would transition the system from threshold-based alerting (which requires pre-setting individual gas thresholds) to data-driven anomaly detection that can identify novel pollution events without prior knowledge of their specific signature. The Raspberry Pi's ARM processor supports TensorFlow Lite inference, which would be a suitable inference engine for this purpose.

---

## 6.7 Concluding Remarks

Environmental monitoring in hazardous or remote locations is a challenge that has traditionally required either accepting the risk of human exposure or accepting the significant limitations of static sensor networks. The ARM-based Environmental Monitoring and Aide Robot developed in this project demonstrates that a system costing approximately USD 120–140, built entirely from commercially available components and open-source software, can autonomously navigate GPS waypoints, collect calibrated environmental data across four sensor modalities, stream live video, maintain a real-time browser dashboard updated at sub-20 ms latency, and operate safely with deterministic failsafe behaviour — all simultaneously.

The validation of the system through end-to-end autonomous mission testing confirms that the integration of these capabilities is not merely theoretical: the robot completed a five-waypoint, 42.8-metre outdoor navigation mission with 100% waypoint success, 100% obstacle avoidance, and zero data loss. The gap between what this project achieves and what is represented in the existing literature demonstrates the rapid pace of capability improvement that has been made possible by the maturation of ARM-based single-board computer platforms, open-source Python robotics libraries, and web-based visualisation frameworks.

The seven research gaps identified from the literature have each been addressed, and the system's performance — particularly the 860-fold improvement in dashboard update latency, the tenfold extension of wireless operational range, and the addition of quantitative PPM calibration — represents a meaningful advancement over the state of the art established by Salman et al. [17]. It is the author's hope that this work serves as a reproducible, well-documented foundation upon which future researchers and practitioners can build more capable, more accurate, and more widely deployed autonomous environmental monitoring systems.

---

*End of Chapter 6*
