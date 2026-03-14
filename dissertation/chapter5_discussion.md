# CHAPTER 5: DISCUSSION

---

## 5.1 Introduction

This chapter interprets the results presented in Chapter 4 in the context of the research questions and objectives defined in Chapter 1. It evaluates the degree to which each objective was achieved, discusses the significance of the measured performance metrics, compares the system's performance against the benchmark established by Salman et al. [17] and other systems reviewed in the literature, and critically examines the limitations of the current implementation. The chapter concludes by identifying areas where the current design could be extended or improved in future work, which are further developed in Chapter 6.

---

## 5.2 Discussion of Research Objectives

### 5.2.1 OBJ-1: Dual-Processor Hardware Architecture

The first objective required the design and implementation of a dual-processor architecture using a Raspberry Pi 3 Model B as the high-level controller and an Arduino Mega 2560 as the real-time navigation processor. The architecture was successfully realised and validated throughout the testing programme. The partitioning of responsibilities — the Raspberry Pi managing environmental sensing, network communication, navigation mathematics, and the web server, while the Arduino Mega handled motor PWM generation, GPS serial parsing, compass reading, wireless radio interfacing, and obstacle sensor interrupts — proved to be a sound and effective engineering decision.

The results of Section 4.5.1 demonstrated that the I2C inter-processor link achieved zero communication failures across 350 test transactions, with a maximum latency of 1,243 µs well within the 200 ms configured timeout. This confirms that the I2C protocol, combined with the thread-safe locking implementation on the Raspberry Pi and the ISR-deferred command processing on the Arduino, provides a reliable and deterministic communication channel for the dual-processor architecture.

A key validation of the architectural decision is the resource utilisation data of Table 4.25: during the full mission, the Raspberry Pi's four cores were utilised at a mean of 18.7% and a peak of 34.1%, while RAM usage remained stable at approximately 387 MB out of 1 GB available. This headroom demonstrates that the Raspberry Pi is not a computational bottleneck and confirms that the dual-processor partition is appropriately balanced — the Arduino handles time-critical real-time tasks, freeing the Raspberry Pi to perform computation-intensive tasks (geodesic calculations, HTTP/WebSocket serving, MJPEG encoding) without time-pressure.

The decision to place the navigation mathematics on the Raspberry Pi rather than the Arduino Mega — as was done by Salman et al. [17] — deserves specific discussion. In Salman et al.'s [17] design, the Arduino Mega executed the full navigation loop including Haversine calculations, bearing computation, and waypoint management. While this simplifies the inter-processor communication (the Arduino is fully autonomous in navigation), it has the disadvantage of requiring the compass data — in their case also on the Arduino's hardware I2C bus — to be on the same bus as the motor driver, creating interference risk. In the present design, relocating navigation mathematics to the Raspberry Pi allows the compass to live on the Raspberry Pi's own dedicated I2C bus, avoiding potential interference, at the cost of a round-trip I2C command for each motor drive instruction. The latency measurements confirm that this round-trip (< 1.3 ms) is entirely acceptable at the 10 Hz navigation loop rate.

### 5.2.2 OBJ-2: Multi-Sensor Environmental Monitoring Suite and PPM Calibration

The second objective required the integration and calibration of the DHT11 temperature and humidity sensor and the MQ-2, MQ-135, and MQ-7 gas sensors, with quantitative PPM conversion using validated datasheet calibration curves. This objective was fully achieved.

The DHT11 results (Table 4.1) demonstrated temperature accuracy within ±0.4°C and humidity accuracy within ±1.4% RH — both within the device's manufacturer-specified tolerances of ±2°C and ±5% RH. The integer-resolution temperature output (1°C steps) is a characteristic of the DHT11 protocol and represents the primary accuracy limitation of this particular sensor. Had a DHT22 been used, the resolution would improve to 0.1°C, enabling finer thermal monitoring. However, for the environmental survey application — where the primary concern is identifying significantly elevated temperatures indicative of fire or industrial heat sources rather than precise meteorological measurement — 1°C resolution is operationally adequate.

The most significant contribution to the literature achieved by this project in the sensing domain is the implementation of quantitative PPM calibration for all three MQ-series gas sensors. As noted in Section 2.4.3 of the Literature Review, Salman et al. [17] and the majority of comparable systems report raw ADC values rather than calibrated PPM concentrations, limiting the interpretability of the data for scientific and regulatory purposes. The validation result of approximately 420 PPM CO₂ from the MQ-135 sensor (Table 4.2) is highly significant: it closely matches the global atmospheric CO₂ baseline of approximately 421 PPM recorded by NOAA in 2025, providing strong empirical evidence that the calibration model is correctly implemented and producing physically meaningful results for at least the CO₂ channel.

The calibration limitation that must be acknowledged is the estimation of R₀ (the sensor resistance in clean air) directly from the instantaneous measurement rather than from a dedicated clean-air baseline measurement. In production deployments, the sensor should be operated in a known-clean environment for 24–48 hours to establish a reliable R₀ value, which is then hardcoded as a device-specific calibration constant. The present implementation estimates R₀ as $R_S / \text{ratio}_{\text{clean air}}$ on each measurement cycle, which means that if the sensor is powered on in a contaminated environment, the R₀ estimate will be systematically biased and all subsequent PPM readings will be in error. This is a known limitation of the datasheet log-log calibration method when clean-air baseline calibration is not performed, and represents the primary target for improvement in future sensor integration work.

The inclusion of the MQ-2 sensor for smoke and LPG detection, not present in Salman et al.'s [17] design, meaningfully extends the hazard detection capability of the system to include fire-related gases, which is particularly relevant for environmental survey in post-industrial or fire-affected areas. Together, the three gas sensors provide coverage of four distinct hazard categories: combustible gases (MQ-2), carbon monoxide poisoning (MQ-7), general air quality degradation from industrial pollutants (MQ-135), and fire/smoke presence (MQ-2 smoke channel).

### 5.2.3 OBJ-3: GPS Waypoint Navigation with Obstacle Avoidance

The third objective required the implementation of a GPS waypoint navigation system with compass heading correction and servo-scanned dual-sensor obstacle avoidance. The results of Sections 4.4 and 4.3.4 confirm that this objective was fully achieved.

The 100% waypoint arrival success rate (10 out of 10 arrivals across two navigation runs) with a mean positioning error of 2.32 m is the most direct measure of navigation system performance. This error is attributable almost entirely to the GPS receiver's inherent accuracy (2.3 m CEP from Table 4.5) rather than to any error in the navigation algorithm itself, a conclusion supported by the cross-track error data (mean 0.68–0.74 m, Table 4.13), which shows that the heading correction algorithm is keeping the robot closely aligned with the intended path. If the GPS receiver accuracy could be improved — for example by using a dual-frequency GNSS receiver or differential GPS corrections — the waypoint arrival error would decrease proportionally.

The cross-track error results deserve further interpretation. The mean CTE of approximately 0.7 m against the GPS positional uncertainty of 2.3 m indicates that the compass-based heading correction is actively and effectively countering path deviation, as a system relying purely on GPS-based waypoint bearing (without intermediate compass correction) would be expected to show CTE values approaching the GPS positional error. The heading correction deadband of 5° (HEADING_DEADBAND) was shown to be well-calibrated: it is small enough to prevent significant path deviation, yet large enough to avoid over-correction oscillation, which was not observed in either navigation run.

The heading acquisition results (Table 4.12) warrant further discussion. The linear relationship between initial heading error and acquisition time (approximately 0.08 s per degree of initial error) is consistent with the two-speed rotation strategy: at 150 PWM rotation speed for large errors and 100 PWM for small errors, the robot's angular velocity is roughly constant within each speed regime, producing approximately linear acquisition time scaling. The 30-second heading acquisition timeout was never reached in testing, which suggests it is set conservatively and could potentially be reduced in future work without risk of false timeouts.

The obstacle avoidance results (Table 4.9) show 97% overall success across 35 trials. The single failure in the "both sides partially blocked" configuration — where left and right clearance distances differed by only 2 cm — is an inherent limitation of the deterministic "select maximum clearance" decision rule in the presence of measurement noise. The HC-SR04 has a measurement noise of approximately ±0.3 cm at short distances (Table 4.8), meaning that adjacent distance measurements within 1 cm of each other are statistically indistinguishable at the sensor's native resolution. A probabilistic averaging approach (taking multiple measurements at each servo position and using the mean) would reduce this error mode, and is recommended as a future improvement.

The KY-032 IR sensor's measured interrupt-to-motor-stop latency of 3.8 µs is remarkable and deserves emphasis. At the robot's maximum drive speed (PWM 150, corresponding to approximately 0.35 m/s based on motor characterisation), 3.8 µs corresponds to a stopping reaction distance of approximately 1.3 µm — effectively instantaneous relative to any practical obstacle approach scenario. This confirms that the dual-sensor architecture's "fast stop, smart scan" design is correctly prioritised: the IR sensor eliminates close-proximity collision risk in the very worst case (rapidly approaching obstacle), while the HC-SR04 servo-scan provides the intelligence to navigate around obstacles.

### 5.2.4 OBJ-4: Real-Time Web Dashboard

The fourth objective required a real-time web dashboard accessible over both LAN and cellular networks. The results of Section 4.6 confirm this objective was fully and significantly achieved.

The end-to-end WebSocket update latency of 17.4 ms mean (Table 4.17) represents a fundamental improvement over the ThingSpeak 15-second minimum update interval used by Salman et al. [17]. The ratio of improvement is approximately 860:1 (15,000 ms / 17.4 ms), effectively transforming the operator's experience from a quasi-static data display that updates once every quarter-minute to a genuinely live dashboard that updates in near-real-time. This improvement is not merely quantitative but qualitative: at 17.4 ms latency, the dashboard sensor charts animate smoothly, and navigation events (waypoint reached, obstacle detected, state change) are visible to the operator within a human imperceptible delay.

The platform independence of the browser-based dashboard is a practical advantage over Salman et al.'s [17] Android-only application. During testing, the dashboard was successfully accessed from three different devices: a desktop PC running Ubuntu Linux (Firefox browser), a laptop running Windows 11 (Chrome browser), and a smartphone running Android (Chrome for Android). In all cases, the Leaflet.js map, Chart.js trend charts, camera stream, and control buttons functioned identically, confirming the platform-independence claim.

The offline data buffering result (Section 4.6.4) — successful replay of 36 buffered readings within 4.8 seconds of connectivity restoration — demonstrates an important resilience property not present in direct cloud-upload designs. In a real-world deployment where cellular connectivity may be intermittent, this capability ensures that sensor data collected during connectivity gaps is not lost.

The database performance results (4.2 MB for 17,280 records over 24 hours) project a database size of approximately 1.5 GB for one year of continuous operation, which is well within the capacity of the Raspberry Pi's microSD storage. The 30-day configurable retention policy ensures the database does not grow beyond approximately 126 MB under normal operating conditions.

### 5.2.5 OBJ-5: Dual-Redundant Wireless Communication

The fifth objective required a dual-redundant wireless communication architecture maintaining reliable control even if one channel fails. The results of Section 4.7 confirm this objective was achieved.

The ZigBee primary link's 97% reception rate at 100 m (Table 4.18) and the CC1101 secondary link's 94% reception rate at 200 m (Table 4.19) establish that the two channels together provide overlapping and complementary coverage. At distances up to 100 m, both channels are simultaneously operational, providing true redundancy: a temporary interference burst or momentary obstruction on one channel is masked by continuity on the other. Between 100 m and 200 m, the CC1101 continues to provide reliable single-channel control. This combined operational range substantially exceeds the approximately 10 m Bluetooth range of Salman et al.'s [17] design, increasing the practical operational radius by a factor of approximately 10–20.

The failsafe state machine validation results (Table 4.20) are critical for safety. The 530 ms maximum time to FAILSAFE transition, and the immediate (within one superloop iteration, ~2 ms) motor halt upon FAILSAFE entry, confirm that the system meets SR-07 (maintain controllability on single channel failure) and provides a deterministic safety response to total communication loss. The recovery time from FAILSAFE to STATE_WIRELESS upon ZigBee restoration (98 ms) is sufficiently fast to feel instantaneous to an operator, ensuring that communication recovery results in immediate command responsiveness.

The CC1101 secondary channel's sub-GHz (433 MHz) radio frequency provides an inherent advantage over the ZigBee 2.4 GHz band in environments with dense foliage or structural obstructions: sub-GHz signals diffract more readily around obstacles and penetrate vegetation more effectively than 2.4 GHz signals, making the CC1101 link more robust in real outdoor environments than the raw open-air range test might suggest.

### 5.2.6 OBJ-6: Integrated System Validation

The sixth objective required validation through functional testing of each subsystem and combined end-to-end testing. The full-mission test results (Table 4.22) demonstrate that the integrated system operates as a coherent whole: 5/5 waypoints reached, 2/2 obstacles avoided, 280 total data events transmitted without loss, and zero system faults over a 6-minute 43-second autonomous mission. The return-to-start result (Table 4.24) further validates the ring-buffer return algorithm, with the robot successfully returning to within 2.7 m of its origin.

The absence of any system crashes, reboots, or software exceptions during the end-to-end test is significant given the multi-threaded nature of the Raspberry Pi software. Seven concurrent threads share sensor data, GPS positions, and command state — a notoriously error-prone programming paradigm. The absence of deadlocks or race conditions during testing provides empirical validation of the threading lock strategy employed throughout the codebase.

---

## 5.3 Comparison with Salman et al. (2019)

A central purpose of this dissertation is to situate the present work in the context of the most directly relevant prior system: Salman et al.'s [17] GPS-controlled IoT and ARM-based environmental monitoring robot. Table 5.1 presents a structured feature-by-feature comparison.

**Table 5.1: Feature Comparison — Present System vs. Salman et al. [17]**

| Feature | Salman et al. [17] | Present System | Improvement |
|---|---|---|---|
| High-level processor | Raspberry Pi 3 Model B | Raspberry Pi 3 Model B | Same platform |
| Navigation processor | Arduino Mega 2560 | Arduino Mega 2560 | Same platform |
| Environmental sensors | DHT11, MQ-135, MQ-7 | DHT11, MQ-2, MQ-135, MQ-7 | + MQ-2 (smoke/LPG) |
| Gas concentration output | Raw ADC / voltage | Calibrated PPM (log-log model) | Quantitative & interpretable |
| GPS receiver | NEO-6M | NEO-6M + SIM7600E (dual) | Redundant GPS |
| Compass | HMC5883L (hardware I2C) | QMC5883L (software I2C, isolated) | Isolated from EMI |
| Obstacle sensor | HC-SR04 (fixed mount) | HC-SR04 (servo-scan) + KY-032 IR | Directional avoidance + fast stop |
| Control interface | Android app (Bluetooth) | Browser-based web dashboard (any device) | Platform independent |
| Control link | HC-06 Bluetooth (~10 m) | ZigBee (~100 m) + CC1101 (~200 m) | ~10–20× range, redundancy |
| Cloud platform | ThingSpeak (15 s update) | Local dashboard (17.4 ms) + optional ThingSpeak | 860× faster updates |
| Camera | None | Pi Camera V2 (MJPEG stream) | Live situational awareness |
| Data storage | ThingSpeak cloud only | Local SQLite + optional cloud | Offline persistence |
| Failsafe behaviour | Not described | STATE_FAILSAFE (motor halt within 2 ms) | Defined safety response |
| Return-to-start | Not described | 100-point GPS ring buffer | Autonomous return capability |
| Navigation math location | Arduino Mega | Raspberry Pi (NavController) | Compass colocation, lower latency |
| Cellular connectivity | SIM800L GPRS (2.5G) | SIM7600E LTE (4G) | Higher bandwidth, modern networks |
| Cost estimate | < USD 80 | ≈ USD 120–140 | Higher due to added components |

The comparison confirms that the present system achieves improvements across every measured dimension, at a cost increase of approximately USD 40–60 over Salman et al.'s [17] reported USD 80 system cost. The most significant improvements are: the 860-fold reduction in dashboard update latency; the extension of reliable wireless control range from ~10 m to ~100–200 m with failsafe; the addition of quantitative PPM gas calibration; and the platform-independent browser dashboard replacing the Android-only app.

The cost increase is modest relative to the capability improvements and remains well within the SR-10 constraint of USD 150. The additional expenditure is attributable to the SIM7600E LTE module (approximately USD 25 more than the SIM800L), the KY-032 IR sensor (approximately USD 2), the SG90 servo for the HC-SR04 mount (approximately USD 3), and the CC1101 wireless module (approximately USD 4).

---

## 5.4 Discussion of Research Questions

### 5.4.1 RQ-1: Dual-Processor Architecture Partitioning

*"How can an ARM-based dual-processor architecture be effectively partitioned to separate real-time motor control and sensor acquisition from high-level navigation logic and network communication?"*

The results confirm that the partition is effective and well-balanced. The Arduino Mega successfully executes all real-time tasks (motor PWM, GPS NMEA parsing at 1 Hz, ultrasonic ranging, wireless packet polling, I2C slave ISR) within a deterministic superloop cycle of approximately 2–5 ms per iteration. The Raspberry Pi successfully executes all high-level tasks (navigation mathematics at 10 Hz, REST API calls at 0.2 Hz, WebSocket event emission, MJPEG frame encoding at 10 FPS, database writes) within the headroom available at 18.7% mean CPU utilisation. The I2C communication overhead of < 1.3 ms per motor command introduces no measurable degradation to navigation performance at the 10 Hz control rate.

### 5.4.2 RQ-2: Gas Sensor Calibration Methodology

*"What calibration methodology produces accurate, quantitative PPM readings from low-cost MQ-series gas sensors, and what is the achievable accuracy relative to datasheet specifications?"*

The datasheet log-log power-law model, implemented in the `GasCalibration` class, was shown to produce physically plausible and consistent PPM values. The CO₂ reading of ~420 PPM matching the known atmospheric concentration provides strong empirical support for the MQ-135 calibration. However, the accuracy relative to a certified reference instrument (e.g., a non-dispersive infrared (NDIR) CO₂ analyser) was not formally quantified due to the absence of calibrated reference gas equipment at the time of testing. This is the primary limitation of the sensor calibration validation and represents an important direction for future work. The methodology is sound and reproducible; the limiting factor is the availability of reference equipment for absolute accuracy quantification.

### 5.4.3 RQ-3: Servo-Scan Dual-Sensor Obstacle Avoidance

*"How does a servo-scanned dual-sensor obstacle avoidance strategy compare in performance and reliability to single-sensor approaches?"*

The 97% success rate of the servo-scan avoidance (35 trials) compares favourably to the fixed-mount single-sensor approach described by Salman et al. [17], which provides no directional intelligence. While a precise head-to-head comparison under identical conditions was not conducted (as both systems were not available simultaneously), the qualitative improvement is clear: a fixed-mount sensor can only detect the presence of an obstacle, requiring a predetermined default turn direction (typically right) that will fail whenever the obstacle extends to that side. The servo-scan system selects the optimal direction based on measured clearance, which eliminates this class of failure mode entirely. The single failure recorded in testing (due to near-equal clearances) is an edge case that does not occur with a fixed-mount sensor's predetermined turn strategy, representing a marginal case where the deterministic fixed-mount approach could outperform the sensor-scan approach — though the practical significance of this edge case is low.

The KY-032 IR sensor's 3.8 µs response time provides a qualitatively different capability compared to the HC-SR04's ~30 ms polling cycle: immediate hardware-level interrupt response regardless of the software state machine's current position. This is important in the case where the robot's navigation controller is in the middle of a computation-intensive state transition and the polling cycle is delayed. The dual-sensor approach is therefore more robust than a single-sensor approach in software timing edge cases.

### 5.4.4 RQ-4: Dashboard Latency and Acceptability

*"What are the latency and reliability characteristics of a Flask/SocketIO dashboard, and are these acceptable for real-time monitoring?"*

The 17.4 ms mean end-to-end latency is well within the human perceptual threshold for real-time responsiveness (generally considered < 100 ms for interactive systems). The reliability of 280 received dashboard updates out of 280 transmitted (0% loss) in the end-to-end test confirms that the HTTP → WebSocket pipeline is lossless under normal operating conditions over a local Wi-Fi network. These results confirm that the Flask/SocketIO architecture is entirely acceptable for real-time environmental monitoring at the sensor update rates used in this project (0.2 Hz for sensor data, 0.5 Hz for GPS). At higher update rates (e.g., 1 Hz sensor updates), the architecture would need to be profiled for potential bottlenecks, but the current headroom at 18.7% CPU utilisation and < 660 Kbps network bandwidth suggests substantial room to increase data rates if required.

### 5.4.5 RQ-5: Dual-Redundant Wireless Reliability

*"How effectively does a dual-redundant wireless communication system maintain robot controllability under varied environmental conditions?"*

The range tests (Tables 4.18 and 4.19) and failsafe validation (Table 4.20) confirm that the dual-redundant system maintains controllability under the conditions tested (open outdoor environment, clear line of sight). The 530 ms maximum failsafe transition time and the 98 ms recovery time from FAILSAFE to wireless control are both operationally acceptable. However, it must be acknowledged that the tests were conducted in an open outdoor area without multipath fading, urban interference, or significant foliage. In real-world deployments in cluttered or built-up environments, both the ZigBee and CC1101 ranges would be expected to be shorter than the free-space values reported in Table 4.18 and Table 4.19. Future testing should include environments with obstructions, interference, and multipath propagation to establish more conservative operational range estimates.

---

## 5.5 System Limitations

Despite the successful achievement of all six research objectives, the current implementation has several limitations that must be acknowledged:

**1. GPS Accuracy Ceiling:** The 2.3 m CEP of the NEO-6M receiver is the dominant source of navigation error and fundamentally limits the minimum achievable waypoint arrival accuracy. For applications requiring metre-level or sub-metre accuracy (e.g., precise environmental sampling at specific survey points), the current GPS solution is inadequate without augmentation through RTK (Real-Time Kinematic) GNSS corrections or equivalent techniques.

**2. Gas Sensor Absolute Accuracy:** As discussed in Section 5.4.2, the MQ-series sensors' absolute PPM accuracy could not be quantified against certified reference instruments. Metal oxide sensors are known to exhibit cross-sensitivity (MQ-135 responds to multiple gases simultaneously), temperature and humidity dependence (sensor resistance changes with ambient temperature independently of gas concentration), and long-term drift. In the current implementation, no temperature-humidity compensation is applied to the gas sensor readings, which may introduce systematic errors in environments with significantly different temperature or humidity from the calibration baseline.

**3. Battery Life and Power Management:** The current system has no battery monitoring beyond a single battery percentage reading from the Arduino Mega. No power management strategy (e.g., sensor sleep modes, GPS duty cycling, adaptive camera frame rate reduction at low battery) is implemented. The operational endurance of the system on a single battery charge was not characterised as part of this project.

**4. Navigation in Dynamic Environments:** The reactive obstacle avoidance strategy has no memory of previously detected obstacles. If a robot repeatedly encounters the same obstacle from the same direction (e.g., a wall running parallel to the navigation path), it will execute the same avoidance manoeuvre on each encounter, potentially entering a loop. A simple obstacle map or hysteresis mechanism would address this.

**5. Indoor Operation:** The GPS-based navigation system is unusable indoors, where satellite signals do not penetrate. The compass is also affected by ferromagnetic building structures. Indoor navigation would require an entirely different localisation approach (e.g., UWB ranging, visual odometry, or Wi-Fi fingerprinting), which is outside the scope of the current design.

**6. SIM7600E UART Interface:** As noted in the project documentation, the SIM7600E UART interface has not been fully validated due to intermittent connection issues; USB is used as the primary interface. This means the UART pins of the Raspberry Pi are not utilised for this module, potentially leaving a communication resource unnecessarily unused.

**7. Single-Point of Failure — Raspberry Pi:** If the Raspberry Pi software crashes or the Linux OS encounters a fault, all high-level control (navigation, sensor acquisition, dashboard communication) is lost simultaneously. The Arduino Mega's `STATE_FAILSAFE` provides motor safety, but the robot becomes entirely inactive. A hardware watchdog timer (the Raspberry Pi's BCM2837 SoC includes a hardware watchdog accessible via `/dev/watchdog`) could be used to automatically reboot the Raspberry Pi software on crash.

---

## 5.6 Comparison with Other Reviewed Systems

Beyond the direct comparison with Salman et al. [17] in Table 5.1, it is instructive to compare the present system against the broader set of systems reviewed in Chapter 2.

**vs. Mois et al. [5] (CPS for environmental monitoring):** Mois et al.'s cyber-physical system was a static node network with no mobility. The present system's mobile platform enables it to actively survey a spatial area rather than passively monitoring a fixed point — a fundamentally different and more capable sensing paradigm for large-area environmental assessment.

**vs. Jiang and Huacón [7] (CEMSD):** The CEMSD monitored air quality, noise, temperature, and humidity, transmitting data via Wi-Fi or cellular. It did not include autonomous mobility or GPS positioning. The present system adds these capabilities while delivering data to a more capable real-time visualisation platform.

**vs. Tian and Geng [15] (household security robot):** Tian and Geng's system was designed for indoor household use over a wireless mesh network, without GPS navigation or outdoor deployment capability. The present system is designed specifically for outdoor operation with GPS-guided autonomous navigation.

**vs. Adrian and Repole [16] (AMBOA):** The AMBOA system featured multi-sensor fusion for outdoor environmental monitoring but was acknowledged to be cost-prohibitive for widespread adoption. The present system achieves comparable sensing capability at an estimated cost of USD 120–140, demonstrating the democratising potential of ARM-based platforms for environmental robotics research.

The consistent thread across these comparisons is that the present system represents a more complete integration of environmental sensing, autonomous mobility, GPS navigation, and accessible real-time visualisation than any individual system reviewed in the literature, at a competitive cost. The gap identified in Section 2.10 of the Literature Review — the absence of a system combining all these capabilities in a single open-source, reproducible platform — has been addressed.

---

## 5.7 Implications for Environmental Monitoring Practice

The results of this project have several practical implications for the design of environmental monitoring systems:

**ARM Dual-Processor Architecture as a Standard Pattern:** The Raspberry Pi + Arduino Mega architecture has been validated in this project and by Salman et al. [17] as an effective, cost-accessible template for autonomous environmental monitoring robots. The clean separation of concerns between the two processors, and the reliability of I2C inter-processor communication (zero failures in testing), supports the adoption of this pattern as a standard architectural template for similar projects in resource-constrained research settings.

**Web Dashboards over Cloud IoT Platforms for Real-Time Applications:** The 860-fold latency improvement of the local Flask/SocketIO dashboard over ThingSpeak demonstrates that, for applications where real-time situational awareness is important, local web dashboards are strongly preferable to cloud IoT platforms that impose minimum update intervals. Cloud platforms remain appropriate for long-term data archival, multi-site aggregation, and access from arbitrary internet locations; the present project's architecture supports both simultaneously through the optional ThingSpeak upload module.

**PPM Calibration as a Baseline Expectation:** The validation of the log-log calibration model against the known atmospheric CO₂ baseline suggests that reporting raw ADC values from MQ-series sensors, as done by Salman et al. [17] and most comparable systems, understates the capability of the hardware. The PPM calibration adds negligible computational overhead (a few floating-point operations per sensor per reading) and dramatically increases the scientific interpretability of the data. Future deployments of MQ-series sensors in research contexts should incorporate this calibration as a baseline expectation.

**Dual-Redundant Wireless as a Safety Requirement:** The failsafe behaviour demonstrated in Section 4.7.3 — motor halt within 530 ms of total communication loss — addresses a safety requirement that is largely absent from the published literature. For outdoor deployments where a robot may encounter members of the public or animals, deterministic safe stopping on communication loss is a practical safety necessity. The dual-redundant wireless architecture, with CC1101 providing approximately twice the range of ZigBee as a secondary channel, represents a readily implementable pattern for addressing this requirement.

---

## 5.8 Summary

This chapter has interpreted the results of Chapter 4 in the context of the research objectives, questions, and the relevant literature. All six research objectives were fully achieved, with quantitative evidence presented for each. The most significant contributions are: the 860-fold improvement in dashboard update latency over the ThingSpeak cloud approach; the implementation of quantitative PPM calibration validated against a known atmospheric baseline; the extension of reliable wireless control range from ~10 m to ~200 m with dual-redundancy and failsafe; and the complete integration of sensing, navigation, camera streaming, and real-time dashboard in a single open-source platform. The system's limitations — GPS accuracy ceiling, lack of absolute gas sensor validation, absence of power management — are clearly identified as directions for future work. Chapter 6 provides the concluding summary of the research findings, confirms objective achievement, and presents formal recommendations for future development.

---

*End of Chapter 5*
