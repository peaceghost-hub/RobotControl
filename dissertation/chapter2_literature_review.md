# CHAPTER 2: LITERATURE REVIEW

---

## 2.1 Introduction

This chapter presents a comprehensive review of the existing body of literature relevant to the design and implementation of an ARM-based autonomous environmental monitoring robot. The review is organised thematically, beginning with the broader context of environmental monitoring before narrowing progressively to the specific technologies and techniques employed in this project. The chapter examines prior work in mobile environmental monitoring robotics, ARM-based embedded systems, environmental sensing and calibration, GPS-based autonomous navigation, obstacle detection and avoidance, Internet of Things (IoT) platforms, wireless communication protocols, and web-based data visualisation. Each section identifies strengths and limitations of existing approaches, culminating in Section 2.10, which explicitly identifies the research gaps that this project addresses.

---

## 2.2 Environmental Monitoring Systems

### 2.2.1 Overview and Importance

Environmental monitoring is broadly defined as the systematic and repeated measurement of environmental parameters with the purpose of understanding the current state of the environment, detecting changes over time, and generating data that informs scientific analysis and public policy [1]. The parameters of greatest concern in urban and industrial contexts include temperature, relative humidity, carbon monoxide (CO), carbon dioxide (CO₂), particulate matter, ammonia, and a range of volatile organic compounds (VOCs). Prolonged human exposure to elevated concentrations of these pollutants is associated with serious respiratory, cardiovascular, and neurological health effects, making accurate, timely monitoring a public health imperative [11].

Historically, environmental monitoring has been conducted either through laboratory analysis of collected samples or through fixed-station sensor networks. Fixed-station networks, while capable of high-precision measurements, are inherently limited to their installation locations and are unsuitable for surveying spatially heterogeneous environments such as industrial sites, disaster zones, or ecologically sensitive areas where pollutant concentrations may vary significantly over short distances [2]. Laboratory analysis, while accurate, introduces delays that render it unsuitable for real-time hazard detection. These limitations established the motivation for mobile, robot-mounted monitoring platforms.

### 2.2.2 Static Sensor Networks and Their Limitations

Mois, Sanislav, and Folea [5] proposed a cyber-physical system (CPS) for monitoring ambient conditions in indoor spaces using Raspberry Pi-based nodes communicating over a short-range wireless network. The system demonstrated reliable data collection and real-time cloud synchronisation, but, as a static network, could not be repositioned without physical intervention. The authors acknowledged that spatial coverage was the primary limitation of the architecture. Shete and Agrawal [6] similarly presented an IoT-based urban climate monitoring system using a Raspberry Pi, collecting data from temperature, humidity, and gas sensors and uploading it to a cloud platform. While effective for fixed-point monitoring, the authors noted the absence of particulate matter sensing as a gap that left the system incomplete for comprehensive environmental assessment.

Jiang and Huacón [7] developed the Cloud-based Environment Monitoring Smart Device (CEMSD), which monitored air quality, noise, temperature, and humidity, transmitting data to a cloud server via either Wi-Fi or a cellular network. The CEMSD addressed the connectivity limitation of purely Wi-Fi-based systems by incorporating cellular fallback. However, the device remained static, and no provision was made for autonomous repositioning to track evolving pollution events.

Zhi et al. [14] proposed an intelligent indoor air quality control system using remote monitoring, targeting building environment management. Whilst demonstrating effective data collection and cloud integration, their work was confined to indoor, controlled environments and did not address the challenges of outdoor, mobile deployment. These works collectively demonstrate the maturity of static IoT monitoring while highlighting the need for mobile alternatives.

### 2.2.3 Mobile Environmental Monitoring Robots

Trincavelli et al. [1] provided an early and influential study of the use of mobile robots for environmental monitoring, focusing particularly on gas source localisation in outdoor environments. They identified mobile robots as uniquely capable of actively seeking pollution sources rather than passively waiting for pollutants to diffuse to fixed sensors, and demonstrated the feasibility of equipping a standard mobile robot platform with an array of metal oxide gas sensors. Dunbabin and Marques [2] subsequently provided a comprehensive review of robotic environmental monitoring, covering aerial, ground, and aquatic platforms across domains including ecology, oceanography, and pollution monitoring. They concluded that mobile robots, by virtue of their ability to operate in hazardous environments without endangering human operators, represent a transformative technology for environmental science.

Tian and Geng [15] explored the use of a household security robot equipped with environmental sensors communicating over a wireless mesh network. While their work demonstrated the practical integration of sensing and mobility, the robot was designed for indoor household use and lacked GPS navigation or cloud connectivity. Adrian and Repole [16] presented the AMBOA robot sensory system for intelligent autonomous environmental monitoring, demonstrating multi-sensor fusion for outdoor environments. However, the system relied on a proprietary and cost-prohibitive hardware platform, limiting its reproducibility and widespread adoption.

The seminal paper most directly relevant to the present project is that of Salman, Rahman, Tarek, and Wang [17], who proposed a GPS-controlled environment monitoring robotic system based on IoT and ARM. Their system combined a Raspberry Pi 3 Model B for environmental sensing and IoT data upload with an Arduino Mega 2560 for navigation and motor control, mirroring the dual-processor architecture adopted in the present project. The system used DHT11, MQ-135, and MQ-7 sensors; a NEO-6M GPS module; and an HMC5883L compass for navigation, with an HC-06 Bluetooth module providing the control link to an Android app. Data were uploaded to ThingSpeak every 15 seconds. Salman et al.'s [17] work provides the most direct point of comparison for the present project and is cited extensively throughout this dissertation. Whilst it successfully validated the ARM-based dual-processor concept, it was limited by the short range of Bluetooth communication, the absence of a web-accessible dashboard, single-sensor obstacle avoidance, and a lack of quantitative gas sensor calibration yielding PPM values.

---

## 2.3 ARM-Based Embedded Systems in Robotics

### 2.3.1 The ARM Architecture

Advanced RISC Machine (ARM) processors are a family of reduced-instruction-set computing (RISC) processor architectures characterised by high computational performance per unit of power consumption, compact silicon area, and efficient interrupt response — properties that make them ideally suited to embedded and battery-powered applications [3]. As Liu [3] noted in an analysis of ARM-based wireless communication terminals, the ARM architecture's combination of high-cost performance, low power consumption, and rich peripheral support makes it the most effective choice for embedded systems that must execute complex concurrent tasks. The ubiquity of ARM cores across smartphones, IoT devices, single-board computers, and microcontrollers has created a rich ecosystem of development tools, libraries, and community support that substantially reduces development time compared to more exotic architectures.

### 2.3.2 The Raspberry Pi 3 Model B as a Robotics Platform

The Raspberry Pi 3 Model B (RPi 3B) is an ARM-based single-board computer featuring a Broadcom BCM2837 64-bit ARMv8 quad-core processor running at 1.2 GHz, 1 GB of LPDDR2 RAM, and 40 GPIO pins supporting digital I/O, UART, I2C, and SPI protocols [17]. It also integrates 802.11b/g/n Wi-Fi and Bluetooth 4.1, which substantially simplifies local network connectivity. Grimmett [8] provided a comprehensive treatment of the Raspberry Pi as a robotics platform, covering motor control, sensor integration, camera streaming, and network communication in detail. He concluded that the Raspberry Pi's combination of Linux-based operating system support, Python programmability, and hardware peripheral access makes it uniquely capable among single-board computers for complex robotic applications that require both real-time data acquisition and high-level computation.

The RPi's general-purpose input/output architecture, however, has an important limitation for environmental sensing: it supports only digital inputs and does not natively sample analogue voltages [17]. Metal oxide gas sensors such as the MQ series produce analogue output voltages proportional to gas concentration, requiring an external analogue-to-digital converter (ADC) accessible over a digital interface such as I2C. This limitation is addressed in the literature [17] and in the present project through the use of the Texas Instruments ADS1115 16-bit ADC, connected via the RPi's I2C bus, which provides four differential or single-ended analogue input channels with programmable gain amplification.

### 2.3.3 Arduino Microcontrollers in Robotics

The Arduino Mega 2560 is a microcontroller board based on the ATmega2560, offering 54 digital I/O pins, 16 analogue input pins, 4 hardware UARTs, and hardware I2C and SPI support [9]. Schmidt [9] described the Arduino platform as an ideal option for rapid prototyping in robotics due to its simple development environment, extensive library ecosystem, and low cost. Unlike the Raspberry Pi, which runs a full Linux operating system, the Arduino executes a single bare-metal program in a deterministic loop, making it well-suited to real-time tasks such as motor PWM generation, serial sensor polling, and immediate interrupt response — tasks that are difficult to guarantee on a preemptive multitasking OS such as Raspbian.

The architectural complementarity of the Raspberry Pi and Arduino Mega has been recognised in the literature as a powerful basis for robotic systems: the Raspberry Pi handles computationally intensive tasks such as navigation mathematics, network communication, and image processing, while the Arduino handles real-time motor control and hardware interfacing [17]. Inter-processor communication between the two boards is typically implemented over the I2C bus, as demonstrated by Salman et al. [17] and in the present project, exploiting the well-established master-slave protocol defined in the I2C specification [12].

---

## 2.4 Environmental Sensors

### 2.4.1 Temperature and Humidity Sensing

The DHT11 is a widely-used, cost-effective digital temperature and humidity sensor that uses a capacitive humidity sensing element and a thermistor to measure ambient conditions, outputting data over a single-wire serial protocol [10]. It measures temperature in the range 0–50°C with ±2°C accuracy and relative humidity in the range 20–80% RH with ±5% RH accuracy, making it suitable for general environmental monitoring applications where high precision is not required [10]. Both Salman et al. [17] and the present project employ the DHT11 for ambient temperature and humidity monitoring, interfaced to the Raspberry Pi via GPIO using the Adafruit DHT Python library.

More accurate alternatives include the DHT22 (also known as the AM2302), which extends the temperature range to -40–80°C with ±0.5°C accuracy and humidity to 0–100% RH with ±2–5% accuracy. The present system's sensor driver is written to support DHT11, DHT22, and AM2302 interchangeably through a configuration parameter, providing a straightforward upgrade path without software modification.

### 2.4.2 Metal Oxide Gas Sensors

Metal oxide semiconductor (MOS) gas sensors, commonly known by their MQ series designation, operate on the principle of changes in electrical resistance caused by gas adsorption on a heated metal oxide surface [17]. The sensor element, typically tin dioxide (SnO₂), exhibits a decrease in resistance when target gas molecules adsorb onto its surface and donate electrons, altering the surface conductance. This change in resistance produces a proportional change in the output voltage across a fixed load resistor, enabling gas concentration measurement.

The MQ-7 sensor is specifically designed to detect carbon monoxide (CO), with a measurable range of 20–2000 ppm, characterised by high sensitivity and relatively fast response time [17]. CO is a colourless, odourless, and highly toxic gas produced by incomplete combustion of hydrocarbons, making its detection critical in environments with combustion sources. The MQ-135 sensor is sensitive to a range of gases including ammonia (NH₃), carbon dioxide (CO₂), benzene, and NOx compounds, making it suitable as a general air quality indicator [17]. The MQ-2 sensor, included in the present project but absent from Salman et al.'s [17] design, detects flammable gases including LPG, propane, methane, hydrogen, and smoke, extending the system's hazard detection capability.

Tapashetti, Vegiraju, and Ogunfunmi [11] demonstrated an IoT-enabled, low-cost air quality monitoring device incorporating MQ-series sensors, concluding that while MOS sensors are subject to drift, cross-sensitivity, and temperature-humidity dependence, they are adequate for relative concentration monitoring and threshold-based alerting applications when properly calibrated. Their work underscores the importance of calibration, which is addressed in Section 2.4.3.

### 2.4.3 Gas Sensor Calibration and PPM Conversion

A significant limitation of MQ-series sensors as used in the existing literature, including Salman et al. [17], is that raw ADC values or unprocessed voltage readings are reported rather than calibrated concentration values in parts per million (PPM). This renders the data difficult to interpret in absolute terms or to compare across different deployments. Quantitative PPM conversion requires knowledge of the sensor's characteristic resistance ratio (Rs/R₀) curve, where Rs is the sensor resistance under measurement conditions and R₀ is the sensor resistance in clean air — a calibration baseline.

Sensor datasheets provide these characteristic curves on logarithmic axes, allowing the relationship between Rs/R₀ and PPM concentration to be modelled as a power law:

$$\text{PPM} = a \cdot \left(\frac{R_s}{R_0}\right)^b$$

where *a* and *b* are empirically determined constants extracted from the datasheet curves using log-log linear regression [11]. This calibration approach, sometimes termed the "datasheet log-log method," has been validated in the academic literature for MQ-series sensors and yields measurements accurate to within the intrinsic limitations of the sensor hardware. The present project implements this calibration methodology in a dedicated Python module (`gas_calibration.py`), computing PPM values for MQ-2 (smoke/LPG), MQ-135 (CO₂/NH₃), and MQ-7 (CO) in real time, which represents a significant improvement over the raw-value reporting of Salman et al. [17].

---

## 2.5 GPS-Based Autonomous Navigation

### 2.5.1 GPS Technology in Mobile Robotics

The Global Positioning System (GPS) is a satellite-based radionavigation system that provides geographic position and time information to a receiver anywhere on Earth with line-of-sight to four or more GPS satellites [17]. In mobile robotics, GPS provides the absolute position reference required for waypoint navigation, enabling the robot to determine its current location and compute the direction and distance to a target waypoint. The NEO-6M module, manufactured by u-blox, is the GPS receiver used in both Salman et al.'s [17] system and the present project. It supports the NMEA 0183 data protocol over a serial UART interface and can acquire a position fix with typically 4–8 satellites under open sky conditions, providing a horizontal accuracy of 2.5 metres CEP (circular error probable) under typical conditions.

A key challenge in GPS-based navigation is the "last metre" problem: as the robot approaches a waypoint, GPS accuracy (2.5 m CEP) may be comparable to or larger than the remaining distance to the target, leading to oscillation or failure to definitively reach the waypoint. Both Salman et al. [17] and the present project address this by defining a waypoint acceptance radius — the distance within which the robot is considered to have "reached" the waypoint and proceeds to the next. The present project uses a 3.0-metre radius, consistent with the horizontal accuracy of the NEO-6M receiver.

The present project also integrates a second GPS source — the SIM7600E LTE module's integrated GPS receiver — providing redundancy and enabling cross-validation of position data. This dual-GPS architecture is not present in Salman et al.'s [17] system and represents a meaningful reliability enhancement for outdoor deployments.

### 2.5.2 Geodesic Calculations for Waypoint Navigation

Navigation between GPS waypoints requires the computation of two key geometric quantities: the great-circle distance between the current position and the target waypoint, and the forward azimuth (bearing) from the current position to the target. The Haversine formula is the most commonly cited algorithm for computing great-circle distances on a spherical Earth model:

$$d = 2r \arcsin\!\left(\sqrt{\sin^2\!\left(\frac{\phi_2-\phi_1}{2}\right) + \cos\phi_1\cos\phi_2\sin^2\!\left(\frac{\lambda_2-\lambda_1}{2}\right)}\right)$$

where $\phi$ denotes latitude, $\lambda$ denotes longitude, $r$ is the mean Earth radius (6,371 km), and $d$ is the great-circle distance. The corresponding forward azimuth is computed as:

$$\theta = \arctan2\!\left(\sin(\Delta\lambda)\cos\phi_2,\; \cos\phi_1\sin\phi_2 - \sin\phi_1\cos\phi_2\cos(\Delta\lambda)\right)$$

These formulas, used in the present project's navigation controller, assume a spherical Earth model and introduce errors of up to 0.5% over short distances due to the Earth's actual oblate spheroid shape [17]. For higher precision, the present project additionally implements the Karney algorithm for geodesic calculations on the WGS-84 ellipsoid via the GeographicLib library, which reduces positional errors to sub-millimetre levels over any distance — a level of precision well in excess of the GPS receiver's own accuracy but ensuring the navigation mathematics itself is not the limiting factor in system performance.

### 2.5.3 Compass-Based Heading Correction

GPS alone cannot provide the robot's current heading at low speeds or when stationary, as heading is derived from the change in position between consecutive GPS fixes and becomes unreliable below approximately 0.5 m/s. A magnetometer (compass) is therefore used to provide an instantaneous heading reference. Salman et al. [17] used the HMC5883L triple-axis magnetometer connected to the Arduino Mega's hardware I2C bus, noting the importance of physically isolating the compass from ferromagnetic components to avoid magnetic interference. The present project uses the compatible QMC5883L sensor on a dedicated software I2C bus (bit-banging on Arduino digital pins 40 and 41), further isolating the compass measurements from potential electromagnetic interference generated by the motor driver and power supply on the hardware I2C bus.

The navigation algorithm uses compass heading to compute the angular error between the robot's current direction of travel and the bearing to the target waypoint, then adjusts the differential drive motor speeds to turn the robot toward the target. A heading deadband of 5° is implemented to prevent over-correction and oscillatory behaviour at small angular errors.

---

## 2.6 Obstacle Detection and Avoidance

### 2.6.1 Ultrasonic Distance Sensing

The HC-SR04 is an ultrasonic ranging module that measures distance by timing the round-trip travel time of a 40 kHz ultrasonic pulse between the module and a reflective surface. It provides a measurement range of 2 cm to 4 m with approximately 3 mm resolution and is the most commonly used obstacle sensor in low-cost robot designs [17]. Salman et al. [17] mounted the HC-SR04 on the front of the robot chassis at a fixed angle, triggering an obstacle avoidance manoeuvre when the measured distance fell below a 30 cm threshold. This fixed-angle single-sensor approach has a well-documented limitation: it can only detect obstacles within its fixed field of view (approximately 15° cone), and cannot determine whether the obstacle extends to one side, potentially permitting a directional avoidance manoeuvre without requiring a full reversal.

The present project addresses this limitation by mounting the HC-SR04 on a 180°-travel SG90 servo motor. When an obstacle is detected at the forward-facing angle, the servo rotates to scan left (160°), centre (90°), and right (20°) positions, measuring the clearance distance in each direction before selecting the direction of greatest clearance for the avoidance manoeuvre. This servo-scan approach, sometimes called "3-direction ultrasonic scanning," significantly improves avoidance decision quality in environments with obstacles on multiple sides.

### 2.6.2 Infrared Proximity Sensing

The KY-032 is a digital and analogue infrared (IR) obstacle proximity sensor operating by emitting infrared light and detecting reflection from nearby objects. Unlike the HC-SR04, which measures a precise distance, the KY-032 produces a binary obstacle/no-obstacle output (with a tunable threshold potentiometer) and an analogue proximity signal, detecting obstacles within a range of approximately 2–40 cm. Its primary advantage is response latency: a digital pin interrupt can trigger an immediate motor stop within microseconds of obstacle detection, compared to the ~30 ms measurement cycle of the HC-SR04.

The present project combines both sensors in a complementary architecture: the KY-032 provides fast, low-latency immediate-stop triggering for very close obstacles, while the HC-SR04 with servo scanning provides directional distance information to guide the avoidance manoeuvre. This dual-sensor fusion approach is not present in Salman et al.'s [17] design and represents an improvement in both safety and avoidance intelligence.

### 2.6.3 Reactive vs. Deliberative Avoidance Strategies

Obstacle avoidance strategies in mobile robotics are broadly classified as reactive (also known as behaviours-based) or deliberative (map-based). Reactive approaches, as employed in the present project and in Salman et al. [17], make avoidance decisions based solely on immediate sensor readings without maintaining a map of the environment. While this limits global path optimality, reactive strategies are well-suited to resource-constrained embedded systems and environments where the obstacle field is unknown or dynamic. Deliberative approaches, such as A* or Dijkstra path planning on an occupancy grid, require substantial memory and computational resources beyond the scope of the Arduino Mega's 8 KB SRAM and are not explored in the present project. The behaviour control algorithm of Salman et al. [17] — turn left or right based on shortest-radius correction when heading deviation exceeds 15° — forms the conceptual basis from which the present project's more sophisticated servo-scan avoidance strategy was derived.

---

## 2.7 Internet of Things (IoT) Platforms and Data Management

### 2.7.1 IoT Platforms for Environmental Data

The Internet of Things paradigm enables physical devices to collect, transmit, and share data over internet infrastructure, creating opportunities for large-scale, distributed environmental monitoring without the need for dedicated proprietary networks [11]. Cloud IoT platforms provide storage, processing, and visualisation services that can be accessed from any internet-connected device, enabling operators to monitor environmental conditions from remote locations in real time [5].

Salman et al. [17] used ThingSpeak, an open-source IoT analytics platform maintained by MathWorks [4], as the cloud backend for their system. ThingSpeak supports up to eight data fields per channel, with a minimum update interval of 15 seconds on the free tier, and provides real-time visualisation through MATLAB-based analysis tools. It can also trigger actions based on data thresholds, enabling automated alerting. Tapashetti et al. [11] similarly used ThingSpeak in their low-cost air quality monitoring device, validating its suitability for this class of application.

### 2.7.2 Local Web Dashboards vs. Cloud Platforms

While cloud IoT platforms provide global accessibility, they introduce latency, require persistent internet connectivity, and place data on third-party infrastructure — considerations that may be unacceptable in privacy-sensitive or bandwidth-constrained deployments. An alternative approach, adopted in the present project, is to host a local web dashboard on the robot's control network, served by a Flask/SocketIO Python web application running on the Raspberry Pi's companion computer. This approach provides sub-second real-time updates via WebSocket (not limited by the ThingSpeak 15-second minimum interval), full control over data retention and schema, and operation without any internet connectivity. Cloud upload to ThingSpeak is implemented as an optional, configurable feature alongside the local dashboard, providing both approaches simultaneously.

### 2.7.3 Database Design for Sensor Data

The present project implements a SQLite relational database with a SQLAlchemy ORM layer to persistently store all sensor readings, GPS positions, waypoints, robot status records, event logs, and command queues. This structured approach enables historical trend analysis, query-based data export, and configurable data retention policies (30 days for sensor and GPS data; 90 days for event logs). The use of a local database also provides offline buffering: sensor data collected while cellular connectivity is unavailable is retained locally and can be synchronised with cloud platforms upon reconnection — a capability not described in Salman et al.'s [17] system.

---

## 2.8 Wireless Communication Protocols

### 2.8.1 I2C for Inter-Processor Communication

The Inter-Integrated Circuit (I2C) protocol, developed by Philips Semiconductors (now NXP), is a two-wire synchronous serial bus protocol supporting multiple masters and multiple slaves on the same bus using 7-bit addressing [12]. It is widely used for connecting microcontrollers to sensors, ADCs, and other peripherals requiring bidirectional communication at modest data rates (100 kbps standard, 400 kbps fast mode). Kumari and Gayathri [12] demonstrated the versatility of I2C for inter-device communication in embedded systems, confirming its suitability for reliable data exchange in robotics applications.

In both Salman et al.'s [17] system and the present project, I2C is used as the inter-processor communication bus between the Raspberry Pi (master) and the Arduino Mega (slave at address 0x08). The present project extends this with a comprehensive command set of 15 commands covering navigation control, GPS query, status query, manual override, emergency stop, and return-to-start, all encoded as single-byte command codes with variable-length payloads. A thread-safe locking mechanism prevents concurrent I2C access from the Raspberry Pi's multiple threads, and a bit-banging I2C recovery routine is implemented to restore normal bus operation following bus lockup events.

### 2.8.2 Short-Range Wireless Communication

Salman et al. [17] used an HC-06 Bluetooth module connected to the Arduino Mega to provide a Bluetooth Serial Port Profile (SPP) link to an Android app, enabling manual waypoint entry and navigation commands. Bluetooth Class 2, as used by the HC-06, provides a nominal range of 10 metres, which severely limits the operational radius of the system if manual intervention is required. As an alternative to Bluetooth, ZigBee — based on the IEEE 802.15.4 standard — provides a longer range (up to 100 metres line-of-sight with standard modules, up to 1.6 km with the XBee Pro series), lower power consumption, and support for mesh networking, making it more suitable for outdoor robot deployments [17].

The present project implements ZigBee as its primary wireless link, with CC1101 sub-GHz RF modules as the secondary channel, providing hardware-level wireless redundancy that is absent from comparable published systems. The Arduino firmware implements a priority system in which a wireless command received on either channel immediately overrides autonomous navigation, enabling the operator to take manual control at any time regardless of the robot's autonomous state.

### 2.8.3 Long-Range Cellular Communication

For truly remote deployments beyond the range of any local wireless protocol, cellular communication is required. Salman et al. [17] used the SIM800L GPRS module with the Raspberry Pi to provide internet connectivity for cloud data upload. GPRS (2.5G) provides data rates of approximately 56–114 kbps, adequate for the periodic sensor data upload use case. The present project upgrades this to the SIM7600E module, which supports 4G LTE Cat-4 with theoretical downlink speeds of 150 Mbps, in addition to providing an integrated GPS receiver as a secondary position source. This LTE upgrade ensures reliable cellular data connectivity in modern mobile networks where 2G/2.5G service has been discontinued in many regions, and enables real-time video streaming alongside sensor data upload.

---

## 2.9 Web-Based Real-Time Dashboards

A growing trend in IoT and robotics is the use of web-based dashboards as the primary operator interface, replacing dedicated mobile applications with browser-accessible web pages that can run on any device. Web dashboards built with Flask and JavaScript charting libraries have been demonstrated in multiple environmental monitoring contexts [5], [6], [7], with the main advantages being platform independence (any device with a browser can be an operator terminal), ease of deployment (no app installation required), and the ability to leverage mature web development ecosystems.

The WebSocket protocol, standardised in RFC 6455, enables full-duplex communication between a browser client and a web server over a persistent TCP connection, eliminating the latency of repeated HTTP polling and enabling true real-time data push to browser clients [5]. The Socket.IO library, used in the present project, wraps WebSocket with automatic fallback to HTTP long-polling for environments where WebSocket is not available, ensuring broad compatibility. Leaflet.js provides an open-source interactive mapping library that renders GPS position data on a zoomable map tile layer, enabling the operator to visually track the robot's position and set waypoints by clicking on the map. Chart.js provides responsive, animated real-time charts for sensor time-series data.

The combination of Flask, Flask-SocketIO, Leaflet.js, and Chart.js employed in the present project represents a modern, lightweight, and entirely open-source dashboard architecture that provides substantially richer functionality than the ThingSpeak visualisation used by Salman et al. [17], which is limited to eight fields and a 15-second update interval.

---

## 2.10 Identified Research Gaps

The review of the literature conducted in this chapter identifies the following specific gaps that the present project addresses:

1. **Limited communication range:** Salman et al. [17] and several comparable systems [15] use Bluetooth (≤10 m range) as the primary control link, severely limiting the operational radius. The present project replaces Bluetooth with ZigBee as the primary link and adds CC1101 as a secondary link, with automatic failsafe behaviour when both are lost.

2. **Absence of a web-accessible operator interface:** Existing systems [17] require a dedicated Android app, limiting operator terminals to Android devices and introducing dependency on mobile app development toolchains. The present project implements a browser-based dashboard accessible from any device, requiring only a web browser.

3. **Raw sensor values without quantitative calibration:** Published systems including Salman et al. [17] report raw ADC values or voltage readings from MQ-series sensors without converting them to calibrated PPM concentrations, limiting the interpretability and scientific value of the data. The present project implements a validated log-log calibration model producing quantitative PPM readings for all three gas sensors.

4. **Single-sensor, fixed-angle obstacle avoidance:** The fixed-mount, single-sensor ultrasonic avoidance approach of Salman et al. [17] cannot assess directional clearance, leading to potentially sub-optimal or failed avoidance manoeuvres. The present project adds servo-scan directional assessment and a second (IR) sensor for fast proximity triggering.

5. **No persistent local data storage:** Existing systems typically send data directly to a cloud platform without local persistence, meaning data is lost if the internet connection is interrupted. The present project implements a local SQLite database with configurable data retention and buffered cloud upload.

6. **No GPS redundancy:** Existing single-GPS designs have no fallback position source if the primary GPS loses fix. The present project integrates a second GPS receiver (SIM7600E integrated GPS) alongside the NEO-6M on the Arduino.

7. **Limited multi-sensor gas coverage:** Salman et al. [17] detected CO and general air quality (CO₂/NH₃). The present project adds MQ-2 for smoke and LPG detection, providing a more comprehensive hazardous gas detection suite.

---

## 2.11 Summary

This chapter has reviewed the existing literature across the seven thematic domains relevant to this project: environmental monitoring systems, ARM-based embedded platforms, environmental sensors and calibration, GPS navigation, obstacle avoidance, IoT data management, and wireless communication. The review established that the dual-processor ARM architecture (Raspberry Pi + Arduino Mega) pioneered by Salman et al. [17] provides a robust and cost-effective foundation for autonomous environmental monitoring robots, and identified seven specific gaps in the existing literature that the present project addresses through its design and implementation choices. Chapter 3 presents the methodology by which these gaps are addressed in the complete system design.

---

*End of Chapter 2*
