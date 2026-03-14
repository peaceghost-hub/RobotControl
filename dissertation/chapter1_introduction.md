# CHAPTER 1: INTRODUCTION

---

## 1.1 Introduction

This chapter presents the background, motivation, and objectives of this research project. It introduces the concept of ARM-based autonomous robots as platforms for environmental monitoring, establishes the problem being addressed, and defines the scope and boundaries of the study. The chapter further outlines the research questions and objectives that guided the design and implementation process, and concludes with a structural overview of the dissertation.

---

## 1.2 Background and Motivation

The natural environment is subject to increasing pressure from industrial activity, urbanisation, and human-generated pollutants, making real-time environmental monitoring a matter of growing scientific and public-health importance. Environmental monitoring is broadly defined as the systematic collection, analysis, and interpretation of data relating to the state of the environment, with the purpose of tracking changes, identifying hazards, and informing policy decisions [1]. Traditionally, such monitoring has relied on static sensor networks installed at fixed locations; however, fixed infrastructure is inherently limited in spatial coverage and is entirely unsuitable for deployment in environments that are hazardous to human health [2].

Robotic systems have emerged as a compelling solution to these limitations. A mobile robot equipped with environmental sensors can navigate to areas that are inaccessible or dangerous to human operators — such as sites with elevated concentrations of toxic gases, fire-affected zones, or post-disaster areas — and collect data continuously without exposing personnel to risk [2]. Salman et al. [3] demonstrated precisely this concept, proposing an autonomous GPS-guided robot that collects temperature, humidity, carbon monoxide, and air quality data and uploads it in real time to an Internet of Things (IoT) cloud platform. Their work established that a cost-effective ARM-based embedded system, combining a Raspberry Pi and an Arduino Mega, is capable of performing both environmental sensing and autonomous waypoint navigation within a single integrated platform.

The advent of low-cost, high-performance ARM-based single-board computers — particularly the Raspberry Pi family — has dramatically reduced the barrier to entry for sophisticated embedded robotics [4]. The Raspberry Pi 3 Model B, featuring a Broadcom BCM2837 64-bit ARMv8 quad-core processor running at 1.2 GHz with 1 GB of RAM and 40 GPIO pins, provides sufficient computational power and peripheral connectivity to serve as the central processing node of a complete robotic system [3]. Paired with a microcontroller such as the Arduino Mega 2560 — which offers 54 digital I/O pins, 16 analog inputs, and four hardware UART ports — the two-board architecture cleanly separates high-level processing and network communication from low-level real-time motor control and sensor interfacing [3].

Beyond simple data collection, modern environmental monitoring robots must be capable of communicating their data to remote users in real time. IoT platforms such as ThingSpeak enable cloud-based storage and visualisation of sensor data accessible from any internet-connected device [3]. The integration of cellular communication modules — such as the SIM7600E LTE modem — further ensures that the robot remains connected to the internet even in areas without Wi-Fi infrastructure, making truly remote, autonomous monitoring feasible. Together with a web-based dashboard running on the robot's own network, these communication layers provide operators with a comprehensive, real-time picture of environmental conditions at the robot's location.

The present project, titled *"ARM-Based Environmental Monitoring and Aide Robot"*, was motivated by these developments. It extends and enhances the foundational work of Salman et al. [3] by implementing a more sophisticated navigation architecture, a richer multi-sensor suite, a dedicated web dashboard with real-time visualisation, and a dual-redundant wireless communication system. The system is designed to operate autonomously, navigating GPS waypoints while continuously acquiring and transmitting environmental data, and to alert operators when hazardous conditions are detected.

---

## 1.3 Problem Statement

Manually monitoring environmental parameters in potentially hazardous locations presents a direct risk to human health. While static sensor networks address this to some extent, they cannot be repositioned as conditions change, and they offer no ability to survey a spatial area systematically. Existing mobile environmental monitoring systems, as reviewed in the literature, frequently address either autonomous navigation or environmental sensing in isolation; few integrate both capabilities into a single, cohesive, cost-effective system with a modern web-based operator interface [3], [5], [6]. Furthermore, many published prototypes rely on proprietary or expensive hardware platforms and short-range Bluetooth communication, limiting their practical applicability in outdoor, large-scale deployments [3].

There is therefore a clear need for a fully integrated, autonomous, ARM-based environmental monitoring robot that:

- navigates GPS-defined waypoints without human intervention;
- acquires real-time data from a multi-sensor suite covering temperature, humidity, and multiple gas species;
- communicates data to a persistent, remotely accessible web dashboard over both local Wi-Fi and cellular (LTE) networks;
- provides intelligent obstacle detection and avoidance to operate safely in unstructured environments;
- implements a dual-redundancy wireless architecture to maintain control even if one communication channel fails.

---

## 1.4 Scope and Delimitations

This project encompasses the complete design, implementation, and testing of the hardware and software components of the ARM-based environmental monitoring robot described above. The scope includes:

- Selection, integration, and calibration of all hardware components (Raspberry Pi 3B, Arduino Mega 2560, DHT11, MQ-2, MQ-135, MQ-7 gas sensors, NEO-6M GPS, QMC5883L compass, HC-SR04 ultrasonic sensor, KY-032 IR sensor, SIM7600E LTE module, Pi Camera V2, L298N motor driver);
- Development of the multi-threaded Python control software running on the Raspberry Pi;
- Development of the C++ Arduino firmware implementing the navigation state machine;
- Development of the Flask/SocketIO web dashboard with real-time data visualisation;
- Implementation and testing of the I2C inter-processor communication protocol;
- Implementation of a dual-redundant wireless communication layer (ZigBee/CC1101);
- Field testing of waypoint navigation and obstacle avoidance.

The following are explicitly outside the scope of this project:

- Development of a custom mobile application (a web browser interface is used instead);
- Deployment at scale or in actual hazardous industrial environments;
- Advanced machine-learning-based environmental analysis beyond the implemented threshold alerting;
- Solar power or energy harvesting subsystems.

---

## 1.5 Research Objectives

The primary objective of this research is to design, implement, and validate a cost-effective, autonomous, ARM-based mobile robot capable of real-time environmental monitoring and remote data transmission.

The following specific objectives were identified to achieve the primary objective:

1. **OBJ-1:** To design and implement a dual-processor hardware architecture using a Raspberry Pi 3 Model B as the high-level controller and an Arduino Mega 2560 as the real-time navigation processor.

2. **OBJ-2:** To integrate and calibrate a multi-sensor environmental monitoring suite comprising a DHT11 temperature and humidity sensor and MQ-2, MQ-135, and MQ-7 gas sensors, with quantitative PPM conversion using validated datasheet calibration curves.

3. **OBJ-3:** To implement a GPS waypoint navigation system on the Arduino Mega, incorporating compass-based heading correction and servo-scanned ultrasonic and IR dual-sensor obstacle avoidance.

4. **OBJ-4:** To develop a real-time web dashboard, accessible over both local area network and cellular internet, that displays live sensor readings, GPS position, camera feed, and historical trend graphs.

5. **OBJ-5:** To implement and test a dual-redundant wireless communication architecture that maintains reliable command and telemetry links between the operator and the robot.

6. **OBJ-6:** To validate the integrated system through functional testing of each subsystem and combined end-to-end testing of autonomous navigation and environmental monitoring.

---

## 1.6 Research Questions

In pursuit of the above objectives, this research addresses the following questions:

1. How can an ARM-based dual-processor architecture be effectively partitioned to separate real-time motor control and sensor acquisition from high-level navigation logic and network communication?

2. What calibration methodology produces accurate, quantitative PPM readings from low-cost MQ-series electrochemical gas sensors, and what is the achievable accuracy relative to datasheet specifications?

3. How does a servo-scanned dual-sensor obstacle avoidance strategy (ultrasonic + IR) compare in performance and reliability to single-sensor approaches in an unstructured outdoor environment?

4. What are the latency and reliability characteristics of a Flask/SocketIO web dashboard receiving sensor data over an I2C → HTTP → WebSocket pipeline, and are these acceptable for real-time monitoring?

5. How effectively does a dual-redundant wireless communication system (ZigBee primary, CC1101 secondary) maintain robot controllability under varied environmental conditions?

---

## 1.7 Significance of the Study

This project makes several contributions to the field of embedded robotics and environmental monitoring:

- It demonstrates a complete, reproducible, open-source implementation of an ARM-based environmental monitoring robot that integrates capabilities which, in the existing literature, are typically found in isolation [3], [5], [6], [7].
- It introduces a web-based operator interface that is more accessible than the Bluetooth-only Android app of Salman et al. [3], enabling monitoring from any device with a browser without the range limitations of Bluetooth.
- It implements a hierarchical wireless architecture with automatic failsafe behaviour — a safety feature not addressed in comparable published systems.
- It provides a quantitative gas sensor PPM calibration framework using logarithmic curve fitting, offering a replicable methodology for future low-cost air quality monitoring research.
- The project provides a practical proof-of-concept for the use of low-cost ARM platforms in autonomous environmental survey missions, contributing to the broader goal of making environmental monitoring accessible to institutions with limited financial resources.

---

## 1.8 Dissertation Structure

The remainder of this dissertation is organised as follows:

**Chapter 2 — Literature Review** provides a comprehensive review of existing work in the areas of environmental monitoring systems, autonomous mobile robots, ARM-based embedded systems, IoT platforms, and wireless communication for robotic applications. It situates the present work within the existing body of knowledge and identifies the gaps this project addresses.

**Chapter 3 — Methodology** presents the complete systems engineering approach taken in the design and implementation of the robot. It details the hardware architecture, circuit design, sensor selection and calibration, software design, navigation algorithm design, communication protocol design, and the testing methodology used to validate each subsystem.

**Chapter 4 — Results** presents the quantitative and qualitative results of all testing activities, including sensor accuracy measurements, navigation performance metrics, dashboard latency measurements, and wireless link reliability data.

**Chapter 5 — Discussion** interprets the results in the context of the research questions and objectives, compares the achieved performance to the work of Salman et al. [3] and other referenced systems, and discusses the limitations of the current implementation.

**Chapter 6 — Conclusion** summarises the research findings, confirms the degree to which each objective was achieved, and proposes directions for future work.

**Appendices** contain the full annotated source code listings for the Raspberry Pi Python controller, the Arduino Mega C++ firmware, and the Flask web dashboard.

---

*End of Chapter 1*
