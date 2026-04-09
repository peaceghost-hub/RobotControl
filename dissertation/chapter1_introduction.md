# CHAPTER 1: INTRODUCTION

---

## 1.1 Introduction

This chapter introduces the motivation, problem context, objectives, and scope of the ARM-Based Environmental Monitoring and Aide Robot. The project began from the idea of a low-cost autonomous environmental robot, but the final implemented system has matured into something more practical and defensible: a remotely supervised environmental monitoring platform with assisted navigation features, real-time dashboard control, and local safety logic on the Arduino Mega. This distinction is central to the dissertation and is reflected throughout the revised objectives and discussion.

---

## 1.2 Background and Motivation

Environmental monitoring remains essential in industrial, urban, mining, agricultural, and disaster-affected settings where hazardous gases, smoke, or poor air quality may threaten human health. Traditional fixed monitoring stations are useful for long-term measurement at a single location, but they cannot reposition as conditions change and they cannot safely enter places that are actively unsafe for people. This limitation creates a strong motivation for mobile monitoring systems able to move sensors toward the area of interest rather than waiting for pollutants to drift toward a static node.

Low-cost embedded computing platforms have made such systems increasingly feasible. Raspberry Pi boards provide Linux, networking, storage, USB peripherals, and enough processing power for web servers, GPS handling, dashboards, and camera streaming. Arduino-class microcontrollers provide dependable timing for motor actuation, PWM generation, and low-level safety logic. Combining the two creates a useful division of labour: the single-board computer handles computation and connectivity, while the microcontroller handles deterministic motor-side behaviour.

The literature, especially the work of Salman et al. [17], demonstrates the value of ARM-based platforms in environmental robotics. However, the current project shows that the most practically useful improvement is not merely "more autonomy." In a low-cost robot that relies mainly on GPS, compass heading, and a single ultrasonic range sensor, fully unsupervised navigation cannot be treated as trustworthy in all outdoor conditions. Grass, uneven ground, side walls, narrow passages, reflections, and temporary obstacles all challenge simple reactive avoidance. For this reason, the present work gradually shifted emphasis away from treating autonomy as the sole operating mode and toward a more realistic operational model: remote supervision through a browser dashboard, supported by local obstacle safety on the Mega and optional waypoint assistance on the Raspberry Pi.

This supervised model is strengthened by the addition of the SIM7600E cellular modem and the browser-based dashboard. Unlike short-range handheld-only control, the dashboard can be reached through Wi-Fi or LTE internet connectivity, which means the operator is not fundamentally limited by the range of a local radio link. The robot can therefore be viewed as an environmental aide platform: it can sense, report, stream video, receive commands, and assist movement decisions, while still leaving final human judgement available through the dashboard.

---

## 1.3 Problem Statement

There is a practical need for a low-cost mobile environmental monitoring robot that can:

- measure hazardous environmental variables in real time;
- transmit those measurements to a remotely accessible interface;
- be manually controlled from a safe distance through modern network infrastructure;
- retain enough local intelligence to help with waypoint following and obstacle handling;
- and fail safely when communications or sensing become unreliable.

The problem with many earlier low-cost prototypes is not that they lack movement, but that they overstate autonomous capability relative to the sensing hardware available. A robot that depends on GPS, compass heading, and a single ultrasonic sensor may demonstrate autonomous waypoint travel in controlled conditions, but cannot be relied upon as a fully autonomous field robot in all real outdoor settings. In addition, many previously reported systems depend on Bluetooth or app-only interfaces, limiting both operational range and platform flexibility.

The core problem addressed by this project is therefore the design of a more practical system: an ARM-based environmental monitoring robot that combines calibrated sensing, browser-based supervision, LTE-enabled remote access, direct Raspberry Pi to Arduino Mega coordination, and local motor-side safety, while treating autonomy as assisted functionality rather than as an unquestioned operating assumption.

---

## 1.4 Scope and Delimitations

This project covers the design, implementation, and testing of a dual-processor robot platform built around:

- a Raspberry Pi 3 Model B for dashboard communication, data handling, GPS and compass processing, navigation logic, and LTE connectivity;
- an Arduino Mega 2560 for motor actuation, local reactive obstacle handling, CC1101 wireless reception, and failsafe state management;
- a DHT11 sensor and three MQ-series gas sensors (MQ-2, MQ-135, MQ-7) sampled through an ADS1115 ADC;
- a NEO-6M GPS receiver, a Pi-side magnetometer, a SIM7600E LTE/GPS module, a Pi camera, a servo-mounted HC-SR04 ultrasonic sensor, and a differential-drive motor system;
- a web dashboard offering sensor charts, map-based monitoring, live video, manual motion commands, joystick support, and waypoint tools;
- a CC1101 local wireless control path and a USB serial joystick bridge that allows the same handheld controller to be used through the dashboard.

The following are outside the scope of the present work:

- LiDAR, stereo vision, SLAM, or occupancy-grid mapping for trustworthy full autonomy;
- formal industrial certification of gas readings against laboratory reference instruments;
- large-scale multi-robot deployment;
- a native mobile application;
- Google Maps integration or road-navigation services;
- and fully autonomous operation in dense, cluttered, or indoor environments.

The project therefore focuses on a deployable undergraduate prototype for supervised environmental survey rather than on a fully autonomous production robot.

---

## 1.5 Research Objectives

The primary objective of this research is to design, implement, and validate a low-cost ARM-based environmental monitoring robot that supports practical remote supervision through a browser dashboard while retaining assisted waypoint navigation and local motor-side safety behaviour.

The specific objectives are:

1. **OBJ-1:** To design and implement a dual-processor hardware architecture in which a Raspberry Pi and an Arduino Mega are directly coordinated through an explicit I2C command protocol.

2. **OBJ-2:** To integrate and calibrate an environmental sensor suite comprising DHT11, MQ-2, MQ-135, and MQ-7 sensors, and to present gas readings in meaningful PPM units rather than raw ADC values.

3. **OBJ-3:** To implement a browser-based monitoring and control dashboard that supports live charts, map interaction, camera streaming, manual drive, joystick-assisted drive, and operator visibility over both local and LTE-connected access paths.

4. **OBJ-4:** To implement assisted waypoint navigation and local Mega-side obstacle handling using GPS, compass heading, and a servo-mounted ultrasonic sensor, while explicitly recognising the limits of low-cost autonomy.

5. **OBJ-5:** To implement practical remote-control modes using CC1101 local radio and dashboard-mediated joystick control, including a tank-drive path suitable for operator-supervised motion.

6. **OBJ-6:** To validate the complete system as a supervised environmental monitoring platform through subsystem testing, integration testing, and controlled navigation and teleoperation trials.

---

## 1.6 Research Questions

This dissertation addresses the following research questions:

1. How can a Raspberry Pi and Arduino Mega be partitioned so that networked supervision and deterministic motor control are both achieved reliably in one robot?

2. How effectively can low-cost MQ-series gas sensors be converted from raw ADC values into usable calibrated PPM information for live environmental monitoring?

3. What is the most practical operational role of low-cost waypoint navigation when the robot's local obstacle sensing is limited to a servo-mounted ultrasonic sensor?

4. How effective is a browser dashboard, accessed over local networking and LTE internet, as the primary operator interface for a mobile environmental robot?

5. How can local CC1101 manual control and dashboard-mediated joystick control be integrated so that the operator retains meaningful authority over the robot at different practical ranges?

---

## 1.7 Significance of the Study

This study is significant for several reasons.

First, it reframes the contribution of low-cost environmental robots in a more realistic way. Rather than claiming that inexpensive sensors automatically yield trustworthy autonomy, it demonstrates a safer and more practical middle ground: assisted navigation combined with strong remote supervision.

Second, it demonstrates that a browser dashboard can serve as the central operating interface for an environmental robot. This removes the platform restrictions of app-only designs and expands control range from short-range local radio to any location where LTE-backed internet connectivity is available.

Third, it provides a directly implemented Raspberry Pi to Arduino Mega architecture with explicit I2C command handling, rather than leaving the relationship between high-level and low-level processing ambiguous.

Fourth, it upgrades environmental sensing from raw voltage-style reporting to calibrated gas readings, which improves the usefulness of the collected data for interpretation and alerting.

Finally, it offers a reproducible undergraduate-scale example of how remote robotics, embedded systems, environmental sensing, and web technologies can be brought together into one coherent project with practical field value.

---

## 1.8 Dissertation Structure

The remainder of this dissertation is organised as follows.

**Chapter 2 — Literature Review** reviews prior work in environmental monitoring robots, ARM-based embedded platforms, gas sensing and calibration, navigation, obstacle handling, wireless communication, and browser-based supervisory control. It identifies the gaps most relevant to the final implemented system.

**Chapter 3 — Methodology** describes the hardware and software design of the robot, including the Raspberry Pi to Arduino Mega architecture, sensor integration, dashboard design, communication paths, joystick support, and obstacle-handling strategy.

**Chapter 4 — Results** presents the implemented system capabilities and the outcomes of subsystem and integration testing, with emphasis on calibrated sensing, dashboard control, joystick operation, waypoint assistance, and local obstacle handling.

**Chapter 5 — Discussion** interprets the results, relates them to the research questions, and explains why supervised remote control emerged as the most practical operational focus of the finished system.

**Chapter 6 — Conclusion** summarises the work, states the degree of objective achievement, and outlines future improvements required for higher-trust autonomy.

**Appendices** provide updated notes on the major source files and implementation structure used in the completed project.

---

*End of Chapter 1*
