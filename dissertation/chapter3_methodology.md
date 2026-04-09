# CHAPTER 3: METHODOLOGY

---

## 3.1 Introduction

This chapter presents the methodology used to design, implement, and validate the final system. The chapter follows the practical engineering flow of the project: requirements definition, hardware design, software architecture, communication design, navigation and obstacle-handling design, dashboard design, and validation strategy. The emphasis is placed on the implemented system as it now exists in the repository and on the operational philosophy that emerged during development: remote supervision first, assisted navigation second.

---

## 3.2 Systems Engineering Approach

The project was developed iteratively rather than as a single-pass design. Each major subsystem was first made to work independently, then integrated into the whole robot, then revised when practical field behaviour exposed a weakness. This was especially important in three areas:

- obstacle handling;
- manual and remote control;
- and the boundary between "autonomous" and "operator-supervised" behaviour.

The final requirements therefore reflect the system that was actually stabilised, not merely the earliest concept.

**Table 3.1: Final System Requirements**

| ID | Requirement | Category |
|---|---|---|
| SR-01 | The system shall measure temperature, humidity, smoke-related gases, air-quality-related gases, and carbon monoxide in real time | Functional |
| SR-02 | The system shall display gas values in calibrated PPM rather than raw ADC counts | Functional |
| SR-03 | The system shall provide a browser dashboard for monitoring and control | Functional |
| SR-04 | The system shall support remote supervised control over local networking and LTE-backed internet access | Functional |
| SR-05 | The system shall support direct local manual control through a CC1101 handheld controller | Functional |
| SR-06 | The system shall support dashboard joystick control through a USB serial bridge | Functional |
| SR-07 | The Raspberry Pi and Arduino Mega shall coordinate through a direct I2C link | Functional |
| SR-08 | The Arduino Mega shall implement local motor-side safety and obstacle reaction using servo-mounted ultrasonic sensing | Functional |
| SR-09 | The robot shall support waypoint assistance, heading acquisition, and return functions under supervision | Functional |
| SR-10 | The total hardware cost shall remain within low-cost undergraduate prototype limits | Constraint |
| SR-11 | The design shall avoid blocking logic that could freeze the Mega control loop | Constraint |

The final system should therefore be understood as a supervised environmental monitoring robot with assisted autonomy, not as a robot intended for blind fully autonomous deployment.

---

## 3.3 System Architecture Overview

### 3.3.1 Architectural Principle

The implemented architecture is a direct dual-processor arrangement:

- **Raspberry Pi 3 Model B**
  Performs sensor aggregation, GPS and compass processing, data storage, dashboard communication, LTE access, camera handling, and high-level navigation logic.

- **Arduino Mega 2560**
  Performs motor control, CC1101 radio reception, servo-mounted ultrasonic scanning, local reactive obstacle handling, and three-state ownership management.

The Raspberry Pi and Arduino Mega are directly connected over I2C. This is an explicit feature of the present system and must not be conflated with the more loosely described processor relationships in the reviewed literature.

### 3.3.2 Ownership Model

The Arduino Mega firmware is organised around a three-state ownership machine:

- `STATE_I2C`
  The Raspberry Pi owns motion intent through I2C commands.

- `STATE_WIRELESS`
  The ESP8266/CC1101 handheld controller owns motion intent.

- `STATE_FAILSAFE`
  Motors are stopped because safe control ownership has been lost.

This design ensures that only one control source owns the motors at a time and that wireless manual control always overrides Pi-side motion when direct CC1101 control is active.

### 3.3.3 Operational Philosophy

The completed robot operates in three practical modes:

1. **Remote supervised monitoring via dashboard**
   The primary field mode. The operator uses a browser over Wi-Fi or LTE connectivity to observe the robot and issue commands.

2. **Local manual control via CC1101**
   A near-range operator mode for direct manual driving.

3. **Assisted waypoint navigation**
   A convenience mode in which the Pi computes heading and waypoint progress while the Mega still retains local motor-side obstacle behaviour.

This practical mode split became a defining characteristic of the final system.

---

## 3.4 Hardware Design

### 3.4.1 Raspberry Pi Subsystem

The Raspberry Pi 3 Model B was chosen for the high-level controller because it provides:

- Linux support for Python, Flask, and networking;
- USB connectivity for the SIM7600E modem and optional joystick-attached devices;
- hardware I2C for Pi-side peripherals and Mega communication;
- CSI camera support for live video;
- and sufficient processing margin for navigation, database, and dashboard tasks.

The Pi hosts the dashboard backend and is therefore both the robot controller and the operator-access gateway.

### 3.4.2 Arduino Mega Subsystem

The Arduino Mega 2560 was chosen because it offers:

- multiple UARTs for GPS and debugging;
- PWM-capable outputs for differential motor drive;
- enough I/O for ultrasonic, servo, buzzer, and radio hardware;
- and deterministic superloop execution without Linux scheduling jitter.

The Mega is not used as a second general-purpose computer. Its role is deliberately narrower: real-time drive execution, direct radio reception, and safety-sensitive local behaviour.

### 3.4.3 Major Sensors and Actuators

The final implemented hardware set is summarised below.

**Environmental sensing**

- DHT11 temperature and humidity sensor
- MQ-2 gas sensor
- MQ-135 gas sensor
- MQ-7 gas sensor
- ADS1115 16-bit ADC for analogue sensor acquisition

**Navigation and positioning**

- NEO-6M GPS module
- Pi-side compass / magnetometer used by the Raspberry Pi navigation path
- SIM7600E integrated GPS as a secondary Pi-side source

**Vision and monitoring**

- Raspberry Pi Camera Module for live forward video

**Obstacle sensing**

- HC-SR04 ultrasonic sensor mounted on a servo

**Drive and control**

- differential drive motors through an L298N driver
- CC1101 radio on the Mega for local wireless manual control
- ESP8266 handheld transmitter with dual joysticks operating in tank-drive form

The earlier draft references to ZigBee/XBee and KY-032 infrared sensing do not describe the completed system and are therefore removed from the updated dissertation.

### 3.4.4 Core Mega Pin Usage

The core motor and obstacle-related Mega pin usage in the implemented firmware is shown in Table 3.2.

**Table 3.2: Core Arduino Mega Pin Assignments**

| Pin(s) | Role | Connected Device |
|---|---|---|
| 5, 22, 23 | Left motor PWM and direction | L298N left channel |
| 6, 24, 25 | Right motor PWM and direction | L298N right channel |
| 7 | Servo PWM | Ultrasonic scan servo |
| 10 | Buzzer | Passive buzzer |
| 18, 19 | Serial1 | NEO-6M GPS |
| 20, 21 | I2C | Raspberry Pi direct link |
| 30, 31 | Digital I/O | HC-SR04 trigger and echo |
| 50, 51, 52, 53 | SPI | CC1101 module |
| 2, 3 | CC1101 status lines | GDO0 and GDO2 |

### 3.4.5 ESP8266 Handheld Controller

The ESP8266 handheld controller uses:

- an ADS1115 ADC,
- two joysticks,
- and a CC1101 radio module.

The current transmitter firmware operates in tank-drive form, assigning one joystick axis to each wheel. The same handheld unit can also be connected by USB to the dashboard machine, where a serial bridge reads its wheel values and forwards them through the dashboard to the Raspberry Pi. A recent refinement added an explicit USB claim mode so that when dashboard joystick control is active, the same ESP8266 temporarily suppresses direct CC1101 transmission to avoid conflicting control ownership.

---

## 3.5 Sensor Integration and Calibration

### 3.5.1 DHT11

The DHT11 is connected to the Raspberry Pi GPIO interface and sampled by the Pi-side sensor manager. Its role is environmental context rather than laboratory-grade meteorology. The sensor provides a low-cost ambient temperature and relative humidity baseline to accompany the gas measurements.

### 3.5.2 ADS1115 and MQ Sensors

The MQ gas sensors produce analogue voltages that must be digitised externally because the Raspberry Pi has no native analogue inputs. An ADS1115 on I2C performs this conversion. Each MQ sensor is assigned a dedicated ADC channel, and the Pi-side gas calibration module converts the digitised values into PPM using a log-log model based on manufacturer curves.

The present implementation uses explicit clean-air assumptions and gas-specific calibration constants so that the dashboard can present:

- smoke-related concentration for MQ-2,
- CO₂-oriented air-quality estimates for MQ-135,
- and carbon monoxide values for MQ-7.

This is a major methodological improvement over raw-count reporting.

### 3.5.3 Gas Warning Logic

The dashboard warning system is based on calibrated PPM bands rather than raw ADC thresholds. The calibrated warning logic is therefore consistent with the values actually shown to the operator. This alignment became important during development because mismatches between raw-triggered warnings and displayed PPM values created operator confusion and had to be corrected in the dashboard code.

### 3.5.4 GPS and Compass

The robot uses two GPS-related sources:

- the NEO-6M path associated with the Mega subsystem;
- and the SIM7600E GPS path on the Raspberry Pi.

The Pi-side compass path supplies heading information to the Raspberry Pi navigation logic and to the dashboard compasses. A correction workflow was added so that the dashboard can distinguish between raw magnetic heading and corrected heading, and so that navigation uses a corrected heading path rather than a cosmetically adjusted display only.

---

## 3.6 Software Architecture

### 3.6.1 Raspberry Pi Control Software

The Raspberry Pi software is implemented in Python and is responsible for:

- polling and formatting sensor data;
- managing camera and dashboard updates;
- handling command polling and instant commands;
- maintaining Pi-side navigation state;
- forwarding motor intent to the Mega over I2C;
- communicating with the SIM7600E module;
- and storing or relaying system data.

The navigation engine on the Pi is implemented as `NavController`. It computes target bearing, heading error, distance-to-target, and reacquisition after an obstacle has been passed. Importantly, the Pi no longer owns the physical obstacle manoeuvre itself. Local obstacle motion belongs to the Mega, while the Pi remains aware of obstacle state and resumes the higher-level task afterward.

### 3.6.2 Arduino Mega Firmware

The Mega firmware is a non-blocking superloop that respects the project's golden rules:

- no state-machine freezing;
- no blocking navigation logic in the main loop;
- and explicit motor ownership.

Its responsibilities include:

- applying motor outputs,
- reacting to CC1101 wireless packets,
- receiving I2C commands,
- running servo-mounted ultrasonic scans,
- executing local reactive avoid behaviour,
- and broadcasting status back to the Pi.

### 3.6.3 Dashboard Software

The dashboard is implemented with Flask and Flask-SocketIO. The frontend uses:

- Chart.js for plots,
- Leaflet/OpenStreetMap for mapping,
- HTML/CSS/JavaScript for the control interface,
- and a camera stream relay for live video.

The dashboard is more than a display. It is an active command surface supporting:

- waypoints,
- manual arrows,
- hard-locked targets,
- return-home requests,
- speed control,
- joystick bridge status,
- and camera-assisted supervision.

### 3.6.4 AI Vision

AI vision remains present in the software stack, but its safety role has been deliberately reduced. Obstacle avoidance is now treated as a local Mega responsibility. The AI path is therefore better described as experimental visual assistance or full-drive experimentation, not as the primary safety layer of the robot.

---

## 3.7 Communication Design

### 3.7.1 Raspberry Pi to Arduino Mega I2C

The Raspberry Pi and Arduino Mega communicate through a direct I2C protocol. The Raspberry Pi sends:

- manual drive commands,
- raw motor values,
- waypoint management commands,
- navigation start/pause/resume/stop requests,
- heading data,
- and status-related commands.

The Mega returns:

- acknowledgements,
- navigation status,
- GPS/status data where applicable,
- and obstacle/reactive-avoid state information.

This direct link is one of the most important concrete architectural features of the system and differentiates the present implementation from vaguer multi-board descriptions in earlier literature.

### 3.7.2 Dashboard to Raspberry Pi

The dashboard communicates with the Raspberry Pi using:

- HTTP/REST requests for command submission,
- WebSocket events for real-time updates,
- and a fast instant-command queue for low-latency motion commands.

This enables manual driving, target control, and live monitoring without requiring a custom mobile application.

### 3.7.3 CC1101 Local Wireless Control

The CC1101 path provides a direct local manual channel between the handheld ESP8266 transmitter and the Mega. This is useful when a nearby operator wants immediate local radio control without depending on the dashboard path.

### 3.7.4 Dashboard USB Joystick Bridge

An important later addition to the methodology was the USB joystick bridge. In this mode:

1. the ESP8266 handheld controller is connected to the dashboard machine by USB;
2. the dashboard backend reads wheel-speed values from serial;
3. those values are forwarded to the Raspberry Pi;
4. the Pi relays them over I2C to the Mega.

To avoid simultaneous radio and USB control from the same transmitter, the dashboard now sends a serial claim command that temporarily disables CC1101 transmission while the USB bridge is active. This refinement is important because the Mega gives direct wireless priority over Pi motion, and without the claim mechanism the two paths could conflict.

### 3.7.5 LTE and Internet Access

The SIM7600E module provides the practical long-range communication path by giving the Raspberry Pi internet connectivity. This allows the dashboard to serve as the realistic long-range operator interface. In effect, LTE extends robot reach far beyond the range of a short-range local radio, provided network coverage exists.

---

## 3.8 Navigation and Obstacle-Handling Design

### 3.8.1 Navigation Philosophy

The navigation system is deliberately split into:

- **Pi-side reasoning**
  bearing acquisition, waypoint progress, heading correction, and task resumption;

- **Mega-side local reaction**
  immediate motor-side obstacle planning and avoid/stop behaviour.

This arrangement prevents the Pi from having to execute physical reactive manoeuvres at the motor layer while still preserving high-level navigation context.

### 3.8.2 Servo-Based Local Obstacle Logic

The local obstacle sequence implemented on the Mega is based on three thresholds:

- **90 cm**
  obstacle detection and planning zone;

- **60 cm**
  avoidance commit zone;

- **30 cm**
  failed avoidance stop threshold.

When the obstacle falls into the planning band, the servo-mounted HC-SR04 begins scanning. If a valid side is identified and the distance decreases into the avoid band, the Mega performs a local avoid manoeuvre. If the obstacle remains ahead at 30 cm or less, avoidance is treated as failed and the robot stops rather than continuing blindly.

### 3.8.3 Distinction Between Dashboard Arrows and Raw Joystick Modes

The project uses two different manual-drive philosophies:

- **Dashboard arrows / Pi-owned manual commands**
  These can invoke local Mega auto-avoid behaviour.

- **Raw joystick modes**
  These prioritise operator authority. In joystick raw mode the Mega scans automatically when needed, but local auto-avoid requires explicit acknowledgement by the operator's control release logic. After one acknowledged avoid, the robot stops and waits for further manual action.

This distinction was added after testing showed that automatic takeover during all manual modes was not desirable, especially in rough terrain or grassy environments where ultrasonic readings can be ambiguous.

### 3.8.4 Why Full Trust in Autonomy Was Rejected

The methodological conclusion of the project is that a single servo-mounted ultrasonic sensor is useful, but not sufficient to justify unconditional trust in full autonomy. The dissertation therefore treats waypoint navigation as an assisted capability and remote dashboard supervision as the primary practical operating model.

---

## 3.9 Web Dashboard Design

The dashboard was designed to be the main operator interface. Its main functional areas are:

- environmental sensor display and historical plotting;
- map-based robot tracking and waypoint entry;
- dual compass visualisation with correction support;
- live camera view;
- manual arrow control;
- joystick bridge monitoring;
- manual target lock and return-home functions;
- and AI vision panels for experimental assistance.

The dashboard can be used locally on the same network as the robot or remotely when the Pi is reachable over LTE-backed internet connectivity.

---

## 3.10 Testing and Validation Methodology

Validation was performed in layers:

1. **Bench validation**
   Sensor reading checks, gas calibration verification, ADC checks, GPS and compass checks, and dashboard rendering checks.

2. **Communication validation**
   Pi-Mega I2C checks, CC1101 local control checks, and USB joystick bridge validation.

3. **Mobility validation**
   Manual driving, tank drive, heading acquisition, manual target mode, and return-home logic.

4. **Obstacle validation**
   Servo scan, local avoidance thresholds, manual acknowledgement behaviour, and fail-stop behaviour at the defined near-obstacle threshold.

5. **Integrated trials**
   Supervised waypoint runs and remote dashboard operation with live sensing.

The updated dissertation intentionally avoids overstating the final system as if it were already validated for fully unsupervised deployment in arbitrary outdoor conditions.

---

## 3.11 Summary

This chapter has described the complete methodology of the final implemented system. The robot is built on a direct Raspberry Pi to Arduino Mega architecture, calibrated sensing, browser-based supervision, CC1101 local manual control, LTE-backed remote access, and servo-based local reactive obstacle handling. Most importantly, the methodology shows how the project moved from an initially autonomy-centred concept to a more realistic supervised environmental robotics platform. Chapter 4 presents the resulting system capabilities and validation outcomes.

---

*End of Chapter 3*
