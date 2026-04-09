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

**April 2026**

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

To fellow students and colleagues who participated as test observers during bench and field validation trials, and whose feedback contributed to the refinement of the dashboard, joystick control path, and navigation behaviour.

To the open-source community whose contributions to Python, Flask, Flask-SocketIO, Chart.js, Leaflet.js, Arduino, GeographicLib, TinyGPSPlus, and related embedded software libraries made this project achievable within undergraduate time and budget constraints.

Finally, to my family and friends, for their patience and moral support throughout the long hours of hardware debugging, software development, testing, and writing.

---

&nbsp;

## ABSTRACT

This dissertation presents the design, implementation, and validation of an ARM-based Environmental Monitoring and Aide Robot built around a Raspberry Pi 3 Model B and an Arduino Mega 2560. The completed system is a browser-controlled environmental monitoring platform that combines real-time gas sensing, GPS localisation, live camera streaming, local reactive obstacle handling, and both local and internet-based teleoperation. Autonomous-style waypoint navigation and heading acquisition were both implemented and tested, but the practical operating emphasis of the finished robot is supervised remote operation rather than fully unsupervised autonomy because the robot's local obstacle perception remains centred on a single servo-mounted ultrasonic sensor.

The Raspberry Pi performs high-level tasks including dashboard communication, sensor aggregation, GPS and compass processing, navigation logic, data persistence, and LTE-enabled internet access through the SIM7600E module. The Arduino Mega implements the real-time motor interface, CC1101 local wireless receiver, non-blocking three-state control machine, and servo-mounted HC-SR04 obstacle scanning. The two processors are directly linked by an explicit I2C command protocol, which is a major architectural distinction of the implemented system. Environmental sensing is provided by a DHT11 temperature-humidity sensor together with MQ-2, MQ-135, and MQ-7 gas sensors digitised through an ADS1115 ADC and converted into calibrated parts-per-million values for dashboard display and warning generation.

The browser dashboard functions as the practical control centre of the robot. It provides live charts, map-based monitoring, compass visualisation, manual arrows, manual target locking, return-home support, live MJPEG video, and joystick-assisted teleoperation. A USB serial joystick bridge was added so that an ESP8266 handheld controller can feed wheel-speed commands into the dashboard and onward to the Raspberry Pi, while a local CC1101 radio path remains available for short-range direct manual control. For long-range operation, the dashboard can be reached through the Raspberry Pi's LTE data connection, making 4G-assisted remote environmental survey a more realistic use case than relying purely on autonomous obstacle avoidance from a single ultrasonic sensor.

The final system contributes a practical low-cost architecture for supervised environmental robotics: calibrated gas sensing, direct Raspberry Pi to Arduino Mega coordination, servo-based local obstacle scanning, tank-drive joystick control, and a web dashboard that removes the range and platform restrictions of Bluetooth-only or app-only control schemes reported in earlier low-cost environmental robot literature. The dissertation therefore argues that, at this stage of implementation, the robot is best understood as a remotely operable environmental aide platform with assisted navigation features, rather than as a fully trustworthy autonomous field robot.

**Keywords:** Environmental monitoring robot, Raspberry Pi, Arduino Mega, I2C control, web dashboard, LTE teleoperation, CC1101, tank drive, gas sensor calibration, servo ultrasonic scanning.

---

&nbsp;

## TABLE OF CONTENTS

| Section | Description |
|---|---|
| Declaration | Statement of originality |
| Dedication | Personal dedication |
| Acknowledgements | Recognition of support received |
| Abstract | Summary of the project and findings |
| Chapter 1 | Introduction |
| Chapter 2 | Literature Review |
| Chapter 3 | Methodology |
| Chapter 4 | Results |
| Chapter 5 | Discussion |
| Chapter 6 | Conclusion |
| Reference List | Bibliographic references |
| Appendices | Source code and implementation notes |

---

&nbsp;

## LIST OF ABBREVIATIONS

| Abbreviation | Meaning |
|---|---|
| ADC | Analogue-to-Digital Converter |
| ARM | Advanced RISC Machine |
| CC1101 | Sub-GHz wireless transceiver module used for local manual control |
| CEP | Circular Error Probable |
| CSI | Camera Serial Interface |
| GPS | Global Positioning System |
| GPIO | General Purpose Input/Output |
| I2C | Inter-Integrated Circuit |
| LTE | Long-Term Evolution (4G cellular communication) |
| MJPEG | Motion JPEG |
| MOS | Metal Oxide Semiconductor |
| PWM | Pulse Width Modulation |
| PPM | Parts Per Million |
| UART | Universal Asynchronous Receiver-Transmitter |
| UI | User Interface |

---

&nbsp;

## GLOSSARY

**Dashboard joystick bridge** — The USB serial software path in which an ESP8266 handheld controller is connected to the dashboard machine and its joystick values are forwarded through the dashboard backend to the Raspberry Pi and then to the Arduino Mega.

**Failsafe** — The deterministic state entered by the Arduino Mega when active control ownership is lost or when commanded safety logic requires motor halt. In this state, motor outputs are forced to stop.

**Hard-locked target** — A manual dashboard mode in which the operator sets a destination coordinate while keeping the robot's present compass direction as the travel heading to be reacquired after obstacle handling.

**NavController** — The Raspberry Pi navigation engine that computes bearing, heading error, and waypoint progress, and sends high-level drive intent to the Arduino Mega over I2C.

**Reactive avoidance** — The local Mega-side obstacle response based on a servo-mounted ultrasonic scan and fixed thresholds rather than on a global map or SLAM model.

**Servo scan** — The process of rotating the HC-SR04 ultrasonic sensor through left, centre, and right positions to estimate which direction is clearer before a local avoidance manoeuvre is started.

**STATE_I2C / STATE_WIRELESS / STATE_FAILSAFE** — The three mutually exclusive ownership states implemented on the Arduino Mega firmware. They determine whether the Mega is currently obeying Raspberry Pi commands, CC1101 wireless control, or safe-stop behaviour.

**Tank drive** — A differential drive scheme in which the left and right wheels are controlled independently, allowing forward motion, reverse motion, pivot turning, and curved steering by varying wheel speeds directly.

---

*End of Front Matter*
