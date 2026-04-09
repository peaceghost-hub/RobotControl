# CHAPTER 6: CONCLUSION

---

## 6.1 Introduction

This chapter concludes the dissertation by summarising the final implemented system, restating the extent to which the research objectives were achieved, identifying the main contributions of the work, and outlining realistic directions for future improvement. The conclusion follows the revised position of the dissertation: the completed robot is best understood as a supervised environmental monitoring platform with assisted navigation features, not as a fully trustworthy autonomous field robot.

---

## 6.2 Summary of the Research

This dissertation has presented the design, implementation, and refinement of an ARM-based Environmental Monitoring and Aide Robot built around a Raspberry Pi 3 Model B and an Arduino Mega 2560. The work began from the broad idea of a low-cost autonomous environmental robot, but practical testing and integration led to a more grounded and more useful result.

The final platform combines:

- calibrated environmental sensing using DHT11, MQ-2, MQ-135, MQ-7, and ADS1115;
- browser-based monitoring and control through a Flask/SocketIO dashboard;
- direct Raspberry Pi to Arduino Mega coordination over I2C;
- servo-mounted ultrasonic obstacle scanning on the Mega;
- CC1101 local manual control;
- dashboard-mediated joystick teleoperation through a USB serial bridge;
- live camera streaming;
- GPS- and compass-assisted waypoint functions;
- and LTE-backed remote dashboard access through the SIM7600E module.

The most important conclusion of the work is not that the robot has "some autonomous features," but that the final system is operationally strongest when used under supervision. Autonomous navigation was implemented and tested in the project, but the dissertation deliberately stops short of treating that as sufficient proof of trustworthy field autonomy because the robot's local obstacle perception is still based mainly on one servo-mounted ultrasonic sensor. In other words, the project demonstrates that a low-cost environmental robot becomes more practical when it prioritises:

- good operator visibility,
- good remote access,
- clear control ownership,
- meaningful sensor presentation,
- and conservative local safety behaviour.

That conclusion is both a technical result and a design lesson.

---

## 6.3 Achievement of Research Objectives

The achievement of the research objectives is summarised below.

**OBJ-1 — Dual-Processor Hardware Architecture:** *Achieved.*
The Raspberry Pi and Arduino Mega were successfully integrated through a direct I2C relationship with clear separation of high-level and low-level responsibilities.

**OBJ-2 — Environmental Sensing and PPM Reporting:** *Achieved.*
The final robot presents environmental measurements through calibrated gas channels and dashboard warning logic rather than raw ADC values alone.

**OBJ-3 — Browser Dashboard for Monitoring and Control:** *Achieved.*
The browser dashboard now acts as the practical control centre of the robot, combining monitoring, mapping, camera viewing, manual control, and navigation support.

**OBJ-4 — Assisted Navigation and Local Obstacle Handling:** *Achieved at assisted-navigation level.*
Waypoint logic, heading acquisition, return functions, and local servo-based obstacle handling were implemented. However, the project now correctly interprets these capabilities as assisted navigation rather than as proof of dependable unsupervised autonomy.

**OBJ-5 — Practical Manual Control Paths:** *Achieved.*
The system now supports both CC1101 local manual control and dashboard-mediated joystick teleoperation. This provides a practical control architecture for both local and longer-range supervised operation.

**OBJ-6 — Integrated Prototype Validation:** *Achieved at prototype level.*
The completed system functions as an integrated platform in which sensing, dashboard access, teleoperation, assisted navigation, and local obstacle behaviour coexist coherently.

---

## 6.4 Main Contributions of the Project

The project makes the following contributions.

1. **A practical supervised environmental robot architecture**
   The dissertation demonstrates a low-cost architecture in which a Raspberry Pi and Arduino Mega are used in a genuinely complementary way rather than redundantly.

2. **A direct and explicit Pi-to-Mega coordination model**
   The completed implementation shows how high-level coordination and low-level motor control can be linked cleanly through a direct command protocol.

3. **A browser-first supervision model for environmental robotics**
   The dashboard becomes the robot's primary operational interface, making monitoring and control available across ordinary computing devices and over LTE-backed connectivity.

4. **Calibrated operator-facing gas monitoring**
   The robot reports approximate gas concentrations and warning bands in a more interpretable way than raw-count-only systems.

5. **A combined local and remote teleoperation strategy**
   CC1101 provides nearby direct control, while the dashboard joystick bridge extends supervised manual control through the browser path.

6. **A servo-based local obstacle reaction layer on the Mega**
   The Mega now performs local reactive scanning and obstacle response with explicit detect, avoid, and fail-stop thresholds.

Just as importantly, the dissertation contributes a more careful claim about autonomy. It argues that engineering honesty about system limits is itself a contribution when many low-cost robotic systems are described too confidently relative to their sensing stack.

---

## 6.5 Limitations of the Final System

The current robot still has important limitations.

- A single servo-mounted ultrasonic sensor is still a limited perception system.
- GPS and low-cost compass heading remain imperfect in real outdoor conditions.
- MQ-series gas sensing remains approximate and non-certified.
- The dashboard joystick bridge depends on clean serial behaviour and host reliability.
- Low-cost motors and drivetrain mechanics limit terrain performance and turning authority.
- AI remains supportive or experimental rather than a trusted obstacle-safety layer.

These limitations do not invalidate the project. Rather, they define the boundary within which the present robot should be used responsibly.

---

## 6.6 Recommendations for Future Work

The following recommendations arise naturally from the final state of the project.

**1. Improve perception before increasing autonomy claims**
If the project is to move closer to trustworthy autonomous operation, richer sensing is required. This may include stereo vision, depth sensing, LiDAR, or a more structured local mapping approach. Stronger autonomy should follow stronger perception, not the other way around.

**2. Perform formal gas validation against reference instruments**
The gas subsystem would benefit from controlled comparison against certified reference equipment so that measurement uncertainty can be stated more rigorously.

**3. Improve localisation accuracy**
Higher-quality GNSS or correction techniques would strengthen waypoint usefulness and reduce navigation uncertainty.

**4. Refine teleoperation hardware for long-duration use**
The dashboard joystick bridge and handheld controller are already useful, but they could be further improved through better enclosure design, cleaner joystick hardware, and more robust transmitter-side calibration workflows.

**5. Strengthen system resilience**
Future work should include power management, watchdog recovery, and more formal communication-loss testing under realistic field conditions.

**6. Extend obstacle handling beyond reactive local behaviour**
The present servo-scan approach is a reasonable undergraduate solution, but future path planning would benefit from memory, mapping, or obstacle persistence mechanisms that reduce repeated encounters.

**7. Expand the dashboard into a stronger field-operations console**
The dashboard is already the strongest practical part of the platform. Future work could expand mission logging, replay tools, richer operator prompts, and multi-session supervisory workflows.

---

## 6.7 Final Concluding Remarks

The final form of this project is more valuable than a superficial "autonomous robot" label would suggest. It demonstrates that a low-cost environmental robot can become genuinely useful when the design priorities are chosen carefully: give the operator a strong dashboard, keep sensing interpretable, separate high-level and low-level control clearly, and let local safety behaviour remain conservative.

The completed robot therefore stands as a practical environmental aide platform. It can move, sense, stream, warn, report, and be supervised over modern network infrastructure. It can assist with navigation and handle some local obstacle cases. But most importantly, it does so without pretending to be more autonomous than its sensors justify. That is the central conclusion of the dissertation and the clearest statement of where the system now rests.

---

*End of Chapter 6*
