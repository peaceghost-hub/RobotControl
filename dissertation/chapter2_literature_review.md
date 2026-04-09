# CHAPTER 2: LITERATURE REVIEW

---

## 2.1 Introduction

This chapter reviews the literature relevant to the final implemented form of the project. The review is organised around seven themes: environmental monitoring robots, ARM-based embedded control, low-cost gas sensing and calibration, GPS and compass navigation, obstacle sensing and reactive avoidance, wireless communication and teleoperation, and web-based dashboard supervision. The purpose of the review is not only to establish theoretical background, but also to explain why the finished system places greater emphasis on supervised remote operation than on fully unsupervised autonomy.

---

## 2.2 Environmental Monitoring Robots

Environmental monitoring systems are traditionally implemented either as fixed stations or as distributed static sensor networks [1], [2], [5], [6]. These systems are valuable for long-duration observation at known locations, but they lack mobility and cannot easily inspect a spatially changing hazard. Mobile robots address this limitation by carrying the sensing payload toward the area of interest, reducing human exposure to unsafe environments and improving spatial coverage [1], [2].

Earlier mobile environmental robots demonstrated the feasibility of combining gas sensors, temperature-humidity sensing, and movement on low-cost embedded platforms [15], [16], [17]. Salman et al. [17] are particularly important because they showed that an ARM-based environmental robot could combine GPS navigation, IoT reporting, and low-cost components. However, the literature also reveals a common pattern: the robot is often described primarily as "autonomous," while operator access, communication range, and safety fallback are treated as secondary matters. In practice, those secondary matters are often what determine whether a robot is usable outside a demonstration setting.

The present project therefore builds on the environmental monitoring robot literature while intentionally shifting emphasis. The implemented system still includes waypoint navigation, heading acquisition, and local obstacle handling, but it treats remote supervision as a first-class operating mode rather than as an afterthought.

---

## 2.3 ARM-Based Embedded Platforms for Robotics

ARM-based systems are attractive in robotics because they combine low power consumption, low cost, and sufficient computational capability for sensing, communication, and control [3], [8]. The Raspberry Pi family is especially well suited to projects that require Linux-based networking, databases, dashboards, web APIs, or camera handling. Arduino-class microcontrollers remain valuable where deterministic timing, interrupt handling, and simple direct motor control are needed [9].

The literature supports combining these two device classes, but it is important to state their relationship accurately. Salman et al. [17] demonstrate the joint use of Raspberry Pi and Arduino in one robotic system, but that study should not be treated as if it already contained the same explicit Raspberry Pi to Arduino Mega control link used in the present project. In the system implemented here, the Raspberry Pi and Arduino Mega are directly coordinated through a defined I2C command protocol. That direct link is a concrete design choice of this project, not merely an assumption imported from the literature.

This architectural distinction matters because it determines where responsibilities are assigned. In the present implementation:

- the Raspberry Pi handles high-level coordination, dashboard communication, LTE connectivity, GPS/compass-based navigation calculations, and data persistence;
- the Arduino Mega handles motor output, CC1101 local wireless reception, non-blocking state ownership, and local servo-based obstacle reaction.

This division reflects a practical hybrid model: the higher-level computer reasons, stores, and communicates, while the microcontroller safeguards and drives.

---

## 2.4 Environmental Sensors and Calibration

Low-cost environmental robots commonly use DHT-series temperature-humidity sensors together with MQ-series metal oxide gas sensors [10], [11], [17]. The chief benefit of MQ sensors is affordability and availability. Their chief limitation is that their output is not directly meaningful without interpretation. Many low-cost prototypes therefore stop at reporting raw ADC counts or voltages, which reduces the scientific usefulness of the data.

The literature on MQ sensors shows that calibration is difficult but not impossible [10], [11]. Datasheet curves relating Rs/R0 to gas concentration can be approximated using log-log models, allowing a low-cost system to produce approximate PPM values when clean-air assumptions and proper warm-up are observed. This does not make MQ sensors equivalent to laboratory instruments, but it does make them more interpretable than raw values alone.

For this reason, the present project adopts calibrated PPM reporting as a core design requirement. The robot uses:

- MQ-2 for smoke and combustible gases;
- MQ-135 for CO₂-oriented air-quality estimation;
- MQ-7 for carbon monoxide;
- and a DHT11 for ambient temperature and humidity context.

The significance of this decision is not merely technical. It changes the dashboard from a developer-facing instrument panel into an operator-facing monitoring tool whose alerts can be interpreted meaningfully in the field.

---

## 2.5 GPS Navigation, Compass Use, and the Limits of Low-Cost Autonomy

GPS waypoint navigation is well established in mobile robotics, especially for outdoor platforms where a metre-scale error band is acceptable [17]. A heading source, usually a magnetometer, is then used to reduce cross-track error between GPS updates. This general principle is sound and is retained in the present system.

However, the literature also makes clear that low-cost autonomy is highly sensitive to sensing quality. GPS has metre-level uncertainty, low-cost magnetometers require careful calibration and are vulnerable to nearby ferromagnetic materials, and simple range sensors provide only partial awareness of the environment. These constraints do not make navigation impossible, but they do affect how much autonomy can be trusted.

In the present project, the Raspberry Pi performs waypoint and heading calculations using GPS and compass data, while the Arduino Mega handles the local motor side. The resulting system can acquire heading, follow waypoints, and recover after some obstacle encounters. Yet the final implementation also demonstrates an important practical lesson: a robot that relies mainly on GPS, compass, and a single ultrasonic sensor should not be described as fully trustworthy in all outdoor environments. Instead, such a system is better framed as assisted navigation under supervision.

This framing aligns more honestly with the capabilities of the finished robot and directly informs the revised objectives of the dissertation.

---

## 2.6 Obstacle Detection and Reactive Avoidance

Obstacle avoidance in low-cost robots is often reactive rather than map-based. Salman et al. [17] used a simpler obstacle sensing concept based on local detection, while many comparable low-cost platforms rely on a forward-facing ultrasonic sensor or a simple threshold trigger. The limitation of fixed-angle sensing is that detection does not automatically reveal which side is safer.

Servo-mounted ultrasonic scanning is therefore a well-motivated improvement. By rotating a single HC-SR04 sensor through left, centre, and right positions, a robot can at least compare directional clearance before choosing an avoidance action. This remains much cheaper than LiDAR or stereo vision and fits the hardware limits of an undergraduate project.

At the same time, the literature and practical testing both show the limitations of such a strategy:

- ultrasonic readings can be affected by angle, surface material, grass, and narrow geometry;
- the robot has no full map of the environment;
- and a single sensor cannot reliably support globally optimal path planning.

The present project therefore uses servo scanning as a local reactive aid, not as proof of dependable full autonomy. The Arduino Mega's local avoidance logic implements thresholded detect-plan-avoid-stop behaviour, while the dissertation explicitly recognises that true autonomous trust would require richer perception than one scanning ultrasonic sensor.

---

## 2.7 Wireless Communication, Teleoperation, and Range

Wireless communication literature for mobile robots often begins with Bluetooth or short-range serial links [15], [17]. These are simple to implement but operationally restrictive. Salman et al. [17] used a short-range control scheme appropriate for laboratory demonstration, but such a link is inadequate when the robot is intended to travel beyond immediate operator proximity.

Two separate control-range problems exist in practice:

1. **Local direct manual control**
This requires a robust short-to-medium range radio path for nearby operator intervention.

2. **Remote supervision beyond local radio range**
This requires internet-connected control, which is better served by a web dashboard over Wi-Fi or LTE than by a handheld-only radio link.

The present project ultimately adopts exactly this split:

- **CC1101** is used as the implemented local radio path for direct manual control.
- **LTE-backed browser access** through the Raspberry Pi dashboard is used as the practical long-range supervision path.

ZigBee remains relevant in the literature as a useful comparison technology, but it is not part of the completed implementation and should not be described as an active system component in this dissertation.

This is an important conceptual shift. The practical long-range answer in the final robot is not "another short-range radio." It is dashboard-mediated remote control over IP connectivity.

---

## 2.8 Web Dashboards and Browser-Based Robot Supervision

Web dashboards are increasingly attractive in robotics because they are platform-independent, easy to update, and accessible from desktops, laptops, tablets, and phones without distributing a custom mobile application. Flask and Flask-SocketIO provide a lightweight Python backend for such systems, while JavaScript charting and map libraries provide responsive front-end visualisation.

This approach is especially relevant for environmental monitoring, where the operator benefits from seeing several information layers at once:

- live sensor values and warnings;
- historical trend plots;
- GPS position on a map;
- camera view;
- control state;
- and waypoint or heading information.

Compared with cloud-only IoT dashboards such as ThingSpeak, a local browser dashboard offers two major advantages for robot operation:

- lower latency and tighter operator feedback;
- and direct integration with robot control features rather than sensor plotting alone.

The present project adopts this browser-based model as the primary human-machine interface. That decision is central to the dissertation, because it changes the practical role of the robot from a mostly autonomous node that occasionally reports data into a supervised environmental platform that can be observed and steered in real time.

---

## 2.9 Identified Research Gaps

The literature review identifies the following gaps most relevant to the final implemented system:

1. **Control interfaces are often range-limited or platform-limited.** Bluetooth or app-specific interfaces constrain who can operate the robot and from how far away [15], [17].

2. **Low-cost environmental robots often report raw gas values instead of actionable quantities.** This weakens the usefulness of the sensor payload for real monitoring tasks.

3. **The relationship between high-level and low-level processors is often under-specified.** The need for a clearly defined Raspberry Pi to Arduino Mega coordination protocol is not always addressed explicitly.

4. **Autonomy is often overstated relative to sensing quality.** Low-cost robots with limited perception are commonly described as autonomous even when safe field use still depends on supervision.

5. **Fixed-angle obstacle sensing gives poor directional awareness.** Servo scanning improves this, but still does not equal full environment perception.

6. **Long-range practical teleoperation is under-emphasised.** A browser dashboard over Wi-Fi or LTE may be more useful in practice than adding multiple local radio schemes.

7. **Manual and assisted control integration is rarely treated as a primary design problem.** Yet in real deployments, switching between remote supervision, local radio control, and assisted waypoint motion is essential.

The present project addresses these gaps by combining calibrated sensing, a direct Pi-Mega architecture, a browser dashboard, LTE connectivity, CC1101 local manual control, USB joystick bridging, and explicitly limited claims about autonomy.

---

## 2.10 Summary

This chapter has reviewed the key literature domains relevant to the implemented robot and has shown why the final design takes its current form. The literature supports the use of ARM-based platforms, low-cost gas sensing, GPS-compass guidance, and reactive local avoidance, but it also reveals important limitations in communication range, operator access, and the realism of autonomy claims. These findings justify the final direction of the project: a supervised environmental monitoring robot with assisted navigation rather than a robot whose safety depends on over-trusting a minimal sensor stack. Chapter 3 describes the methodology by which this design was implemented.

---

*End of Chapter 2*
