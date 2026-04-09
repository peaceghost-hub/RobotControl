# CHAPTER 5: DISCUSSION

---

## 5.1 Introduction

This chapter interprets the updated results in relation to the research objectives, research questions, and literature reviewed earlier. The central argument of the chapter is that the most valuable outcome of the project is not a claim of complete autonomy, but the creation of a practical supervised environmental monitoring platform built around a clear Raspberry Pi to Arduino Mega architecture, a browser dashboard, LTE-backed access, and multiple well-defined manual control paths.

---

## 5.2 Discussion of Research Objectives

### 5.2.1 OBJ-1: Dual-Processor Hardware Architecture

This objective was achieved. The completed robot clearly separates high-level and low-level responsibilities:

- the Raspberry Pi manages networking, dashboard interaction, data handling, GPS and heading logic, and high-level navigation intent;
- the Arduino Mega manages motors, local radio control, servo scanning, and deterministic control ownership.

The importance of this result is not merely that two processors were used, but that their relationship is now explicit and defensible. Earlier drafts overstated similarities with Salman et al. [17] by implying the same direct coupling already existed in that work. The updated dissertation corrects this: Salman et al. are an inspiration for dual-platform environmental robotics, but the explicit Pi-to-Mega control contract is a concrete contribution of the present implementation.

### 5.2.2 OBJ-2: Calibrated Environmental Monitoring

This objective was also achieved. The sensor subsystem now produces operator-facing values that are substantially more useful than raw counts. The discussion is important here because the value of the result lies less in absolute laboratory accuracy and more in practical interpretability.

Instead of a dashboard that merely shows rising ADC numbers, the final system shows:

- smoke-oriented MQ-2 readings,
- CO₂-oriented MQ-135 readings,
- MQ-7 carbon monoxide readings,
- and warning states tied to the same calibrated PPM channels.

This is a meaningful improvement over many low-cost prototypes. At the same time, the discussion must remain honest: MQ sensors are still approximate MOS sensors subject to drift, warm-up dependence, and cross-sensitivity, so the system should be understood as a field indicator and early-warning tool rather than as a certified gas analyser.

### 5.2.3 OBJ-3: Browser Dashboard for Monitoring and Control

This objective was achieved strongly and is one of the clearest practical successes of the project. The browser dashboard now functions as the main operating surface of the robot, integrating:

- sensor monitoring,
- map-based position awareness,
- camera viewing,
- manual drive controls,
- joystick bridge status,
- target and waypoint tools,
- and AI tools.

This matters because it shifts the robot away from the narrow usability of app-only or short-range-only designs. A browser dashboard, especially when reachable through LTE-backed connectivity, is a far more practical operating model for a field robot intended to keep humans away from hazardous or inconvenient locations.

### 5.2.4 OBJ-4: Assisted Navigation and Local Obstacle Handling

This objective was achieved, but with an important refinement in meaning. The project did implement waypoint assistance, heading acquisition, return functions, servo scanning, and local reactive avoidance. However, the final work also shows why such capability should not be over-interpreted.

The real lesson from development is that low-cost autonomy is fragile when it relies on:

- metre-scale GPS error,
- a magnetometer that must be carefully corrected,
- weak traction motors,
- and a single ultrasonic sensor, even when mounted on a servo.

For that reason, the dissertation now treats the objective as **assisted navigation with local safety behaviour**, not as proof of dependable unsupervised autonomy. This is a more rigorous and more academically honest interpretation of what the platform can presently do.

### 5.2.5 OBJ-5: Practical Manual Control Paths

This objective was achieved and is central to the final identity of the robot. The completed system now has two meaningful manual-control layers:

- **CC1101 local control** for nearby operator intervention;
- **dashboard-mediated joystick control** for browser-supervised teleoperation using the USB serial bridge.

This is a much more practical result than the outdated dual-wireless narrative that remained in the earlier dissertation draft. The important point is not redundant radios for their own sake. The important point is that the system now has a clear near-range local control path and a clear longer-range browser-mediated path.

### 5.2.6 OBJ-6: Integrated Prototype Validation

This objective was achieved at prototype level. The robot is no longer just a collection of separate modules. The current implementation demonstrates a coherent platform in which sensing, dashboard supervision, waypoint tools, manual control, tank-drive joystick control, Pi-Mega coordination, and local obstacle handling coexist in one operational system.

The phrase "at prototype level" matters. The integration is real, but the dissertation no longer pretends that this alone proves production-grade autonomy.

---

## 5.3 Discussion of Research Questions

### 5.3.1 RQ-1: How should the Raspberry Pi and Arduino Mega be partitioned?

The project shows that the most effective partition is not "Pi does everything" and not "Mega does everything." The better arrangement is:

- Raspberry Pi for communication, coordination, storage, dashboard access, and high-level navigation reasoning;
- Arduino Mega for motor ownership, local radio control, and deterministic obstacle reaction.

This partition keeps Linux flexibility where it is useful and keeps time-sensitive drive behaviour on the microcontroller where it is safer.

### 5.3.2 RQ-2: How useful is low-cost gas calibration in practice?

The calibrated gas readings materially improve usability. Even if the absolute values remain approximate, PPM-like displays and warning bands are much more interpretable than raw ADC numbers. This means the robot can now function more plausibly as an environmental warning platform rather than just a developer test rig.

### 5.3.3 RQ-3: What is the practical role of waypoint navigation with a single ultrasonic sensor?

The answer emerging from the final system is clear: waypoint navigation is useful as an assisted feature, but it should remain under supervision. The single servo-mounted ultrasonic sensor improves local awareness and is a worthwhile low-cost enhancement, but it does not provide the richness of perception needed for high-trust full autonomy in cluttered field conditions.

This question is central to the revised dissertation. The robot can acquire heading, track waypoints, and locally react to obstacles, but the responsible academic conclusion is that such navigation should be supervised, especially in variable outdoor environments.

### 5.3.4 RQ-4: Is a browser dashboard practical as the primary operator interface?

Yes. In fact, the updated work suggests that the browser dashboard is the most practically valuable part of the system. It unifies monitoring and control, is not tied to one device type, and extends operational reach when used over LTE-backed networking. This makes it more useful in practice than a short-range app-only interface.

### 5.3.5 RQ-5: How should local and remote manual control coexist?

The final system answers this by giving each path a clear role:

- CC1101 is the local direct manual path;
- the dashboard joystick bridge is the remote supervised manual path;
- and the Mega ownership machine ensures that only one source should own the motors at a time.

This is a more realistic answer than simply adding more radios. Practical operator control is achieved through clear ownership, not through confusing overlapping control links.

---

## 5.4 Comparison with the Literature

### 5.4.1 Comparison with Salman et al.

Salman et al. [17] remain the most important direct point of comparison because they demonstrated that low-cost ARM-based environmental robotics is feasible. However, the present project differs in several important ways that the revised dissertation must state accurately.

**Table 5.1: Updated Comparison with Salman et al. [17]**

| Aspect | Salman et al. [17] | Present Project |
|---|---|---|
| High/low-level processor concept | Raspberry Pi + Arduino used together | Raspberry Pi + Arduino Mega used together with explicit direct I2C coordination |
| Operator interface | Mobile / app-oriented control emphasis | Browser dashboard as the primary interface |
| Long-range practical supervision | Limited in reported form | LTE-backed browser access supported |
| Gas presentation | Literature commonly reports raw values or simpler treatment | Dashboard presents calibrated gas channels and warning bands |
| Local obstacle sensing | Lower-cost reactive concept | Servo-mounted ultrasonic scan with explicit 90/60/30 local rule |
| Local manual control | Short-range manual control concept | CC1101 local control plus dashboard USB joystick bridge |
| Autonomy claim in revised interpretation | Stronger autonomy emphasis in literature framing | Assisted navigation under supervision |

The comparison therefore supports the argument that the present project's main advancement is not simply "more features," but a more practical and more clearly partitioned operational architecture.

### 5.4.2 Comparison with Other Reviewed Systems

Compared with the wider literature reviewed in Chapter 2, the final system contributes a stronger integration of:

- browser supervision,
- calibrated gas presentation,
- Pi-to-Mega coordination,
- local RF control,
- internet-backed remote access,
- and servo-based local obstacle scanning.

However, the updated dissertation also makes a deliberate methodological correction: it does not claim that these features automatically produce a trustworthy autonomous robot. In that sense, the project advances the literature not only technically, but also by being more explicit about the limits of low-cost autonomy.

---

## 5.5 System Limitations

The updated system remains limited in several important ways.

**1. Perception limitation**
The robot still relies on a single ultrasonic sensor, even though it is now servo-mounted. This improves directional awareness but does not provide full environment understanding. Grass, angled surfaces, clutter, and narrow passages remain difficult cases.

**2. GPS and heading uncertainty**
The platform still inherits the limits of low-cost GPS and magnetometer operation. Assisted navigation is therefore useful, but precise or fully trustworthy autonomous travel is still outside the practical envelope of the robot.

**3. Gas accuracy limitation**
MQ-series sensors remain approximate and environmentally sensitive. They are useful for warning and trend observation, but they are not substitutes for certified gas instrumentation.

**4. Teleoperation dependence on communication quality**
The browser dashboard is powerful, but its practical value depends on network quality, dashboard host reliability, and clean serial behaviour in the joystick bridge path.

**5. Mechanical and traction limitations**
The robot uses low-cost DIY drive components. Turning performance and terrain capability are therefore limited by motor torque, wheel traction, and surface conditions.

**6. AI role remains limited**
AI Vision is present, but the project has deliberately moved away from treating it as the obstacle-safety authority. This is an honest limitation, not a failure: it reflects the present maturity of the perception stack.

---

## 5.6 Practical Implications

The project suggests several broader implications for low-cost environmental robotics.

First, **remote supervision over a browser dashboard may be more important in practice than stronger autonomy claims**. For an undergraduate robot with limited perception, a good human interface and good communication architecture can add more field usefulness than aggressive autonomy rhetoric.

Second, **explicit processor partitioning matters**. A Raspberry Pi and Arduino Mega combination becomes much more useful when the ownership and command boundaries are clear.

Third, **servo-mounted ultrasonic sensing is helpful, but should be framed correctly**. It is a reasonable low-cost improvement for local reactive behaviour, not a replacement for richer perception.

Fourth, **manual-control design deserves primary attention**. In real field use, the ability to intervene safely through CC1101 or a dashboard joystick path is not a backup feature; it is a core operational requirement.

---

## 5.7 Summary

This chapter has shown that the most valuable interpretation of the finished project is as a supervised environmental monitoring platform with assisted navigation, not as an overconfident autonomy demonstration. The revised discussion corrects earlier inaccuracies about wireless architecture, processor coupling, and obstacle sensing, and it shows that the real strengths of the robot lie in calibrated sensing, browser-based supervision, LTE-backed access, clear Pi-to-Mega coordination, and practical manual-control options. Chapter 6 concludes the dissertation on that basis.

---

*End of Chapter 5*
