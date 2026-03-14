# CHAPTER 3: METHODOLOGY

---

## 3.1 Introduction

This chapter presents the complete methodology employed in the design, implementation, and testing of the ARM-based Environmental Monitoring and Aide Robot. The chapter is structured to follow the natural sequence of the systems engineering process: from hardware architecture and component selection, through circuit design and mechanical integration, to software architecture, algorithm design, communication protocol design, and finally the testing methodology used to validate each subsystem. Where design decisions were made, the rationale is provided and alternatives are discussed. Specific numerical parameters, pin assignments, and calibration constants are presented in full to ensure reproducibility.

---

## 3.2 Research Design and Systems Engineering Approach

The project followed a systems engineering approach structured around four iterative phases: requirements definition, architecture design, implementation, and validation. The requirements definition phase identified the key performance and functional requirements from the research objectives defined in Chapter 1, producing a set of system-level requirements against which the final implementation is validated in Chapter 4. Table 3.1 summarises the top-level system requirements.

**Table 3.1: Top-Level System Requirements**

| ID | Requirement | Category |
|---|---|---|
| SR-01 | The system shall autonomously navigate GPS-defined waypoints without human intervention | Functional |
| SR-02 | The system shall measure temperature, humidity, CO, CO₂/air quality, and smoke/LPG concentrations | Functional |
| SR-03 | The system shall report gas concentrations in calibrated PPM values | Functional |
| SR-04 | The system shall detect and avoid obstacles without operator intervention | Functional |
| SR-05 | The system shall transmit all sensor data to a web dashboard accessible from any browser | Functional |
| SR-06 | The system shall operate over both local Wi-Fi and LTE cellular networks | Functional |
| SR-07 | The system shall maintain robot controllability if one wireless link fails | Functional |
| SR-08 | The system shall update the dashboard with sensor data at no less than 0.2 Hz (every 5 seconds) | Performance |
| SR-09 | The system shall achieve waypoint approach accuracy within the GPS receiver's CEP (≤ 3 m) | Performance |
| SR-10 | The total component cost shall not exceed USD 150 | Constraint |
| SR-11 | The system shall operate from a 12 V DC power source | Constraint |

The architecture design phase partitioned the system into five major subsystems: the sensing subsystem, the navigation subsystem, the communication subsystem, the data management subsystem, and the human-machine interface (dashboard). Each subsystem was then designed in detail, with interfaces between subsystems explicitly defined before implementation began. This approach minimised integration risk and ensured that hardware and software components developed by different modules were compatible at their interfaces.

---

## 3.3 Hardware Architecture

### 3.3.1 System Block Diagram

The overall system architecture follows a dual-processor hierarchy, as illustrated by the conceptual block diagram described below. The Raspberry Pi 3 Model B serves as the high-level controller, responsible for environmental sensing, navigation mathematics, internet communication, and the local web server. The Arduino Mega 2560 serves as the real-time low-level controller, responsible for motor drive PWM, GPS serial parsing, compass reading, servo actuation, and obstacle sensor interfacing. The two processors communicate over an I2C bus at 400 kHz (fast mode), with the Raspberry Pi as I2C master and the Arduino Mega as I2C slave at address 0x08.

```
[INSERT FIGURE 3.1: System block diagram showing the dual-processor hierarchy,
subsystem partitioning, and inter-system communication links]
```

The key architectural principle is separation of concerns: the Arduino Mega provides hard real-time guarantees for motor control and direct sensor interfacing that would be difficult to achieve on a Linux-based system subject to operating system scheduling preemption, while the Raspberry Pi provides the processing power and operating system environment required for TCP/IP networking, Python-based navigation mathematics, and database management.

### 3.3.2 Raspberry Pi 3 Model B

The Raspberry Pi 3 Model B (RPi 3B) was selected as the high-level controller based on its Broadcom BCM2837 64-bit ARMv8 quad-core processor at 1.2 GHz, 1 GB LPDDR2 RAM, and 40 GPIO pins. The Raspbian OS (Debian-based Linux) provides the Python 3 runtime, package management (pip), and kernel-level I2C and SPI drivers required by the software stack. The integrated 802.11b/g/n Wi-Fi adapter provides the local network interface for dashboard access without requiring an additional Wi-Fi dongle. Table 3.2 summarises the GPIO pin assignments used on the Raspberry Pi.

**Table 3.2: Raspberry Pi GPIO Pin Assignments**

| GPIO Pin (BCM) | Function | Connected Device |
|---|---|---|
| GPIO 4 | 1-Wire / digital input | DHT11 data pin |
| GPIO 2 (SDA) | I2C SDA (hardware) | ADS1115 (0x48) + Arduino Mega (0x08) |
| GPIO 3 (SCL) | I2C SCL (hardware) | ADS1115 (0x48) + Arduino Mega (0x08) |
| USB Port 0 | USB-Serial /dev/ttyUSB0 | SIM7600E LTE module |
| CSI Connector | MIPI CSI-2 camera bus | Pi Camera Module V2 |

### 3.3.3 Arduino Mega 2560

The Arduino Mega 2560, based on the Atmel ATmega2560 8-bit AVR microcontroller, was selected as the real-time navigation controller. Its four hardware UART ports (Serial0–3), 15 PWM-capable pins, and large SRAM (8 KB) and Flash (256 KB) make it well-suited to simultaneously managing GPS serial parsing, wireless serial communication, motor PWM, servo actuation, and I2C slave service. Table 3.3 summarises the pin assignments on the Arduino Mega.

**Table 3.3: Arduino Mega 2560 Pin Assignments**

| Pin(s) | Function | Connected Device |
|---|---|---|
| Serial1: TX1(18) / RX1(19) | UART GPS | NEO-6M GPS module |
| Serial2: TX2(16) / RX2(17) | UART ZigBee | XBee ZigBee module (primary wireless) |
| Serial3: TX3(14) / RX3(15) | UART Bluetooth | HC-05 Bluetooth (optional) |
| SDA(20) / SCL(21) | I2C hardware bus (slave) | Raspberry Pi master |
| Pins 40 / 41 | Software I2C (bit-bang) | QMC5883L compass (SDA / SCL) |
| Pin 8 | Digital output (TRIG) | HC-SR04 ultrasonic trigger |
| Pin 9 | Digital input (ECHO) | HC-SR04 ultrasonic echo |
| Pin 11 | PWM servo signal | SG90 servo motor |
| Pin 2 | Digital interrupt (INPUT_PULLUP) | KY-032 IR obstacle digital output |
| Pin A0 | Analogue input | KY-032 IR obstacle analogue output |
| SPI (MOSI 51, MISO 50, SCK 52, SS 53) | SPI bus | CC1101 wireless module |
| Motor A: pins 6, 7, 5 | PWM + direction | L298N motor driver (left motor) |
| Motor B: pins 3, 4, 2 | PWM + direction | L298N motor driver (right motor) |

### 3.3.4 Power Distribution

The system is powered from a 12 V DC source (rechargeable lithium polymer battery pack, nominal 11.1 V, 3S). Power is distributed as follows:

- **12 V rail:** L298N motor driver (drives 12 V DC gear motors directly)
- **5 V rail (derived from L298N onboard 5 V regulator):** Arduino Mega, SG90 servo, HC-SR04, KY-032, CC1101 module, XBee module, NEO-6M GPS, QMC5883L compass
- **5 V USB rail (from separate buck converter or Pi USB port):** MQ-2, MQ-135, MQ-7 heater circuits and ADS1115 ADC
- **5 V from RPi 5V pin:** ADS1115 I2C supply
- **3.3 V from RPi 3V3 pin:** DHT11

The MQ-series sensors draw up to 800 mA total at 5 V during heater warm-up and were therefore supplied from a dedicated buck converter rather than the Arduino's onboard 5 V regulator to prevent voltage sag affecting the microcontroller.

```
[INSERT FIGURE 3.2: Power distribution schematic showing voltage rails,
buck converters, and per-component power connections]
```

---

## 3.4 Sensor Subsystem Design

### 3.4.1 DHT11 Temperature and Humidity Sensor

The DHT11 sensor is connected directly to BCM GPIO 4 of the Raspberry Pi, using the single-wire protocol library `Adafruit_DHT`. A 10 kΩ pull-up resistor is connected between the data line and the 3.3 V supply rail. The sensor driver (`raspberry_pi/sensors/dht_sensor.py`) imposes a minimum sampling interval of 2 seconds to comply with the DHT11's protocol timing requirements, validates readings against physical bounds (temperature: −40 to 85°C; humidity: 0–100% RH), and retries up to three times on communication failure before returning a `None` value to the calling thread. The sensor is sampled every 5 seconds by the main sensor acquisition thread.

### 3.4.2 ADS1115 Analogue-to-Digital Converter

Because the Raspberry Pi has no native analogue input pins, a Texas Instruments ADS1115 16-bit, 4-channel, delta-sigma ADC is used to digitise the analogue output voltages of the three MQ-series gas sensors. The ADS1115 communicates over I2C at address 0x48 (ADDR pin tied to GND). The device is configured with a programmable gain amplifier (PGA) setting of 2/3×, yielding a full-scale input range of ±6.144 V across a 15-bit single-ended measurement (32,767 counts full scale), corresponding to 0.1875 mV per LSB. This range is appropriate for the MQ-sensor output voltage span of 0–5 V at 5 V supply. The Adafruit_ADS1x15 Python library is used for device control. The three gas sensors are assigned to channels as shown in Table 3.4.

**Table 3.4: ADS1115 Channel Assignments**

| ADS1115 Channel | Sensor | Target Gas |
|---|---|---|
| A0 | MQ-2 | Smoke, LPG, combustible gases |
| A1 | MQ-135 | CO₂, NH₃, NOx, air quality |
| A2 | MQ-7 | Carbon monoxide (CO) |
| A3 | Reserved | — |

### 3.4.3 MQ-Series Gas Sensors

All three MQ-series sensors use a tin dioxide (SnO₂) metal oxide sensing element heated to 300–400°C by an internal resistive heater. The sensor output is taken across a fixed load resistor (RL = 1000 Ω on the breakout board) in series with the sensing element between the 5 V supply and ground. The output voltage (V_out) across RL is:

$$V_{out} = V_{IN} \cdot \frac{R_L}{R_S + R_L}$$

where $R_S$ is the sensor resistance, which varies with gas concentration. Rearranging gives the sensor resistance:

$$R_S = \left(\frac{V_{IN} \cdot R_L}{V_{out}}\right) - R_L$$

The sensors require a pre-heat time of approximately 24–48 hours for initial burn-in and a 20-second warm-up period at each power-on before readings stabilise. This warm-up delay is implemented in the `SensorManager` initialisation routine.

```
[INSERT FIGURE 3.3: Circuit schematic for one MQ sensor showing the
ADS1115 connection, load resistor RL, and supply voltage]
```

### 3.4.4 Gas Sensor PPM Calibration

Conversion of raw ADS1115 ADC counts to calibrated parts-per-million (PPM) gas concentrations is performed by the `GasCalibration` class in `raspberry_pi/utils/gas_calibration.py`. The methodology follows the log-log power law model derived from manufacturer datasheets, as described in Section 2.4.3. The full calculation chain is as follows:

**Step 1 — ADC to voltage:**

$$V_{out} = \text{ADC}_{\text{raw}} \times \frac{6.144 \text{ V}}{32767}$$

**Step 2 — Voltage to sensor resistance:**

$$R_S = \left(\frac{5.0 \times 1000}{V_{out}}\right) - 1000 \quad [\Omega]$$

**Step 3 — Estimate baseline resistance in clean air:**

$$R_0 = \frac{R_S}{\text{ratio}_{\text{clean air}}}$$

where $\text{ratio}_{\text{clean air}}$ is the characteristic Rs/R₀ value measured in clean air, determined from the sensor datasheet and incorporated as a calibration constant.

**Step 4 — Compute PPM concentration:**

$$\text{PPM} = a \cdot \left(\frac{R_S}{R_0}\right)^b$$

The calibration constants $a$, $b$, and $\text{ratio}_{\text{clean air}}$ for each sensor and target gas are tabulated in Table 3.5. These constants were derived by fitting a log-log linear regression to the characteristic sensitivity curves published in the respective sensor datasheets.

**Table 3.5: Gas Sensor Calibration Constants**

| Sensor | Target Gas | a | b | R₀ ratio (clean air) |
|---|---|---|---|---|
| MQ-2 | Smoke | 3697.4 | −3.109 | 9.83 |
| MQ-2 | LPG | 2000.0 | −2.95 | 9.83 |
| MQ-135 | CO₂ | 110.47 | −2.862 | 3.60 |
| MQ-135 | NH₃ | 102.2 | −2.473 | 3.60 |
| MQ-7 | CO | 99.04 | −1.518 | 27.5 |

The implementation also provides a sensor status classification (Table 3.6) based on raw ADC thresholds, enabling fast-path alerting without requiring the full PPM computation to be displayed.

**Table 3.6: Sensor Status Classification Thresholds (Raw ADC Counts)**

| ADC Range | Status Label |
|---|---|
| < 500 | Very Low (Check wiring) |
| 500 – 1999 | Clean |
| 2000 – 9999 | Normal |
| 10000 – 19999 | Elevated |
| ≥ 20000 | High Alert |

### 3.4.5 Pi Camera Module V2

The Pi Camera Module V2 features an 8-megapixel Sony IMX219 sensor connected to the Raspberry Pi via the MIPI CSI-2 serial interface. In the robot application it is configured for live MJPEG streaming rather than still photography. The `CameraStream` class (`raspberry_pi/camera/camera_stream.py`) uses the `picamera` library to capture frames into an in-memory `BytesIO` buffer at a resolution of 320×240 pixels, 10 frames per second, JPEG quality 85. A companion `MJPEGServer` class serves the MJPEG stream over HTTP on a dedicated port; the dashboard relays this stream to browser clients. The low resolution is deliberately chosen to minimise network bandwidth consumption while maintaining adequate situational awareness for the operator.

---

## 3.5 Navigation Subsystem Design

### 3.5.1 NEO-6M GPS Module

The u-blox NEO-6M GPS receiver is connected to the Arduino Mega's Serial1 port (hardware UART, TX1 pin 18, RX1 pin 19) at 9600 baud. The TinyGPSPlus Arduino library parses the NMEA 0183 sentence stream (specifically the GPRMC and GPGGA sentences), extracting latitude, longitude, altitude, speed over ground, number of satellites tracked, and HDOP (horizontal dilution of precision). The GPS is initialised at system startup and the firmware waits for a valid fix (≥ 4 satellites) before allowing autonomous navigation to begin. GPS data is broadcast by the Arduino Mega to the Raspberry Pi over I2C on request using the `CMD_REQUEST_GPS` command, encoded as a 17-byte payload comprising a validity flag, four single-precision floating-point values (latitude, longitude, altitude, speed), and a satellite count byte.

### 3.5.2 SIM7600E Integrated GPS

The SIM7600E LTE module, connected to the Raspberry Pi via USB (/dev/ttyUSB0), provides a secondary GPS receiver as well as LTE cellular internet connectivity. The `SIM7600EGPS` class (`raspberry_pi/communication/sim7600e_gps.py`) communicates with the module using AT commands. The `AT+CGPS=1,1` command activates the integrated GPS, and `AT+CGPSINFO` retrieves the current position. The SIM7600E GPS is used as the primary position source for the Raspberry Pi's navigation controller (NavController), while the NEO-6M on the Arduino Mega provides a secondary redundant position source. In the event that the SIM7600E GPS loses fix, the navigation controller falls back automatically to the NEO-6M position data polled via I2C.

### 3.5.3 QMC5883L Magnetometer (Compass)

The QMC5883L triple-axis magnetometer is electrically compatible with the HMC5883L used by Salman et al. [17] and provides a 16-bit magnetic field measurement on three axes. To avoid electromagnetic interference from the Arduino Mega's motor PWM lines and from the L298N motor driver, the compass is connected on a dedicated software I2C bus implemented by bit-banging on Arduino digital pins 40 (SDA) and 41 (SCL) rather than on the hardware I2C bus (pins 20/21) used for Raspberry Pi communication. This physical isolation eliminates the possibility of motor driver electromagnetic interference corrupting compass readings. The compass is also physically positioned on the robot chassis at maximum distance from ferromagnetic components, consistent with the recommendation of Salman et al. [17].

The compass heading is computed from the raw X and Y magnetic field components using:

$$\theta_{\text{mag}} = \arctan2(B_y,\; B_x) \times \frac{180°}{\pi}$$

A magnetic declination correction is applied for the deployment location to convert magnetic north to true north. The corrected heading is used by the navigation controller for bearing error computation.

### 3.5.4 Motor Drive System

The L298N dual H-bridge motor driver module controls two 6 V DC gear motors providing differential drive. The Arduino Mega generates PWM signals on its Timer-1 and Timer-3 channels for the left and right motors respectively, enabling independent speed and direction control. Motor speed is encoded as an unsigned 8-bit PWM duty cycle (0–255), and direction as a two-bit enable/direction combination on the H-bridge logic inputs. The differential drive equations for forward motion, rotation, and turning are:

| Manoeuvre | Left Motor | Right Motor |
|---|---|---|
| Forward | +PWM | +PWM |
| Reverse | −PWM | −PWM |
| Rotate Left | −PWM | +PWM |
| Rotate Right | +PWM | −PWM |
| Curve Left | +PWM × 0.5 | +PWM |
| Curve Right | +PWM | +PWM × 0.5 |

All motor state changes pass through a central `MotorControl` class that enforces a minimum stop pulse (10 ms) between direction reversals to prevent current spikes in the H-bridge.

### 3.5.5 Obstacle Detection Hardware

**HC-SR04 Ultrasonic Sensor:** The HC-SR04 is mounted on the shaft of an SG90 micro-servo (Arduino pin 11) that enables rotation over a 160° arc (0° = full right, 90° = forward, 160° = full left). Distance measurement uses the Arduino's `pulseIn()` function to measure the echo pulse width, converting to distance by:

$$d\;[\text{cm}] = \frac{\text{pulse width}\;[\mu\text{s}]}{58.0}$$

The measurement is bounded to the HC-SR04's reliable operating range of 2–400 cm. Readings below 30 cm in the forward-facing orientation trigger the obstacle avoidance state machine.

**KY-032 IR Proximity Sensor:** The KY-032 sensor provides a digital active-low output (Arduino pin 2, configured as INPUT_PULLUP with an external interrupt) and an analogue proximity voltage (Arduino pin A0). The digital interrupt pin is configured as a falling-edge interrupt using `attachInterrupt()`, enabling an immediate motor halt within one interrupt service routine cycle (~4 µs) of obstacle detection at very close range (< 5 cm). This sub-millisecond response provides a critical safety net for situations where the robot approaches an obstacle faster than the HC-SR04 polling rate can resolve.

```
[INSERT FIGURE 3.4: Physical layout diagram showing HC-SR04 servo mount,
KY-032 IR sensor position, and field-of-view coverage diagram]
```

---

## 3.6 Software Architecture

### 3.6.1 Raspberry Pi Software Architecture

The Raspberry Pi software is structured as a multi-threaded Python 3 application (entry point: `raspberry_pi/main.py`, 1,398 lines). The `RobotController` class owns all subsystem instances and launches seven concurrent threads, each performing a specific periodic task. The use of Python's `threading.Thread` class with a shared `threading.Event` (`shutdown_event`) provides clean lifecycle management: all threads check the shutdown event and exit gracefully on `SIGINT` or `SIGTERM`. Thread-safety for shared data structures (sensor readings, GPS position, command queue) is enforced through `threading.Lock` objects. Table 3.7 describes each thread.

**Table 3.7: Raspberry Pi Concurrent Threads**

| Thread | Frequency | Responsibility |
|---|---|---|
| Sensor Thread | 0.2 Hz (every 5 s) | DHT11 + MQ-2/135/7 via ADS1115, PPM conversion, REST POST to dashboard |
| GPS Thread | 0.5 Hz (every 2 s) | Poll SIM7600E GPS, REST POST to dashboard |
| Status Thread | 0.1 Hz (every 10 s) | Battery level, signal strength, system info, REST POST to dashboard |
| Camera Thread | 5 FPS | Capture MJPEG frame, push to MJPEGServer buffer |
| Waypoint Thread | 0.2 Hz (every 5 s) | Poll dashboard for new waypoints, push to NavController |
| Command Thread | 0.5 Hz (every 2 s) | Poll dashboard command queue, dispatch to NavController or I2CComm |
| Instant Command Thread | 10 Hz (every 100 ms) | Poll dashboard for immediate commands (emergency stop, manual override) |

The `NavController` runs its own internal thread at 10 Hz, independent of the threads above. Configuration is loaded at startup from `raspberry_pi/config.json`, with environment variable `$ROBOT_CONFIG_PATH` allowing override for testing. Figure 3.5 illustrates the thread interaction model.

```
[INSERT FIGURE 3.5: Thread interaction diagram showing the RobotController
threads, shared data structures, and inter-thread communication patterns]
```

### 3.6.2 Module Structure

The Raspberry Pi software is organised into the following module packages:

- **`sensors/`**: `sensor_manager.py` (orchestrates all sensors), `dht_sensor.py`, `mq_sensors.py`, `compass.py`
- **`communication/`**: `i2c_comm.py` (I2C to Mega), `api_client.py` (REST to dashboard), `gsm_module.py` (AT commands), `sim7600e_gps.py`, `cloud_uploader.py` (ThingSpeak/Blynk)
- **`navigation/`**: `nav_controller.py` (navigation state machine and geodesic calculations)
- **`camera/`**: `camera_stream.py`, `mjpeg_server.py`
- **`utils/`**: `gas_calibration.py`, `logger.py`, `data_formatter.py`, `qmi_health.py`

### 3.6.3 Arduino Mega Firmware Architecture

The Arduino firmware (`arduino_mega/robot_navigation/robot_navigation.ino`, 1,127 lines) follows a strict non-blocking superloop architecture. The use of `delay()` is prohibited throughout the codebase; all timing is managed through non-blocking timestamp comparisons using `millis()`. This ensures that the I2C slave interrupt service routine (ISR), GPS serial parsing, CC1101 wireless polling, and motor control all receive CPU time on each loop iteration without any single task blocking others.

The firmware is partitioned into the following modules (`.h`/`.cpp` file pairs):

- **`gps_handler`**: TinyGPSPlus-based NMEA parsing from Serial1
- **`navigation`**: Haversine geodesic, bearing computation, waypoint management
- **`motor_control`**: L298N H-bridge PWM abstraction
- **`obstacle_avoidance`**: HC-SR04 servo-scan logic, KY-032 interrupt handler
- **`cc1101_driver`**: SPI-based CC1101 transceiver driver
- **`wireless_interface`**: Command parsing from the CC1101 packet stream
- **`globals`**: Shared state declarations (robot state, pending waypoints, I2C buffers)

### 3.6.4 Arduino Three-State Machine

The Arduino firmware implements a three-state machine governing motor ownership. The three states are mutually exclusive, and only one is active at any time. Table 3.8 defines the states and their transition logic.

**Table 3.8: Arduino Mega Three-State Machine**

| State | Motor Owner | Entry Condition | Exit Condition |
|---|---|---|---|
| `STATE_I2C` (AUTO) | Raspberry Pi | No wireless activity; Pi heartbeat present | Wireless command received |
| `STATE_WIRELESS` | CC1101 remote | Wireless command received | Wireless timeout (no packet for 500 ms) |
| `STATE_FAILSAFE` | None (motors halted) | Both Pi I2C and wireless lost simultaneously | Either channel recovers |

The Mode Manager function, executed at the top of every superloop iteration, evaluates the timestamps of the most recent I2C activity and the most recent CC1101 packet, and transitions the state machine accordingly. Wireless control always takes priority over autonomous (I2C) control, enabling the operator to take manual control at any time by simply transmitting a command on the wireless link. On every state transition, motors are halted and a non-blocking buzzer pattern is sounded to provide audible feedback.

The superloop execution order is fixed as follows:
1. Deferred CC1101 initialisation (runs once on first iteration after a configurable startup delay)
2. SPI service — poll CC1101, update wireless command buffer
3. I2C service — process any deferred ISR command from the Raspberry Pi
4. Mode Manager — evaluate and apply state transitions
5. Sensor tasks — GPS serial parsing, ultrasonic distance measurement
6. Buzzer tick — non-blocking buzzer pulse sequencer
7. I2C bus watchdog — detect and recover stuck I2C bus
8. Decision Layer — execute behaviour for current state (auto navigation OR manual control OR failsafe hold)
9. Obstacle buzzer — proximity alert
10. Status print — periodic serial debug output
11. GPS broadcast — periodic wireless GPS position broadcast

### 3.6.5 Dashboard Web Application

The dashboard is implemented as a Python Flask web application (`dashboard/app.py`, 1,826 lines) with Flask-SocketIO for real-time WebSocket communication, Flask-CORS for cross-origin access, and SQLAlchemy ORM for database access. The application architecture follows the Model-View-Controller (MVC) pattern: SQLAlchemy models (`database.py`) define the data layer, Jinja2 HTML templates (`templates/index.html`) define the view layer, and Flask route functions and SocketIO event handlers define the controller layer.

The dashboard exposes the following REST API endpoints to the Raspberry Pi:

**Table 3.9: Dashboard REST API Endpoints**

| Method | Endpoint | Function |
|---|---|---|
| POST | `/api/sensor-data` | Receive sensor readings from robot |
| POST | `/api/gps-data` | Receive GPS position from robot |
| POST | `/api/status` | Receive robot status update |
| GET | `/api/waypoints` | Robot polls for pending waypoints |
| POST | `/api/waypoints` | Operator submits new waypoints |
| GET | `/api/commands` | Robot polls command queue |
| POST | `/api/commands` | Operator submits command |
| GET | `/api/sensor-history` | Historical sensor data (with time range filter) |
| GET | `/api/gps-history` | Historical GPS track data |
| GET | `/stream/camera` | MJPEG camera stream relay |

On receipt of sensor or GPS data, the Flask controller emits a WebSocket event (`sensor_update` or `gps_update`) via Flask-SocketIO, delivering the new data to all connected browser clients within milliseconds of arrival.

### 3.6.6 Dashboard Front-End

The dashboard front-end is a single-page application (SPA) implemented in HTML5, CSS3, and vanilla JavaScript within `dashboard/templates/index.html` (672 lines). Three JavaScript libraries are loaded via CDN:

- **Socket.IO client** — maintains the WebSocket connection to Flask-SocketIO and handles `sensor_update` and `gps_update` events
- **Leaflet.js** — renders an interactive OpenStreetMap tile layer on which the robot's GPS position is displayed as a marker, the GPS track history is drawn as a polyline, and waypoints can be placed by clicking on the map
- **Chart.js** — renders real-time scrolling time-series charts for temperature, humidity, CO PPM, CO₂ PPM, and smoke PPM

The dashboard layout is divided into a map panel (full left half), a camera feed panel (top right), sensor readings cards (middle right), and a control panel (bottom right) providing buttons for navigation commands (Start, Stop, Pause, Resume, Emergency Stop, Return to Start) and a joystick widget for manual control.

```
[INSERT FIGURE 3.6: Dashboard screenshot showing map panel, sensor cards,
camera feed, Chart.js trend graphs, and control panel layout]
```

### 3.6.7 Database Design

The SQLite database (`instance/robot_data.db`), accessed via SQLAlchemy, implements the schema defined in `database/schema.sql`. Table 3.10 summarises the six database tables and their key columns.

**Table 3.10: Database Schema Summary**

| Table | Key Columns | Purpose |
|---|---|---|
| `sensor_readings` | timestamp, device_id, temperature, humidity, mq2, mq135, mq7 | Environmental sensor history |
| `gps_locations` | timestamp, device_id, latitude, longitude, altitude, speed, satellites | GPS position history |
| `waypoints` | sequence, latitude, longitude, completed, timestamp | Waypoint queue and completion status |
| `robot_status` | timestamp, online, battery_level, signal_strength, system_info | Robot health monitoring |
| `event_logs` | timestamp, event_type, category, message | System events and alerts |
| `robot_commands` | timestamp, command, parameters, executed | Command queue |

Indexes are defined on the `timestamp` and `device_id` columns of the sensor and GPS tables to optimise time-range queries used by the historical chart endpoint. A configurable data retention policy (`dashboard/config.py`) automatically purges records older than 30 days for sensor and GPS data, and older than 90 days for event logs, preventing unbounded database growth.

---

## 3.7 Communication Protocol Design

### 3.7.1 I2C Inter-Processor Protocol

The I2C protocol between the Raspberry Pi (master) and Arduino Mega (slave at 0x08) was designed as a lightweight command-response protocol. All communication is initiated by the Raspberry Pi. A command is transmitted as a variable-length byte sequence: the first byte is the command opcode, followed by zero or more payload bytes specific to that command. The Arduino acknowledges most commands with an ACK byte (0x80) and, for data-returning commands (GPS, status, obstacle), follows the ACK with a structured payload. Table 3.11 summarises the full command set.

**Table 3.11: I2C Command Set**

| Opcode (ASCII) | Command | Payload | Response |
|---|---|---|---|
| `P` (0x50) | CMD_PING | None | ACK |
| `S` (0x53) | CMD_NAV_START | None | ACK |
| `T` (0x54) | CMD_NAV_STOP | None | ACK |
| `A` (0x41) | CMD_NAV_PAUSE | None | ACK |
| `R` (0x52) | CMD_NAV_RESUME | None | ACK |
| `C` (0x43) | CMD_WAYPOINT_CLEAR | None | ACK |
| `W` (0x57) | CMD_WAYPOINT_PACKET | 9 bytes (index, lat float, lon float) | ACK |
| `M` (0x4D) | CMD_WAYPOINT_COMMIT | None | ACK |
| `G` (0x47) | CMD_REQUEST_GPS | None | GPS payload (17 bytes) |
| `U` (0x55) | CMD_REQUEST_STATUS | None | Status payload (8 bytes) |
| `O` (0x4F) | CMD_REQUEST_OBSTACLE | None | Obstacle payload (3 bytes) |
| `V` (0x56) | CMD_MANUAL_OVERRIDE | 2 bytes (left PWM, right PWM) | ACK |
| `E` (0x45) | CMD_EMERGENCY_STOP | None | ACK |
| `B` (0x42) | CMD_RETURN_TO_START | None | ACK |
| `F` (0x46) | CMD_SEND_GPS | 8 bytes (lat float, lon float) | ACK |
| `D` (0x44) | CMD_SEND_HEADING | 4 bytes (heading float) | ACK |
| `H` (0x48) | CMD_HEARTBEAT | None | ACK |

The GPS payload (17 bytes) is encoded as: `[valid_flag (1 byte)] [latitude (4 bytes, IEEE 754 float)] [longitude (4 bytes)] [altitude (4 bytes)] [speed (4 bytes)] [satellites (1 byte) ]`.

The status payload (8 bytes) encodes: `[mode (1)] [nav_active (1)] [manual_override (1)] [waypoint_count (1)] [battery_percent (1)] [signal_quality (1)] [current_waypoint (1)] [waypoint_completed (1)]`.

Waypoint transmission uses a three-step atomic protocol: `CMD_WAYPOINT_CLEAR` resets the pending waypoint buffer on the Arduino; `CMD_WAYPOINT_PACKET` transmits one waypoint at a time (index + lat/lon encoded as IEEE 754 floats); `CMD_WAYPOINT_COMMIT` atomically activates the new waypoint list. This sequence ensures that the navigation system never operates on a partially-loaded waypoint set.

The I2C bus is protected by a `threading.Lock` on the Raspberry Pi side, preventing concurrent access from the sensor, GPS, and navigation threads. An I2C bus watchdog function on the Arduino Mega detects clock line lockup (I2C clock held low for > 25 ms by a failed transaction) and executes a nine-cycle clock recovery sequence by bit-banging the SCL line, followed by re-initialisation of the Wire library slave mode.

### 3.7.2 REST and WebSocket Communication

The Raspberry Pi communicates with the dashboard exclusively via HTTP REST API calls, using the Python `requests` library. Each sensor acquisition cycle POSTs a JSON payload to `/api/sensor-data`; each GPS cycle POSTs to `/api/gps-data`. The `DashboardAPI` class (`raspberry_pi/communication/api_client.py`) wraps all REST calls with retry logic (up to 3 attempts with exponential back-off) and buffering: if the dashboard is unreachable, readings are queued in memory and replayed when connectivity is restored.

Real-time browser updates are delivered via Flask-SocketIO WebSocket events. On receipt of a REST POST, the Flask server immediately emits a corresponding SocketIO event to all connected browser clients, typically delivering data to the browser within 20–50 ms of the Raspberry Pi's POST. This eliminates the 15-second polling minimum of cloud-based ThingSpeak and enables smooth real-time chart animation.

### 3.7.3 Wireless Communication Architecture

The wireless communication system implements a dual-channel redundancy architecture with automatic priority and failsafe behaviour:

**Primary Channel — ZigBee (XBee):** An XBee ZigBee module connected to the Arduino Mega's Serial2 (pins 16/17) at 57,600 baud provides the primary wireless link for manual control commands and telemetry. ZigBee provides a nominal outdoor range of 100 m with standard antennas, extendable to over 1,500 m with the XBee Pro variant.

**Secondary Channel — CC1101 Sub-GHz RF:** A CC1101 transceiver module connected to the Arduino Mega's SPI bus (MOSI 51, MISO 50, SCK 52, SS 53) provides a secondary wireless link on the 433 MHz or 868 MHz ISM band, with a typical outdoor range of 200–500 m. The CC1101 driver is initialised with a deferred startup routine to avoid SPI bus contention during boot.

**Priority Logic:** The Mode Manager checks the timestamp of the most recent received wireless packet on either channel. If a packet was received within the last 500 ms on either channel, the state machine enters `STATE_WIRELESS` and manual control takes effect. Once both channels have been silent for 500 ms, the state machine returns to `STATE_I2C` (if the Pi heartbeat is present) or `STATE_FAILSAFE` (if the Pi has also timed out).

**Cellular (LTE):** The SIM7600E module provides internet connectivity for dashboard access from outside the local Wi-Fi range and for optional cloud upload to ThingSpeak. AT command sequences manage connection establishment with the configured APN (`Econet` in `config.json`), with automatic reconnection on link loss.

---

## 3.8 Navigation Algorithm Design

### 3.8.1 Pi-Side Navigation Controller

All navigation mathematics is executed on the Raspberry Pi within the `NavController` class (`raspberry_pi/navigation/nav_controller.py`, 893 lines). This architectural decision places navigation computation where the compass sensor resides (on the Raspberry Pi's I2C bus via the `Compass` class), eliminating a round-trip I2C latency that would occur if heading data had to be fetched from the Arduino before a navigation decision could be made. The Arduino Mega acts as a motor driver slave, executing only simple drive commands (`CMD_MANUAL_OVERRIDE` with left and right PWM values) issued by the NavController.

The NavController runs at 10 Hz (NAV_LOOP_HZ = 10) in its own dedicated thread. A `threading.Lock` protects all state variable access, allowing the waypoint and command threads to safely update the waypoint list and state while the navigation loop is running.

### 3.8.2 Navigation State Machine

The navigation state machine implements eleven states as shown in Table 3.12 and illustrated in Figure 3.7.

**Table 3.12: Navigation State Machine States**

| State | Duration | Description |
|---|---|---|
| `IDLE` | Indefinite | Navigation inactive; robot stationary |
| `PREPARING` | 3 s | Short hold before heading acquisition; provides user review window |
| `ACQUIRING_HEADING` | Up to 30 s | Robot rotates slowly until compass heading stabilises and matches GPS satellite bearing; timeout → IDLE |
| `HEADING_ACQUIRED` | 10 s | Countdown hold before forward drive begins; heading confirmed |
| `NAVIGATING` | Until waypoint reached | Robot drives toward current waypoint; compass heading continuously corrected |
| `OBSTACLE_DETECTED` | < 0.5 s | Immediate motor halt; transition to avoidance sequence |
| `OBSTACLE_AVOID` | ~3 s | Servo-scan, direction selection, rotation, clear-path recheck |
| `WAYPOINT_REACHED` | 3 s | Robot stops; notification emitted; dashboard updated; proceed to next waypoint or COMPLETE |
| `COMPLETE` | Indefinite | All waypoints reached; robot stops |
| `PAUSED` | Indefinite | Operator-initiated pause; state is preserved for resume |

```
[INSERT FIGURE 3.7: Navigation state machine diagram showing all states,
transitions, trigger conditions, and timing parameters]
```

### 3.8.3 Geodesic Calculations

Distance and bearing to the target waypoint are computed on every NavController loop iteration. The implementation preferentially uses the GeographicLib library's implementation of Karney's algorithm on the WGS-84 ellipsoid:

```python
result = _geod.Inverse(lat1, lon1, lat2, lon2)
distance = result['s12']   # metres
bearing  = result['azi1']  # degrees, true north reference
```

If GeographicLib is unavailable (import fails at startup), the controller falls back to the Haversine formula for distance and the forward azimuth formula for bearing, implemented in pure Python with no external dependencies. The NavController logs which geodesic library is in use at startup to facilitate debugging.

A waypoint is considered "reached" when the computed distance falls below `WAYPOINT_RADIUS = 3.0 m`, consistent with the NEO-6M receiver's 2.5 m CEP specification.

### 3.8.4 Heading Correction and Motor Steering

During the `NAVIGATING` state, the heading error $\epsilon$ is computed on each 10 Hz iteration as:

$$\epsilon = \theta_{\text{target}} - \theta_{\text{current}}$$

where both angles are normalised to the range [−180°, +180°] to ensure the correction always takes the shortest angular path. The heading error drives a differential motor speed command:

- If $|\epsilon| \leq \text{HEADING\_DEADBAND}$ (5°): drive straight at `DRIVE_SPEED = 150` PWM
- If $|\epsilon| > \text{COURSE\_DRIFT\_THRESH}$ (12°): re-enter `PREPARING` state to reacquire heading (prevents cumulative drift)
- Otherwise: apply proportional differential correction — the lagging side is slowed relative to the leading side by a factor proportional to $|\epsilon|$

During heading acquisition (`ACQUIRING_HEADING`), the robot rotates in place:
- $|\epsilon| > \text{ACQUIRE\_SLOW\_THRESH}$ (15°): rotate at `ROTATION_SPEED_FAST = 150` PWM
- $|\epsilon| \leq 15°$: rotate at `ROTATION_SPEED_SLOW = 100` PWM for fine alignment

All motor commands are sent to the Arduino Mega as `CMD_MANUAL_OVERRIDE` I2C messages with the computed left and right PWM values encoded as signed 8-bit integers.

### 3.8.5 Servo-Scan Obstacle Avoidance

When the HC-SR04 measures a forward distance below 30 cm, the NavController transitions to `OBSTACLE_DETECTED`. The avoidance sequence proceeds through three phases:

**Phase 1 — Immediate Stop (0.3 s):** Both motors are halted. A short pause allows the robot's momentum to dissipate before servo movement begins.

**Phase 2 — Servo Scan:** The SG90 servo rotates to three positions in sequence:
1. Centre (90°) — measure forward clearance (already known: < 30 cm, triggering obstacle)
2. Left (160°) — measure left clearance; store distance
3. Right (20°) — measure right clearance; store distance

The direction with the greatest clearance is selected for the avoidance turn.

**Phase 3 — Rotation and Recheck:** The robot rotates toward the selected direction for `OBSTACLE_TURN_TIME = 1.0 s` at `ROTATION_SPEED_FAST`. After the rotation, the ultrasonic sensor is queried again; if clearance is now > 30 cm, the state machine returns to `PREPARING` to reacquire heading toward the original target waypoint. If still blocked, the avoidance sequence repeats, alternating direction on each attempt.

The KY-032 IR sensor fires a falling-edge interrupt at any time, including within the `NAVIGATING` state, causing an immediate hardware-level motor halt irrespective of the navigation state machine — providing a "panic stop" for very fast-approaching close obstacles.

### 3.8.6 Return-to-Start

The `CMD_RETURN_TO_START` command instructs the robot to return to its starting position. This is implemented using a 100-point ring buffer that records the robot's GPS positions during outbound navigation. On receipt of the return command, the NavController reverses the buffer into a new waypoint list and executes normal GPS waypoint navigation in reverse, effectively retracing the outbound path. This approach avoids the need for a global path planner while providing a reliable return capability.

---

## 3.9 Testing Methodology

### 3.9.1 Testing Strategy

The testing strategy followed a bottom-up integration approach: individual components were tested in isolation before being integrated into subsystems, and subsystems were validated before end-to-end system testing was conducted. This approach localised faults to the smallest possible scope, reducing debugging time. Table 3.13 summarises the test categories.

**Table 3.13: Test Categories and Methods**

| Category | Method | Tools |
|---|---|---|
| Unit testing | Individual class and function testing | Python unittest, Arduino Serial Monitor |
| Sensor accuracy | Comparison against reference instruments | Digital thermometer (DHT11), calibrated gas reference |
| Gas calibration validation | PPM output vs. datasheet characteristic curves | `test_ppm_calibration.py` script |
| I2C communication | Command loopback testing | `test_i2c_mega.py` script |
| Navigation algorithm | Geodesic computation validation | `scripts/calibrate_compass.py`, GPS trace log |
| Dashboard latency | Timestamp delta from POST to browser render | Browser developer tools network timeline |
| Obstacle avoidance | Physical obstacle course testing | Measured obstacle distances, video recording |
| End-to-end | Full autonomous waypoint navigation | GPS track log, sensor data log, video |

### 3.9.2 Gas Sensor Calibration Test

A dedicated test script (`test_ppm_calibration.py`) was developed to validate the `GasCalibration` module independently of the hardware. The script exercises all five gas-type calibration curves across a range of synthetic ADC input values spanning the full 0–32767 count range and verifies that the computed PPM values are monotonically consistent with the expected direction of the sensitivity curves (higher ADC → higher voltage → lower sensor resistance → higher PPM for most MQ sensors). The script also tests edge cases including zero-value ADC (returns 0.0 PPM), negative ADC (returns 0.0 PPM), and unknown sensor type (logs error and returns 0.0 PPM).

### 3.9.3 I2C Communication Test

The `raspberry_pi/test_i2c_mega.py` script tests the I2C link to the Arduino Mega by executing the full command set in sequence and validating the response codes. Commands returning structured payloads (GPS, status) are decoded and their fields are validated for type correctness and physical plausibility. The test includes a bus recovery test that artificially holds the SCL line low (using a GPIO bit-bang) for 30 ms and then verifies that the Arduino's watchdog routine restores normal I2C operation within 1 second.

### 3.9.4 Compass and ADS1115 Tests

Dedicated test scripts (`raspberry_pi/test_ads1115.py`, `scripts/calibrate_compass.py`) verify sensor hardware independently. The ADS1115 test reads all four channels at 5-second intervals and validates that the returned values are within the expected 0–32767 range and that channels A0–A2 (connected to MQ sensors) show values consistent with sensor warm-up curves. The compass calibration script rotates the robot in a full 360° circle and records the raw X/Y magnetometer values, detecting any elliptical distortion indicative of hard-iron interference and computing the correction offset.

### 3.9.5 End-to-End Navigation Test

End-to-end validation was conducted outdoors by defining a series of GPS waypoints at measured distances from the robot's starting position. The robot's GPS track was logged to the dashboard database and post-processed to compute cross-track error (deviation from the straight-line path between consecutive waypoints) and final positioning error (distance between the logged arrival position and the intended waypoint coordinate). Obstacle avoidance was tested by placing physical obstacles (cardboard boxes) on the robot's path at known positions and recording the avoidance manoeuvre outcomes from the dashboard camera feed and event log.

---

## 3.10 Summary

This chapter presented the complete methodology for the ARM-based Environmental Monitoring and Aide Robot, covering hardware architecture, component selection and configuration, sensor integration and PPM calibration, software architecture and threading model, I2C and wireless communication protocols, navigation algorithm design, and the testing strategy. The dual-processor architecture partitioning high-level control to the Raspberry Pi and real-time motor control to the Arduino Mega provides a clean and validated engineering solution to the requirements defined in Chapter 1 and the gaps identified in the literature review of Chapter 2. Chapter 4 presents the results obtained from testing each subsystem and the integrated system as a whole.

---

*End of Chapter 3*
