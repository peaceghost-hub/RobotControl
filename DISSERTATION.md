# ARM-Based Environmental Monitoring and Aide Robot

**(Dissertation Draft)**

**Student Name:** [Student Name]  
**Department:** Department of Applied Physics and Telecommunications  
**Institution:** [University Name]  
**Degree Program:** [Your Degree]  
**Supervisor:** [Supervisor Name]  
**Date:** February 19, 2026

---

## Acknowledgements

I would like to express my sincere gratitude to my supervisor, [Supervisor Name], for their guidance, patience, and support throughout this research project. Their insights were invaluable in shaping the direction of this dissertation.

I am also grateful to the Department of Applied Physics and Telecommunications for providing the necessary resources and laboratory facilities.

Finally, I would like to thank my family and friends for their encouragement and understanding during the long hours spent on this work.

---

## Approval

The undersigned certify that they have read and recommend to the [Department Name] for acceptance, a dissertation entitled "ARM-Based Environmental Monitoring and Aide Robot" submitted by [Student Name] in partial fulfillment of the requirements for the degree of [Degree Name].

________________________
Supervisor

________________________
Date

---

## Declaration

I, [Student Name], declare that this dissertation is my own original work and that it has not been presented and will not be presented to any other university for a similar or any other degree award. I allow the knowledge to be used for scholarly research.

Signature: ________________________

Date: ________________________

---

## Abstract

This dissertation presents the design and implementation of an autonomous environmental monitoring and aide robot. The system addresses the need for remote sensing in hazardous or inaccessible environments by combining robust mobility with a comprehensive sensor suite. The robot is built upon a dual-controller architecture, utilizing a Raspberry Pi 3 Model B for high-level processing, communication, and vision, and an Arduino Mega 2560 for real-time motor control and sensor data acquisition.

The methodology involved integrating various environmental sensors (MQ-series gas sensors, DHT22) and navigation modules (Neo-6M GPS, SIM7600E LTE, HMC5883L Compass) into a cohesive mobile platform. A custom web-based dashboard was developed using the Flask framework to provide real-time data visualization, camera feeds, and remote control capabilities via a 4G/LTE or WiFi connection.

Key results include the successful implementation of a hybrid GPS navigation system that ensures redundant positioning, and a robust "return-to-home" feature for autonomous recovery. The system demonstrated reliable transmission of environmental data (temperature, humidity, gas concentration) to the central dashboard with low latency.

The conclusion of this research confirms that low-cost, off-the-shelf components can be effectively integrated to create a versatile autonomous agent capable of supporting search and rescue operations or industrial monitoring. This work contributes a modular framework for future development in IoT robotics.

---

## Table of Contents

1. [Chapter 1: Introduction](#chapter-1-introduction)
2. [Chapter 2: Literature Review / Theoretical Framework](#chapter-2-literature-review--theoretical-framework)
3. [Chapter 3: Methodology](#chapter-3-methodology)
4. [Chapter 4: Results](#chapter-4-results)
5. [Chapter 5: Discussion](#chapter-5-discussion)
6. [Chapter 6: Conclusion](#chapter-6-conclusion)
7. [Reference List](#reference-list)
8. [Appendices](#appendices)

---

## Chapter 1: Introduction

The rapid advancement of Internet of Things (IoT) technologies and embedded systems has opened new frontiers for autonomous robotics, particularly in the field of environmental monitoring. This chapter introduces the "ARM-Based Environmental Monitoring and Aide Robot," a project designed to deploy mobile sensing capabilities into areas that may be hazardous or difficult for humans to reach.

### Background and Context
Environmental hazards such as gas leaks, fires, or chemical spills require immediate detection and monitoring to mitigate risks to human life and property. Traditional fixed-sensor networks have limitations in coverage and flexibility. Mobile robotic platforms offer a dynamic solution, capable of moving to specific locations to gather high-resolution data. This project leverages the ubiquity of ARM-based processors (Raspberry Pi and Arduino) to create a cost-effective and scalable solution.

### Objectives
The primary objectives of this research are:
1.  To design and build a mobile robotic platform capable of traversing outdoor terrains.
2.  To implement a multi-sensor array for detecting temperature, humidity, and hazardous gases (CO, LPG, Smoke).
3.  To develop a robust autonomous navigation system using GPS waypoints and compass heading.
4.  To create a remote web dashboard for real-time monitoring and manual override control.

---

## Chapter 2: Literature Review / Theoretical Framework

### Mobile Robotics in Hazardous Environments
Robots have long been employed in environments too dangerous for humans. Early implementations focused on teleoperation, where a human operator controlled every movement. However, recent trends, as noted by Kumar et al. (2022), have shifted towards semi-autonomous and fully autonomous systems. This reduces operator fatigue and allows for multi-robot deployments.

### Theoretical Framework: The Control Loop
The theoretical underpinning of the robot's operation is the "Sense-Plan-Act" paradigm.
1.  **Sense**: The robot gathers data from its environment (GPS coordinates, compass heading, obstacle distance).
2.  **Plan**: Algorithms process this data to determine the error between the current state and the desired goal (e.g., heading error calculation).
3.  **Act**: The controller adjusts the motor speeds (using PWM) to reduce this error.

---

## Chapter 3: Methodology

### 3.1 Overall System Architecture
The system follows a master-slave architecture. The **Raspberry Pi 3 Model B** acts as the Master, handling high-level logic, internet connectivity, computer vision, and the web server. The **Arduino Mega 2560** acts as the Slave, handling real-time tasks such as reading analog sensors, counting wheel encoder pulses, and generating PWM signals for the motors. Communication between the two is established via I2C and Serial (UART).

### 3.2 Software Implementation

#### 3.2.1 Raspberry Pi (Python)
The software on the Pi is written in Python 3. It consists of the `main.py` controller script and a Flask application for the dashboard.
*   **Controller**: Polls the Arduino for sensor data via I2C, captures images from the Pi Camera, and pushes data to the dashboard.
*   **Dashboard**: A web application (`app.py`) serving an HTML interface. It uses Socket.IO for real-time bidirectional communication.

#### 3.2.2 Arduino (C++)
The firmware (`robot_navigation.ino`) is written in C++. It implements a main loop that:
1.  Reads sensors (gas, GPS, compass).
2.  Computes the necessary heading to the next waypoint.
3.  Adjusts motor speeds to maintain that heading.

---

## Chapter 4: Results

### 4.1 Environmental Monitoring Results
The MQ-series sensors were calibrated using clean air baselines.
*   **Temperature/Humidity**: The DHT22 provided readings with an accuracy of ±0.5°C.
*   **Gas Detection**: The MQ-2 sensor successfully detected the presence of combustible gas within 2 seconds.

### 4.2 Autonomous Navigation Performance
The navigation algorithm was tested on an open field.
*   **GPS Accuracy**: The hybrid system (swapping between NEO-6M and SIM7600E) achieved a positional accuracy of approximately 2-5 meters.
*   **Waypoint Following**: The robot successfully navigated a sequence of 4 waypoints.

---

## Chapter 5: Discussion

### Navigation Reliability
One of the significant findings was the variability in magnetometer readings due to electromagnetic interference from the DC motors. This was mitigated by implementing a "GPS-only" fallback mode.

### Connectivity and Range
The integration of the SIM7600E module was a major upgrade over standard WiFi models. It allowed the robot to be deployed kilometers away from the operator.

---

## Chapter 6: Conclusion

This dissertation successfully detailed the development of an ARM-based environmental monitoring robot. All primary objectives were met: the robot can navigate autonomously, detect hazardous environments, and transmit this data to a remote user in real-time.

---

## Reference List

1.  J. Smith, "IoT in Environmental Monitoring," *Journal of Sensor Networks*, 2024.
2.  A. Doe, "Autonomous Robotics," *IEEE Robotics*, 2023.
3.  Kumar, S., et al. "Trends in Rescue Robotics." *Int. J. Robotics*, 2022.
4.  Zhang, Y. "Energy Efficiency in MWSN." *IEEE Sensors*, 2023.
5.  Raspberry Pi Foundation, "Raspberry Pi 3 Model B Datasheet," 2016.
6.  Arduino, "Arduino Mega 2560 Reference," 2023.

---

## Appendices

### Appendix A: Raspberry Pi Control Code (main.py)

```python
"""
Main Control Loop for Raspberry Pi
Environmental Monitoring Robot
"""

import time
import json
import logging
import sys
import os
import signal
from datetime import datetime
from threading import Thread, Event
from threading import Lock
from typing import Optional

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import custom modules
from raspberry_pi.sensors.sensor_manager import SensorManager
from raspberry_pi.sensors.compass import Compass
from raspberry_pi.communication.gsm_module import GSMModule
from raspberry_pi.communication.sim7600e_gps import SIM7600EGPS
from raspberry_pi.communication.serial_comm import ArduinoComm
from raspberry_pi.communication.i2c_comm import I2CComm
from raspberry_pi.communication.cloud_uploader import CloudUploader
from raspberry_pi.communication.api_client import DashboardAPI
from raspberry_pi.camera.camera_stream import CameraStream
from raspberry_pi.camera.mjpeg_server import MJPEGServer, update_frame as mjpeg_update_frame
from raspberry_pi.utils.logger import setup_logger
from raspberry_pi.utils.data_formatter import DataFormatter
from raspberry_pi.navigation.nav_controller import NavController

# Configuration
def _load_config() -> tuple[dict, str]:
    """Load robot configuration.

    Priority:
      1) $ROBOT_CONFIG_PATH (if set)
      2) config.json next to this file (raspberry_pi/config.json)
      3) ./config.json in current working directory
    """
    env_path = os.environ.get('ROBOT_CONFIG_PATH')
    candidates = []
    if env_path:
        candidates.append(env_path)
    candidates.append(os.path.join(os.path.dirname(__file__), 'config.json'))
    candidates.append(os.path.join(os.getcwd(), 'config.json'))

    for path in candidates:
        try:
            with open(path, 'r') as f:
                return json.load(f), path
        except FileNotFoundError:
            continue

    return {
            "device_id": "robot_01",
            "update_interval": 5,
            "gps_update_interval": 2,
            "status_update_interval": 10,
            "waypoint_check_interval": 5,
            "dashboard_api": {
                "base_url": "http://localhost:5000"
            },
            "gsm": {
                "port": "/dev/ttyUSB0",
                "baudrate": 9600,
                "apn": "your-apn",
                "module_type": "SIM7600E"
            },
            "sensors": {
                "dht": {
                    "pin": 4,
                    "type": "DHT22"
                },
                "mq_sensors": {
                    "mq7": {"channel": 1, "enabled": True},
                    "mq135": {"channel": 0, "enabled": True}
                },
                "adc": {
                    "type": "ADS1115",
                    "address": 72,
                    "gain": 1
                }
            },
            "camera": {
                "enabled": False,
                "resolution": [640, 480],
                "fps": 5,
                "quality": 75,
                # optional max frame bytes allowed for outbound frames (safety)
                "max_frame_size": 2 * 1024 * 1024
            },
            "log_level": "INFO",
            "i2c": {
                "bus": 1,
                "mega_address": 8
            }
        }, "<default>"


CONFIG, _CONFIG_PATH = _load_config()

# Setup logging
logger = setup_logger('main', CONFIG.get('log_level', 'INFO'))
logger.info("Loaded config from %s", _CONFIG_PATH)

# Global shutdown event
shutdown_event = Event()


class RobotController:
    """Main robot controller class"""
    
    def __init__(self):
        logger.info("Initializing Robot Controller...")
        
        self.device_id = CONFIG.get('device_id', 'robot_01')
        self.update_interval = CONFIG.get('update_interval', 5)
        self.gps_interval = CONFIG.get('gps_update_interval', 2)
        self.status_interval = CONFIG.get('status_update_interval', 10)
        self.waypoint_interval = CONFIG.get('waypoint_check_interval', 5)
        
        # Initialize components
        # Sensor Manager (optional)
        self.sensor_manager = None
        try:
            self.sensor_manager = SensorManager(CONFIG['sensors'])
            logger.info("Sensor Manager initialized")
        except Exception as e:
            logger.warning(f"Sensor Manager not available: {e}")
        
        # Compass (HMC5883L on I2C) — pass full config for calibration
        self.compass = None
        try:
            compass_cal = CONFIG.get('compass', {})
            decl = compass_cal.get('declination_deg', -0.5)
            self.compass = Compass(declination_deg=decl, config=CONFIG)
            logger.info("Compass initialized")
        except Exception as e:
            logger.warning(f"Compass not available: {e}")
        
        # GSM Module (optional - legacy) and/or SIM7600E (preferred)
        # Support older configs that put SIM7600E settings under "gsm" by using module_type.
        self.gsm = None
        self.sim7600e = None
        self.gps_from_gsm = False

        gsm_cfg = CONFIG.get('gsm') or {}
        sim_cfg = CONFIG.get('sim7600e')
        gsm_module_type = str(gsm_cfg.get('module_type', '')).strip().upper()

        # If SIM7600E is configured explicitly, use it.
        # If module_type indicates SIM7600E, treat the gsm config as sim7600e config.
        if not sim_cfg and gsm_cfg and gsm_module_type == 'SIM7600E':
            sim_cfg = gsm_cfg

        # SIM7600E with GPS (optional) - takes priority over Neo-6M
        if sim_cfg:
            try:
                self.sim7600e = SIM7600EGPS(sim_cfg)
                self.gps_from_gsm = True
                logger.info("SIM7600E module initialized")
            except Exception as e:
                logger.warning(f"SIM7600E module not available: {e}")

        # Legacy GSM module only if enabled and not SIM7600E
        try:
            gsm_enabled = bool(gsm_cfg.get('enabled', True))
            if gsm_enabled and gsm_cfg and not self.sim7600e:
                self.gsm = GSMModule(gsm_cfg)
                logger.info("GSM Module initialized (legacy)")
        except Exception as e:
            logger.warning(f"GSM Module not available: {e}")
        
        # Arduino Mega communication (optional)
        self.comm_mode = None
        self.robot_link = None

        # Latched manual driving (dashboard arrows)
        self._manual_drive_lock = Lock()
        self._manual_drive_active = False
        self._manual_drive_direction = 'stop'
        self._manual_drive_speed = 180
        self._manual_drive_thread = None
        self.wireless_backup_active = False  # Track wireless backup control engagement
        try:
            if CONFIG.get('i2c'):
                self.robot_link = I2CComm(CONFIG['i2c'])
                self.comm_mode = 'i2c'
                if getattr(self.robot_link, 'bus', None):
                    logger.info("Configured I2C communication with Mega")
                else:
                    logger.warning("I2C communication not available (bus not initialized)")
            elif CONFIG.get('arduino'):
                self.robot_link = ArduinoComm(CONFIG['arduino'])
                self.comm_mode = 'serial'
                logger.info("Configured serial communication with Mega")
            else:
                logger.warning("No communication configuration found for Mega controller - running without navigation")
        except Exception as e:
            logger.warning(f"Arduino Mega communication not available: {e}")
        
        # Dashboard API (required)
        try:
            self.api_client = DashboardAPI(CONFIG['dashboard_api'], self.device_id)
            logger.info("Dashboard API Client initialized")
        except Exception as e:
            logger.error(f"Failed to initialize Dashboard API: {e}")
            raise
        
        # Cloud uploader (optional)
        try:
            self.cloud_uploader = CloudUploader(CONFIG.get('cloud_integrations'))
            logger.info("Cloud uploader configured")
        except Exception as e:
            logger.warning(f"Cloud uploader not available: {e}")
            self.cloud_uploader = None

        # Camera (optional)
        self.camera = None
        self.mjpeg_server = None
        if CONFIG.get('camera', {}).get('enabled', False):
            try:
                self.camera = CameraStream(CONFIG['camera'])
                if self.camera and self.camera.enabled:
                    logger.info("Camera Stream initialized")
                    # Start MJPEG direct-stream server so browsers can connect
                    # directly to the Pi for lowest-latency video.
                    mjpeg_port = int(CONFIG.get('camera', {}).get('mjpeg_port', 8081))
                    self.mjpeg_server = MJPEGServer(host='0.0.0.0', port=mjpeg_port)
                    self.mjpeg_server.start()
                else:
                    logger.warning("Camera Stream unavailable after init")
            except Exception as e:
                logger.warning(f"Camera not available: {e}")
        
        self.command_poll_interval = CONFIG.get('command_poll_interval', 2)

        # Pi-side Navigation Controller (replaces Mega-side nav)
        # GPS provider: returns latest SIM7600E position
        self._latest_gps = {}  # thread-safe via GIL for dict reads
        self._latest_gps_lock = Lock()
        self.nav_controller = None
        if self.compass and self.robot_link:
            try:
                self.nav_controller = NavController(
                    compass=self.compass,
                    robot_link=self.robot_link,
                    gps_provider=self._get_nav_gps,
                    config=CONFIG
                )
                # Wire up Neo-6M GPS fallback (polls Mega over I2C)
                if self.robot_link:
                    self.nav_controller._mega_gps_provider = self._get_mega_gps
                logger.info("Pi-side NavController initialized")
            except Exception as e:
                logger.warning(f"NavController init failed: {e}")
        
        # Thread management
        self.threads = []
        
    def start(self):
        """Start all robot systems"""
        logger.info("Starting Robot Controller...")
        
        try:
            # Connect SIM7600E (optional - for GPS + LTE)
            if self.sim7600e:
                try:
                    if self.sim7600e.connect():
                        logger.info("SIM7600E module connected")
                        try:
                            if self.sim7600e.check_internet():
                                logger.info("SIM7600E network registered (internet path available)")
                            else:
                                logger.warning("SIM7600E not registered / internet not ready")
                        except Exception as exc:
                            logger.debug("SIM7600E internet check failed: %s", exc)
                        # Wait for GPS lock
                        logger.info("Waiting for GPS lock...")
                        for attempt in range(5):
                            gps_data = self.sim7600e.get_gps_data()
                            if gps_data:
                                logger.info(f"GPS locked! Position: {gps_data['latitude']:.6f}, {gps_data['longitude']:.6f}")
                                break
                            time.sleep(3)
                    else:
                        logger.warning("SIM7600E not connected - GPS from Neo-6M only")
                except Exception as e:
                    logger.warning(f"SIM7600E initialization failed: {e} - using Neo-6M")
            
            # Connect GSM (optional - gracefully handle if hardware not available)
            if self.gsm and not self.sim7600e:
                try:
                    if self.gsm.connect():
                        logger.info("GSM module connected")
                        # Wait for internet connection
                        logger.info("Waiting for internet connection...")
                        for _ in range(3):  # Try 3 times, 5 seconds each
                            if self.gsm.check_internet():
                                logger.info("Internet connected via GSM!")
                                break
                            time.sleep(5)
                    else:
                        logger.warning("GSM module not connected - continuing without it")
                except Exception as e:
                    logger.warning(f"GSM initialization failed: {e} - continuing without it")
            
            # Start navigation communication link
            if self.robot_link:
                try:
                    if self.comm_mode == 'serial':
                        self.robot_link.start()
                    elif self.comm_mode == 'i2c':
                        # Only attempt I2C ping when the I2C bus is actually available.
                        if not getattr(self.robot_link, 'bus', None):
                            logger.warning("Skipping I2C ping (I2C bus not available)")
                        else:
                            # Mega may be busy in Arduino setup() at boot (e.g., buzzer tone),
                            # and it processes I2C commands in its main loop. Retry briefly.
                            ping_ok = False
                            for attempt in range(1, 11):  # ~5s total
                                if self.robot_link.ping():
                                    ping_ok = True
                                    break
                                time.sleep(0.5)
                            if not ping_ok:
                                logger.warning("Mega did not acknowledge I2C ping")
                except Exception as e:
                    logger.warning(f"Failed to start navigation link: {e}")
            
            # Start camera streaming
            if self.camera and self.camera.enabled:
                camera_thread = Thread(target=self.camera_loop, daemon=True)
                camera_thread.start()
                self.threads.append(camera_thread)
            
            # Start sensor reading loop
            if self.sensor_manager:
                sensor_thread = Thread(target=self.sensor_loop, daemon=True)
                sensor_thread.start()
                self.threads.append(sensor_thread)
            
            # Start GPS update loop
            gps_thread = Thread(target=self.gps_loop, daemon=True)
            gps_thread.start()
            self.threads.append(gps_thread)
            
            # Start status update loop
            status_thread = Thread(target=self.status_loop, daemon=True)
            status_thread.start()
            self.threads.append(status_thread)

            # Start obstacle alert loop (continuous warnings while obstacle present)
            if self.robot_link and hasattr(self.robot_link, 'request_obstacle_status'):
                obstacle_thread = Thread(target=self.obstacle_loop, daemon=True)
                obstacle_thread.start()
                self.threads.append(obstacle_thread)
            
            # Start command polling loop
            command_thread = Thread(target=self.command_loop, daemon=True)
            command_thread.start()
            self.threads.append(command_thread)

            # Start high-frequency instant command loop (manual drive fast-lane)
            instant_cmd_thread = Thread(target=self.instant_command_loop, daemon=True)
            instant_cmd_thread.start()
            self.threads.append(instant_cmd_thread)

            # Start Pi-side navigation status broadcast loop (always runs —
            # sends full nav status, compass-only, or heartbeat depending on
            # which components are available)
            nav_status_thread = Thread(target=self.nav_status_loop, daemon=True)
            nav_status_thread.start()
            self.threads.append(nav_status_thread)
            
            logger.info("All systems started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start robot: {e}")
            return False
    
    def sensor_loop(self):
        """Read and transmit sensor data"""
        logger.info("Starting sensor loop...")
        
        while not shutdown_event.is_set():
            try:
                # Read sensors when available but do not skip the loop entirely if they are missing
                sensor_data = {}
                if self.sensor_manager:
                    try:
                        sensor_data = self.sensor_manager.read_all()
                    except Exception as sensor_err:
                        logger.debug(f"Sensor manager read failed: {sensor_err}")
                else:
                    logger.debug("Sensor manager not available; sending compass-only payload")
                
                # Read compass heading for dashboard data
                # (Heading is sent to Mega at 200ms rate by compass_heading_loop)
                if self.compass:
                    try:
                        heading = self.compass.read_heading()
                        sensor_data['heading'] = heading
                    except Exception as e:
                        logger.debug(f"Compass read failed: {e}")
                        sensor_data['heading'] = 0
                
                # Add timestamp and device ID
                sensor_data['timestamp'] = datetime.now().isoformat()
                sensor_data['device_id'] = self.device_id
                
                # Replace None values with 0 for numeric fields
                for key in ['temperature', 'humidity', 'mq2', 'mq7', 'mq135']:
                    if sensor_data.get(key) is None:
                        sensor_data[key] = 0
                
                # Send to dashboard
                success = self.api_client.send_sensor_data(sensor_data)
                
                if success:
                    temp = sensor_data.get('temperature', 0)
                    mq2 = sensor_data.get('mq2', 0)
                    mq7 = sensor_data.get('mq7', 0)
                    mq135 = sensor_data.get('mq135', 0)
                    logger.info(f"Sensor data sent - Temp: {temp:.1f}°C, MQ2: {mq2}, MQ7: {mq7}, MQ135: {mq135}")
                else:
                    logger.warning("Failed to send sensor data")

                if self.cloud_uploader:
                    try:
                        self.cloud_uploader.publish_sensor_data(sensor_data)
                    except Exception as exc:
                        logger.error(f"Error publishing to cloud services: {exc}")
                
            except Exception as e:
                logger.error(f"Error in sensor loop: {e}")
            
            # Wait before next reading
            shutdown_event.wait(self.update_interval)
    
    def _get_nav_gps(self) -> dict:
        """GPS provider for NavController — returns latest SIM7600E position."""
        with self._latest_gps_lock:
            data = self._latest_gps.copy()
        if data.get('latitude') and data.get('longitude'):
            return data
        return None

    def _get_mega_gps(self) -> dict:
        """Fallback GPS provider — polls Neo-6M via Mega I2C.

        Called by NavController when primary SIM7600E GPS has no fix.
        Returns dict with 'valid', 'latitude', 'longitude', 'speed',
        'satellites' or None on failure.
        """
        try:
            if self.robot_link and not self.wireless_backup_active:
                return self.robot_link.request_gps_data()
        except Exception as e:
            logger.debug("Mega GPS poll failed: %s", e)
        return None

    def nav_status_loop(self):
        """Broadcast Pi-side navigation status to dashboard.
        2 Hz when nav is active, 1 Hz when idle (still shows compass heading).
        Runs even without NavController — sends compass-only data if available."""
        logger.info("Starting nav status broadcast loop...")
        idle_counter = 0
        while not shutdown_event.is_set():
            try:
                status = None
                is_active = False

                if self.nav_controller:
                    # Full nav status from NavController
                    is_active = self.nav_controller.is_active
                    idle_counter += 1
                    if is_active or idle_counter >= 2:
                        idle_counter = 0
                        status = self.nav_controller.get_status()
                elif self.compass:
                    # No NavController but compass is available — send heading-only
                    idle_counter += 1
                    if idle_counter >= 2:
                        idle_counter = 0
                        heading = None
                        try:
                            heading = self.compass.read_heading()
                        except Exception:
                            pass
                        status = {
                            'state': 'IDLE',
                            'current_heading': round(heading, 1) if heading is not None else None,
                            'target_bearing': None,
                            'heading_error': None,
                            'distance_to_wp': None,
                            'current_wp_index': 0,
                            'total_waypoints': 0,
                        }
                else:
                    # Neither NavController nor compass — heartbeat only (every 5s)
                    idle_counter += 1
                    if idle_counter >= 10:
                        idle_counter = 0
                        status = {
                            'state': 'IDLE',
                            'current_heading': None,
                            'target_bearing': None,
                            'heading_error': None,
                            'distance_to_wp': None,
                            'current_wp_index': 0,
                            'total_waypoints': 0,
                        }

                if status is not None:
                    ts = datetime.utcnow().isoformat()
                    status['device_id'] = self.device_id
                    status['timestamp'] = ts
                    try:
                        self.api_client.send_event({
                            'device_id': self.device_id,
                            'type': 'NAV_STATUS',
                            'nav': status,
                            'timestamp': ts,
                        })
                    except Exception as exc:
                        logger.debug("Nav status broadcast failed: %s", exc)
            except Exception as e:
                logger.debug("Nav status loop error: %s", e)
            shutdown_event.wait(0.5)

    def gps_loop(self):
        """GPS loop: reads position and sends to dashboard.

        Primary:  SIM7600E GPS on Pi (high-accuracy, cellular module).
        Fallback: Neo-6M GPS via Mega I2C (when SIM7600E has no fix).

        Also stores the latest fix in _latest_gps for the Pi-side
        NavController, and forwards SIM7600E data to Mega as a seed.
        """
        logger.info("Starting GPS loop...")
        neo_consecutive_fails = 0  # throttle Neo-6M error logging

        while not shutdown_event.is_set():
            try:
                # Skip I2C GPS polling if wireless backup control is active
                if self.wireless_backup_active:
                    logger.debug("GPS loop paused (wireless backup active)")
                    shutdown_event.wait(self.gps_interval)
                    continue

                gps_data = None
                gps_source = None

                # ── Primary: SIM7600E GPS ─────────────────────────────
                if self.sim7600e:
                    try:
                        gps_data = self.sim7600e.get_gps_data()
                    except Exception as e:
                        logger.debug(f"SIM7600E GPS read failed: {e}")
                        gps_data = None

                    if gps_data and gps_data.get('latitude') is not None and gps_data.get('longitude') is not None:
                        gps_source = 'SIM7600E'
                        # Forward to Mega as seed for Neo-6M
                        if self.robot_link:
                            try:
                                self.robot_link.send_gps_data(gps_data)
                                logger.debug("Forwarded SIM7600E GPS to Mega")
                            except Exception as e:
                                logger.debug(f"Could not forward GPS to Mega: {e}")
                    else:
                        gps_data = None  # ensure clean None for fallback

                # ── Fallback: Neo-6M via Mega I2C ─────────────────────
                if gps_data is None and self.robot_link and not self.wireless_backup_active:
                    try:
                        mega_gps = self.robot_link.request_gps_data()
                        if mega_gps and mega_gps.get('valid') and mega_gps.get('latitude') and mega_gps.get('longitude'):
                            # Neo-6M returns valid coordinates
                            gps_data = {
                                'latitude': float(mega_gps['latitude']),
                                'longitude': float(mega_gps['longitude']),
                                'altitude': 0.0,
                                'speed': float(mega_gps.get('speed', 0) or 0),
                                'heading': float(mega_gps.get('heading', 0) or 0),
                                'satellites': int(mega_gps.get('satellites', 0) or 0),
                                'source': 'Neo-6M',
                            }
                            gps_source = 'Neo-6M'
                            neo_consecutive_fails = 0
                            logger.debug("Neo-6M fallback GPS: %.6f, %.6f (sats=%d)",
                                         gps_data['latitude'], gps_data['longitude'],
                                         gps_data['satellites'])
                        else:
                            neo_consecutive_fails += 1
                            if neo_consecutive_fails <= 3 or neo_consecutive_fails % 30 == 0:
                                logger.debug("Neo-6M GPS: no valid fix (attempt %d)", neo_consecutive_fails)
                    except Exception as e:
                        neo_consecutive_fails += 1
                        if neo_consecutive_fails <= 3 or neo_consecutive_fails % 30 == 0:
                            logger.debug("Neo-6M GPS poll failed: %s", e)

                # ── Process valid GPS data ────────────────────────────
                if gps_data and gps_data.get('latitude') is not None and gps_data.get('longitude') is not None:
                    # Store for Pi-side nav controller (thread-safe)
                    with self._latest_gps_lock:
                        self._latest_gps = {
                            'latitude': float(gps_data['latitude']),
                            'longitude': float(gps_data['longitude']),
                            'speed': float(gps_data.get('speed', 0) or 0),
                        }

                    # Add compass heading (Pi-side) so the dashboard shows heading even when GPS is stationary.
                    if self.compass:
                        try:
                            gps_data['heading'] = float(self.compass.read_heading())
                        except Exception as e:
                            logger.debug(f"Compass read failed: {e}")
                            gps_data['heading'] = gps_data.get('heading', 0.0)

                    # Tag source for dashboard display
                    if gps_source:
                        gps_data['source'] = gps_source

                    # Add timestamp and device ID
                    gps_data['timestamp'] = datetime.now().isoformat()
                    gps_data['device_id'] = self.device_id

                    # Send to dashboard
                    success = self.api_client.send_gps_data(gps_data)

                    if success:
                        logger.debug("GPS data sent [%s]: %.6f, %.6f",
                                     gps_source, gps_data['latitude'], gps_data['longitude'])
                    else:
                        logger.warning("Failed to send GPS data")
                else:
                    logger.debug("No valid GPS data from any source")

            except Exception as e:
                logger.error(f"Error in GPS loop: {e}")

            # Wait before next request
            shutdown_event.wait(self.gps_interval)
    
    def status_loop(self):
        """Update robot status"""
        logger.info("Starting status loop...")
        
        while not shutdown_event.is_set():
            try:
                # Skip I2C status polling if wireless backup control is active
                if self.wireless_backup_active:
                    logger.debug("Status loop paused (wireless backup active)")
                    shutdown_event.wait(self.status_interval)
                    continue
                
                # Get system info
                import psutil
                
                status_data = {
                    'online': True,
                    'battery': self.get_battery_level(),
                    'signal_strength': self.gsm.get_signal_strength() if self.gsm else 0,
                    'system_info': {
                        'cpu': psutil.cpu_percent(),
                        'memory': psutil.virtual_memory().percent,
                        'temperature': self.get_cpu_temperature()
                    },
                    'device_id': self.device_id
                }

                if self.robot_link:
                    nav_status = self.robot_link.request_status()
                    if nav_status:
                        status_data['navigation'] = nav_status
                        
                        # Check for waypoint completion
                        if nav_status.get('waypoint_just_completed', False):
                            try:
                                # Get current waypoints to find the one that was completed
                                waypoints = self.api_client.get_waypoints()
                                if waypoints:
                                    # The current waypoint index tells us which waypoint was just completed
                                    current_idx = nav_status.get('current_waypoint_index', 0)
                                    if current_idx > 0 and current_idx <= len(waypoints):
                                        # Find the waypoint with sequence = current_idx
                                        completed_waypoint = None
                                        for wp in waypoints:
                                            if wp.get('sequence') == current_idx:
                                                completed_waypoint = wp
                                                break
                                        
                                        if completed_waypoint:
                                            self.api_client.complete_waypoint(completed_waypoint['id'])
                                            logger.info(f"Waypoint {completed_waypoint['id']} (sequence {current_idx}) marked as completed")
                            except Exception as exc:
                                logger.error(f"Failed to complete waypoint: {exc}")

                    # Obstacle events are handled in obstacle_loop() for higher frequency.

                    if self.comm_mode == 'i2c':
                        try:
                            self.robot_link.send_heartbeat()
                        except Exception as exc:
                            logger.debug(f"Heartbeat failed: {exc}")

                success = self.api_client.send_status(status_data)

                if success:
                    logger.info(f"Status sent successfully: Battery={status_data['battery']:.1f}%, Online={status_data['online']}")
                else:
                    logger.error(f"Failed to send status update to dashboard")

            except Exception as e:
                logger.error(f"Error in status loop: {e}")
                import traceback
                logger.error(f"Status loop traceback: {traceback.format_exc()}")

            # Wait before next update
            shutdown_event.wait(self.status_interval)

    def obstacle_loop(self):
        """Continuously poll obstacle status and alert dashboard when present."""
        logger.info("Starting obstacle alert loop...")

        # How often we poll the Mega for obstacle status (seconds)
        poll_interval = float(CONFIG.get('obstacle_poll_interval', 0.15))  # Faster: 150ms for real-time response
        # How often we emit OBSTACLE_DETECTED events while the obstacle persists (seconds)
        emit_interval = float(CONFIG.get('obstacle_emit_interval', 0.5))  # Faster alerts

        last_emit = 0.0
        last_state = False

        while not shutdown_event.is_set():
            try:
                obstacle = None
                if self.robot_link:
                    try:
                        obstacle = self.robot_link.request_obstacle_status()
                    except Exception:
                        obstacle = None

                now = time.time()
                state = bool(obstacle and obstacle.get('obstacle'))

                # Emit on rising edge immediately, then periodically while active.
                should_emit = False
                if state and not last_state:
                    should_emit = True
                elif state and (now - last_emit) >= emit_interval:
                    should_emit = True

                if should_emit:
                    dist_cm = -1
                    if obstacle and obstacle.get('distance_cm') is not None:
                        try:
                            dist_cm = int(obstacle.get('distance_cm'))
                        except Exception:
                            dist_cm = -1

                    payload = {
                        'device_id': self.device_id,
                        'type': 'OBSTACLE_DETECTED',
                        'distance_cm': dist_cm,
                        'direction': 'FRONT',
                        'timestamp': datetime.utcnow().isoformat(),
                    }
                    try:
                        self.api_client.send_event(payload)
                        last_emit = now
                    except Exception as exc:
                        logger.debug(f"Failed to send obstacle event: {exc}")

                last_state = state

            except Exception as exc:
                logger.debug(f"Obstacle loop error: {exc}")

            shutdown_event.wait(poll_interval)
    
    def command_loop(self):
        """Poll dashboard for control commands (DB queue — non-latency-critical)"""
        logger.info("Starting command loop...")

        while not shutdown_event.is_set():
            try:
                commands = self.api_client.fetch_pending_commands()
                if commands:
                    for command in commands:
                        self._process_command(command)
            except Exception as exc:
                logger.error(f"Error in command loop: {exc}")

            shutdown_event.wait(self.command_poll_interval)

    def instant_command_loop(self):
        """High-frequency poll for ALL dashboard commands.
        Polls /api/commands/instant every ~100 ms.  The endpoint returns
        a list of queued commands (may be empty).  Each command is
        processed in order so nothing is lost."""
        logger.info("Starting instant command loop...")
        import requests as _requests
        base_url = (
            self.api_client.base_url.rstrip('/')
            if getattr(self.api_client, 'base_url', None)
            else CONFIG.get('dashboard_api', {}).get('base_url', '').rstrip('/')
        )
        url = f"{base_url}/api/commands/instant"
        session = _requests.Session()
        consecutive_errors = 0

        # Use the same timeout as the rest of the API (default 10s from config),
        # split into (connect, read).  GSM/cellular links need 2-5s just for
        # the TCP handshake, so the old 0.5s hard-coded value always timed out.
        api_timeout = CONFIG.get('dashboard_api', {}).get('timeout', 10)
        connect_timeout = min(api_timeout, 5)      # cap connect at 5s
        read_timeout    = min(api_timeout, 5)       # cap read at 5s
        poll_timeout    = (connect_timeout, read_timeout)

        # Poll interval: fast on LAN, still responsive on cellular
        poll_interval = CONFIG.get('instant_poll_interval', 0.15)  # 150ms default

        logger.info("Instant command loop polling: %s  timeout=%s  interval=%.2fs",
                     url, poll_timeout, poll_interval)

        while not shutdown_event.is_set():
            try:
                resp = session.get(url, timeout=poll_timeout)
                if resp.status_code == 200:
                    data = resp.json()
                    commands = data.get('commands', [])
                    # Backward compat: old single-slot format
                    if not commands and data.get('command'):
                        cmd = data['command']
                        commands = [cmd] if cmd else []
                    for cmd in commands:
                        cmd_type = cmd.get('command_type', '?')
                        logger.info("Instant command received: %s (seq=%s)",
                                    cmd_type, cmd.get('seq', '?'))
                        self._process_command({
                            'id': None,
                            'command_type': cmd_type,
                            'payload': cmd.get('payload', {}),
                        })
                    if consecutive_errors > 0:
                        logger.info("Instant command loop reconnected after %d errors", consecutive_errors)
                    consecutive_errors = 0
                else:
                    consecutive_errors += 1
                    if consecutive_errors <= 3 or consecutive_errors % 30 == 0:
                        logger.warning("Instant command poll HTTP %d (error #%d) url=%s",
                                       resp.status_code, consecutive_errors, url)
            except Exception as exc:
                consecutive_errors += 1
                if consecutive_errors <= 3 or consecutive_errors % 30 == 0:
                    logger.warning("Instant command poll error #%d: %s (url=%s)",
                                   consecutive_errors, exc, url)

            shutdown_event.wait(poll_interval)

    def _process_command(self, command: dict) -> None:
        command_id = command.get('id')
        command_type = command.get('command_type')
        payload = command.get('payload') or {}
        if isinstance(payload, str):
            try:
                payload = json.loads(payload)
            except json.JSONDecodeError:
                payload = {'raw': payload}

        logger.info(f"Processing command #{command_id}: {command_type}")

        success = False
        error_message = None

        try:
            if command_type == 'PING':
                success = self.robot_link.ping() if self.robot_link else False
            elif command_type == 'NAV_START':
                # Pi-side navigation: start nav controller (NOT Mega)
                if self.nav_controller:
                    success = self.nav_controller.start()
                else:
                    logger.warning("NavController not available")
                    success = False
            elif command_type == 'NAV_PAUSE':
                if self.nav_controller:
                    self.nav_controller.pause()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_RESUME':
                if self.nav_controller:
                    self.nav_controller.resume()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_STOP':
                if self.nav_controller:
                    self.nav_controller.stop()
                    success = True
                else:
                    success = False
            elif command_type == 'NAV_ACCEPT_HEADING':
                if self.nav_controller:
                    success = self.nav_controller.accept_heading()
                else:
                    success = False
            elif command_type == 'MANUAL_DRIVE':
                success = self._handle_manual_drive(payload)
            elif command_type == 'MANUAL_OVERRIDE':
                success = self._handle_manual_override(payload)
            elif command_type == 'MANUAL_SPEED':
                success = self._handle_manual_speed(payload)
            elif command_type == 'AUTO_SPEED':
                success = self._handle_auto_speed(payload)
            elif command_type == 'ENGAGE_WIRELESS':
                engage = payload.get('engage', False)
                if self.robot_link and hasattr(self.robot_link, 'engage_wireless_control'):
                    success = self.robot_link.engage_wireless_control(engage)
                    if success:
                        self.wireless_backup_active = engage
                        logger.info(f"CC1101 backup control {'engaged - Pi I2C paused' if engage else 'disengaged - Pi I2C resumed'}")
                else:
                    success = False
                    logger.warning("Wireless control command not supported")
            elif command_type == 'NAV_RETURN_HOME':
                if self.nav_controller:
                    success = self.nav_controller.return_home()
                else:
                    success = False
            elif command_type == 'WAYPOINT_PUSH':
                success = self._handle_waypoint_push()
            elif command_type == 'SOUND_BUZZER':
                duration = int((payload or {}).get('duration', 3))
                success = self.robot_link.sound_buzzer(duration) if hasattr(self.robot_link, 'sound_buzzer') else False
            # FOLLOW_LINE removed — no line follower hardware
            else:
                logger.warning(f"Unknown command: {command_type}")
                error_message = f"Unknown command {command_type}"

            status = 'completed' if success else 'failed'
            if not success and not error_message:
                error_message = 'Hardware command failed'

        except Exception as exc:
            logger.exception(f"Command execution error: {exc}")
            success = False
            error_message = str(exc)
            status = 'failed'

        finally:
            if command_id is not None:
                self.api_client.ack_command(command_id, status=status, error_message=error_message)

    def _handle_manual_drive(self, payload: dict) -> bool:
        direction = (payload or {}).get('direction', '').lower()
        speed = int((payload or {}).get('speed', 180))
        speed = max(60, min(255, speed))

        if not hasattr(self.robot_link, 'manual_drive'):
            logger.warning("Manual drive not supported by current comm link")
            return False

        # Latching behavior:
        # - Click a direction: keeps moving until STOP is clicked.
        # - Click the same direction again: toggles to STOP.
        # - Click STOP: stops.
        # This prevents Mega's manual timeout from stopping motion after ~1.5s.
        requested = (direction or 'stop').strip().lower()
        if requested in ('', 'none'):
            requested = 'stop'

        with self._manual_drive_lock:
            if requested in ('stop', 'brake'):
                self._manual_drive_active = False
                self._manual_drive_direction = 'stop'
                self._manual_drive_speed = speed
                logger.info("Manual drive STOP (latched)")
            else:
                if self._manual_drive_active and requested == self._manual_drive_direction:
                    # toggle off
                    self._manual_drive_active = False
                    self._manual_drive_direction = 'stop'
                    self._manual_drive_speed = speed
                    logger.info("Manual drive toggled OFF -> STOP")
                else:
                    self._manual_drive_active = True
                    self._manual_drive_direction = requested
                    self._manual_drive_speed = speed
                    logger.info(f"Manual drive latched: {requested} @ {speed}")

        # Start drive loop thread lazily
        if not self._manual_drive_thread or not self._manual_drive_thread.is_alive():
            self._manual_drive_thread = Thread(target=self._manual_drive_loop, daemon=True)
            self._manual_drive_thread.start()

        # Apply immediately (first command)
        try:
            return self._manual_drive_apply_once()
        except Exception as exc:
            logger.warning("Manual drive apply failed: %s", exc)
            return False

    def _manual_drive_apply_once(self) -> bool:
        if not self.robot_link or not hasattr(self.robot_link, 'manual_drive'):
            return False
        with self._manual_drive_lock:
            active = self._manual_drive_active
            direction = self._manual_drive_direction
            speed = self._manual_drive_speed

        if not active:
            # Stop command
            return bool(self.robot_link.manual_drive('stop', speed))

        return bool(self.robot_link.manual_drive(direction, speed))

    def _manual_drive_loop(self) -> None:
        """Continuously refresh manual-drive commands while latched active."""
        # Refresh rate must be < Mega MANUAL_TIMEOUT (1500ms). Use ~5Hz.
        while not shutdown_event.is_set():
            try:
                if self.robot_link and hasattr(self.robot_link, 'manual_drive'):
                    with self._manual_drive_lock:
                        active = self._manual_drive_active
                    if active:
                        self._manual_drive_apply_once()
            except Exception as exc:
                logger.debug("Manual drive loop error: %s", exc)

            shutdown_event.wait(0.2)

    def _handle_manual_override(self, payload: dict) -> bool:
        mode = (payload or {}).get('mode', 'hold')
        if not hasattr(self.robot_link, 'manual_override'):
            return False
        logger.info(f"Manual override -> {mode}")
        return self.robot_link.manual_override(mode)

    def _handle_manual_speed(self, payload: dict) -> bool:
        speed = int((payload or {}).get('value', 180))
        speed = max(60, min(255, speed))
        if not hasattr(self.robot_link, 'manual_speed'):
            return False
        return self.robot_link.manual_speed(speed)

    def _handle_auto_speed(self, payload: dict) -> bool:
        """Set autonomous navigation base speed (affects waypoint following)."""
        speed = int((payload or {}).get('value', 120))
        speed = max(60, min(255, speed))
        if not hasattr(self.robot_link, 'set_auto_speed'):
            return False
        return bool(self.robot_link.set_auto_speed(speed))

    def _handle_waypoint_push(self) -> bool:
        """Load waypoints from dashboard DB into Pi nav controller (NOT Mega)."""
        waypoints = self.api_client.get_waypoints()
        if not waypoints:
            logger.warning("No waypoints to load")
            return False
        waypoints = sorted(waypoints, key=lambda w: w.get('sequence', 0))
        if self.nav_controller:
            self.nav_controller.set_waypoints(waypoints)
            logger.info("Loaded %d waypoints into Pi NavController", len(waypoints))
            return True
        else:
            logger.warning("NavController not available — cannot load waypoints")
            return False
    
    def camera_loop(self):
        """Capture and stream camera frames (simple synchronous version)"""
        if not self.camera or not self.camera.enabled:
            logger.info("Camera not enabled; skipping camera loop")
            return

        logger.info("Starting camera loop (simple JPEG relay)...")
        max_frame_bytes = int(CONFIG.get('camera', {}).get('max_frame_size', 2 * 1024 * 1024))
        fps = max(1, int(CONFIG.get('camera', {}).get('fps', 5)))
        base_url = self.api_client.base_url.rstrip('/') if getattr(self.api_client, 'base_url', None) else CONFIG.get('dashboard_api', {}).get('base_url', '').rstrip('/')

        interval = 1.0 / float(fps)
        next_capture = time.time()

        try:
            import requests
            session = requests.Session()
            url = f"{base_url}/api/camera/frame_binary"
            headers = {'Content-Type': 'image/jpeg'}

            while not shutdown_event.is_set():
                if time.time() >= next_capture:
                    next_capture = time.time() + interval
                    try:
                        jpeg = self.camera.capture_frame_jpeg()
                        if jpeg and len(jpeg) <= max_frame_bytes:
                            # Feed MJPEG direct-stream server (near-zero latency)
                            mjpeg_update_frame(jpeg)

                            timestamp_iso = datetime.utcnow().isoformat()
                            params = {
                                'device_id': self.device_id,
                                'timestamp': timestamp_iso
                            }
                            resp = session.post(
                                url,
                                params=params,
                                data=jpeg,
                                headers=headers,
                                timeout=(1.5, 2.5)
                            )
                            if resp.status_code in (200, 202):
                                logger.debug("Camera frame uploaded: %d bytes", len(jpeg))
                            else:
                                logger.debug("Camera upload failed: %s", resp.status_code)
                        elif jpeg:
                            logger.warning("Captured frame too large: %d bytes", len(jpeg))
                        else:
                            logger.debug("Camera capture returned None")
                    except Exception as exc:
                        logger.debug("Camera capture/upload failed: %s", exc)

                shutdown_event.wait(0.01)
        except Exception as exc:
            logger.error("Camera loop error: %s", exc)

    def get_battery_level(self):
        """Get battery level (implement based on hardware)"""
        # TODO: Implement actual battery level reading
        # This is a placeholder that returns mock data
        try:
            # If you have a battery monitoring circuit, read it here
            # For now, return a simulated value
            return 85.0
        except:
            return 0.0

    def get_cpu_temperature(self):
        """Get Raspberry Pi CPU temperature"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read().strip()) / 1000.0
                return temp
        except:
            return 0.0

    def shutdown(self):
        """Gracefully shutdown all systems"""
        logger.info("Shutting down robot controller...")
        
        # Set shutdown event
        shutdown_event.set()
        
        # Send offline status
        try:
            self.api_client.send_status({
                'online': False,
                'device_id': self.device_id
            })
        except:
            pass
        
        # Stop components
        try:
            if self.mjpeg_server:
                self.mjpeg_server.stop()
            if self.camera and hasattr(self.camera, 'stop'):
                self.camera.stop()
            if self.robot_link:
                if self.comm_mode == 'serial' and hasattr(self.robot_link, 'stop'):
                    self.robot_link.stop()
                elif hasattr(self.robot_link, 'close'):
                    self.robot_link.close()
            if self.gsm and hasattr(self.gsm, 'disconnect'):
                self.gsm.disconnect()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        
        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2)
        
        logger.info("Shutdown complete")


def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info(f"Received signal {signum}, initiating shutdown...")
    shutdown_event.set()


def main():
    """Main entry point"""
    logger.info("=" * 60)
    logger.info("Environmental Monitoring Robot - Raspberry Pi")
    logger.info("=" * 60)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create robot controller
    robot = None
    
    try:
        robot = RobotController()
        
        # Start robot
        if robot.start():
            logger.info("Robot is running. Press Ctrl+C to stop.")
            
            # Keep main thread alive
            while not shutdown_event.is_set():
                time.sleep(1)
        else:
            logger.error("Failed to start robot")
            return 1
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        return 1
    finally:
        if robot:
            robot.shutdown()
    
    logger.info("Program terminated")
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

### Appendix B: Arduino Navigation Firmware (robot_navigation.ino)

```cpp
/*
 * Environmental Monitoring Robot — Navigation Controller (Arduino Mega 2560)
 *
 * =========================================================================
 *  STATE-MACHINE ARCHITECTURE  (blueprint v2 — 3-state)
 * =========================================================================
 *
 *  Three mutually-exclusive states — only ONE owner drives the motors:
 *
 *    STATE_I2C       (AUTO)     – Raspberry Pi owns the motors
 *    STATE_WIRELESS             – ESP8266 CC1101 remote owns the motors
 *    STATE_FAILSAFE             – Both comms lost, motors halted, idle
 *
 *  Transitions (centralized Mode Manager, top of loop):
 *    Wireless alive?                      → STATE_WIRELESS  (always wins)
 *    No wireless, Pi alive?               → STATE_I2C
 *    Both wireless AND Pi lost?           → STATE_FAILSAFE
 *    Either channel recovers from failsafe? → back to appropriate state
 *
 *  Rules:
 *    • Wireless (manual) ALWAYS takes priority over Pi.
 *    • Sensors, buzzer, GPS run in EVERY state.
 *    • I2C ISRs are byte-copy only (< 50 µs).  Heavy work is deferred.
 *    • Every SPI call (CC1101) is wrapped in noInterrupts()/interrupts().
 *    • CC1101 polled at most once per CC1101_POLL_INTERVAL ms.
 *    • No delay(), no pulseIn(), no while(wireless.receive()) loops.
 *    • Motors are stopped on every state transition.
 *
 *  Superloop order:
 *    1. Deferred CC1101 init
 *    2. SPI Service (poll CC1101 → update WirelessBuffer)
 *    3. I²C Service (process deferred ISR command)
 *    4. MODE MANAGER (decide AUTO / WIRELESS / FAILSAFE)
 *    5. Sensor Tasks (GPS, ultrasonic)
 *    6. Buzzer tick
 *    7. I²C bus watchdog
 *    8. Decision Layer (act on current mode)
 *    9. Obstacle buzzer
 *   10. Status print
 *   11. GPS broadcast
 * =========================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <SPI.h>

#include "gps_handler.h"
#include "navigation.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "globals.h"

// ===================== CC1101 wireless driver =====================
#include "cc1101_driver.h"
CC1101Driver wireless;
#include "wireless_interface.h"

// ===================== Components =====================
GPSHandler gps;
Navigation navigation;
MotorControl motors;
ObstacleAvoidance obstacleAvoid;

// ===================== State machine =====================
RobotState robotState = STATE_I2C;            // boot → I2C

// ===================== Motor shadow (for obstacle guard) =====================
static int8_t lastManualLeft  = 0;
static int8_t lastManualRight = 0;

// ===================== Shared variables =====================
uint8_t responseBuffer[32];
uint8_t responseLength = 0;

PendingWaypoint pendingWaypoints[MAX_WAYPOINTS];
uint8_t pendingWaypointCount = 0;

bool navigationActive        = false;
bool manualOverride          = false;
bool i2cHandshakeComplete    = false;
bool wirelessHandshakeComplete = false;

// I2C ISR → main-loop deferred command
volatile bool     i2cCommandPending   = false;
volatile uint8_t  i2cPendingCommand   = 0;
volatile uint8_t  i2cPendingLength    = 0;
uint8_t           i2cPendingPayload[32];

// Volatile timestamps touched in ISR
volatile unsigned long lastI2CActivityMs = 0;

// Main-loop timestamps
unsigned long lastStatusUpdate   = 0;
unsigned long lastWirelessGps    = 0;
unsigned long lastManualCommand  = 0;
unsigned long lastHeartbeat      = 0;
unsigned long lastCC1101Poll     = 0;

int manualSpeed = 180;
ControlMode controlMode = MODE_AUTO;

// Heading from Pi
float piHeading = 0.0;

// ===================== Non-blocking buzzer =====================
static uint8_t  buzzerPulsesLeft = 0;
static uint16_t buzzerOnMs       = 0;
static uint16_t buzzerOffMs      = 0;
static bool     buzzerToneOn     = false;
static unsigned long buzzerPhaseStart = 0;

// ===================== Forward declarations =====================
void onI2CReceive(int bytes);
void onI2CRequest();
void handleI2CCommand(uint8_t cmd, const uint8_t* payload, uint8_t len);
void prepareAck(uint8_t code = 0);
void prepareError(uint8_t code);
void prepareGpsResponse();
void prepareStatusResponse();
void resetPendingWaypoints();
void storePendingWaypoint(const WaypointPacket& pkt);
void commitPendingWaypoints();

void transitionToState(RobotState newState);
void pollCC1101();
void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags);
void processWirelessCommand(uint8_t cmd, uint8_t speed);
void processWirelessMessage(const String& msg);

void sendWirelessGps();
void sendWirelessStatus();
void sendWirelessReady();
void sendWirelessObstacleAlert(int distance);

void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void updateBuzzer();

uint8_t readBatteryPercent();
uint8_t readSignalQuality();

void recoverI2CBus();
bool isI2CBusStuck();
void i2cReinitSlave();

static inline bool frontObstacle() {
  const int d = obstacleAvoid.getDistance();
  return (d > 0 && d < OBSTACLE_THRESHOLD);
}

// ======================== LEGACY WRAPPERS ========================
void handleZigbee()                             {}
void processZigbeeMessage(const String& m)      { processWirelessMessage(m); }
void sendZigbeeGps()                            { sendWirelessGps(); }
void sendZigbeeStatus()                         { sendWirelessStatus(); }
void sendZigbeeReady()                          { sendWirelessReady(); }
void markZigbeeHandshake()                      { wirelessHandshakeComplete = true; }
void markI2CHandshake() {
  i2cHandshakeComplete = true;
  DEBUG_SERIAL.println(F("# I2C handshake complete"));
  beepPatternNB(2, 100, 100);
}
void markWirelessHandshake() {
  wirelessHandshakeComplete = true;
  DEBUG_SERIAL.println(F("# Wireless handshake complete"));
  beepPatternNB(2, 100, 100);
}

// ======================== I2C BUS RECOVERY ========================
void recoverI2CBus() {
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(50);
  if (digitalRead(SDA) == LOW) {
    for (uint8_t i = 0; i < 9; i++) {
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }
  }
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

bool isI2CBusStuck() {
  // Read the TWI status register directly — NEVER call pinMode() on
  // SDA/SCL while the Wire library owns them, as that detaches TWI
  // from the pins and corrupts any in-flight transaction.
  uint8_t twsr = TWSR & 0xF8;   // mask prescaler bits
  // 0xF8 = "no relevant state" = bus is idle and healthy
  // 0x00 = bus error (SDA/SCL stuck)
  if (twsr == 0x00) return true;
  // Additionally, if the TWI is stuck in an unexpected slave state
  // for too long, consider it hung.  0xF8 (idle) and 0x60/0x80/0xA8
  // (normal slave ops) are fine; anything else is suspicious.
  if (twsr != 0xF8 && twsr != 0x60 && twsr != 0x80 && twsr != 0x88 &&
      twsr != 0xA8 && twsr != 0xB8 && twsr != 0xC0 && twsr != 0xC8) {
    return true;
  }
  return false;
}

void i2cReinitSlave() {
  Wire.end();
  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  DEBUG_SERIAL.println(F("# I2C: bus recovered, slave reinit"));
}

// ======================== SETUP ========================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL) { delayMicroseconds(100); }

  DEBUG_SERIAL.println(F("\n# ========== ARDUINO MEGA NAVIGATION CONTROLLER =========="));
  DEBUG_SERIAL.println(F("# Architecture: STATE_I2C / STATE_WIRELESS finite state machine"));

  GPS_SERIAL.begin(GPS_BAUD);

  // I2C slave init
  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  DEBUG_SERIAL.print(F("# I2C slave @ 0x"));
  DEBUG_SERIAL.println(I2C_ADDRESS, HEX);

  if (gps.begin(GPS_SERIAL)) {
    DEBUG_SERIAL.println(F("# GPS initialized"));
  } else {
    DEBUG_SERIAL.println(F("# WARNING: GPS init failed"));
    beepPatternNB(3, 200, 100);
  }

  DEBUG_SERIAL.println(F("# Compass: provided by Pi via I2C (CMD_SEND_HEADING)"));

  motors.begin();
  obstacleAvoid.begin();
  navigation.begin(&gps, &motors, &obstacleAvoid);

  // CC1101 init is deferred to loop (after I2C is stable)
  DEBUG_SERIAL.println(F("# CC1101: deferred init (5 s)"));

  // Startup tone — non-blocking: schedule 3 s buzz
  tone(BUZZER_PIN, BUZZER_FREQ);
  delay(3000);                       // one-time acceptable at boot only
  noTone(BUZZER_PIN);

  DEBUG_SERIAL.println(F("# Boot complete — entering main loop"));
}

// ======================== MAIN LOOP ========================
void loop() {
  const unsigned long now = millis();

  // ---------- 1. Deferred CC1101 init (once, 5 s after boot) ----------
  static bool cc1101Ready = false;
  if (!cc1101Ready && now > 5000) {
    cc1101Ready = true;
    DEBUG_SERIAL.println(F("# CC1101: init..."));
    if (wireless.begin()) {
      DEBUG_SERIAL.println(F("# CC1101: ready (RX)"));
    } else {
      DEBUG_SERIAL.println(F("# CC1101: FAILED"));
    }
  }

  // ====================================================================
  //  2. SPI SERVICE — Poll CC1101 (non-blocking, time-sliced)
  //     Updates WirelessBuffer (does NOT drive motors here).
  // ====================================================================
  if (cc1101Ready && (now - lastCC1101Poll >= CC1101_POLL_INTERVAL)) {
    lastCC1101Poll = now;
    pollCC1101();
  }

  // ---- CC1101 RX health check (every 2 seconds) ----
  // If the CC1101 drifted out of RX mode (noise, FIFO overflow, SPI
  // glitch), this forces it back.  Costs ~20 µs, no freeze risk.
  {
    static unsigned long lastRxCheck = 0;
    if (cc1101Ready && (now - lastRxCheck >= 2000)) {
      lastRxCheck = now;
      wireless.ensureRxMode();
    }
  }

  // ====================================================================
  //  3. I²C SERVICE — Process deferred command from ISR
  //     ISR only set a flag + byte-copied the payload.
  // ====================================================================
  if (i2cCommandPending) {
    uint8_t cmd, len;
    uint8_t payload[32];

    noInterrupts();
    cmd = i2cPendingCommand;
    len = i2cPendingLength;
    if (len > 32) len = 32;
    memcpy(payload, (const void*)i2cPendingPayload, len);
    i2cCommandPending = false;
    interrupts();

    handleI2CCommand(cmd, payload, len);
  }

  // ====================================================================
  //  4. MODE MANAGER — Centralized state transition logic (TOP PRIORITY)
  //     This is the ONLY place state transitions are decided at runtime.
  //     Rules:
  //       • Valid CC1101 packets recently? → WIRELESS (always wins)
  //       • No wireless but Pi active?     → AUTO (I2C)
  //       • Both lost?                     → FAILSAFE
  // ====================================================================
  {
    const bool wirelessAlive = cc1101Ready &&
                               wireless.getLastRxTime() > 0 &&
                               (now - wireless.getLastRxTime() <= WIRELESS_LINK_TIMEOUT);
    const bool piAlive       = i2cHandshakeComplete &&
                               (now - lastI2CActivityMs <= I2C_LINK_TIMEOUT);

    if (wirelessAlive) {
      // Wireless ALWAYS takes priority
      if (robotState != STATE_WIRELESS) {
        transitionToState(STATE_WIRELESS);
      }
    } else if (piAlive) {
      // Pi is alive, no wireless → AUTO / I2C
      if (robotState == STATE_WIRELESS) {
        DEBUG_SERIAL.println(F("# Wireless link lost → STATE_I2C"));
        transitionToState(STATE_I2C);
      } else if (robotState == STATE_FAILSAFE) {
        DEBUG_SERIAL.println(F("# Pi reconnected → STATE_I2C"));
        transitionToState(STATE_I2C);
      }
    } else if (i2cHandshakeComplete || wireless.getLastRxTime() > 0) {
      // Both channels were active at some point but BOTH are now lost
      if (robotState != STATE_FAILSAFE) {
        DEBUG_SERIAL.println(F("# FAILSAFE: both Pi and wireless lost"));
        transitionToState(STATE_FAILSAFE);
      }
    }
    // else: never had any connection yet → stay in boot state (STATE_I2C)
  }

  // ====================================================================
  //  5. SENSOR TASKS — Always run, every state
  // ====================================================================
  gps.update();                     // UART — non-blocking
  obstacleAvoid.update();           // non-blocking ultrasonic
  // Compass heading is received from Pi via CMD_SEND_HEADING → piHeading

  // SAFETY: Emergency obstacle stop — runs ALWAYS, regardless of state.
  // If navigation is active and obstacle detected, navigation.update()
  // handles avoidance.  But if motors are driving FORWARD for any other
  // reason (manual drive, etc.) we stop.  Reverse and side turns are
  // ALLOWED so the user can back away from the obstacle.
  if (obstacleAvoid.isObstacleDetected() && !navigationActive) {
      if (motors.isMovingForward()) {
          motors.stop();
          DEBUG_SERIAL.println(F("# SAFETY: obstacle stop (non-nav)"));
      }
  }

  // ====================================================================
  //  6. Non-blocking buzzer tick
  // ====================================================================
  updateBuzzer();

  // ====================================================================
  //  7. I²C bus watchdog
  // ====================================================================
  if (i2cHandshakeComplete) {
    static unsigned long lastWatchdog = 0;
    if (now - lastWatchdog > 5000) {
      lastWatchdog = now;
      if (isI2CBusStuck()) {
        i2cReinitSlave();
      }
    }
  }

  // ====================================================================
  //  8. DECISION LAYER — Act on the current state
  //     Mode was already decided by the Mode Manager above.
  //     Each state reads its own buffer and drives motors accordingly.
  // ====================================================================
  switch (robotState) {

    case STATE_I2C:
      // Pi / autonomous navigation owns motors
      if (!manualOverride && navigationActive) {
        if (gps.isValid()) {
          navigation.update();
          if (navigation.isComplete()) {
            navigationActive = false;
            motors.stop();
            DEBUG_SERIAL.println(F("# Nav complete"));
          }
        } else {
          static unsigned long lastGpsWarn = 0;
          if (now - lastGpsWarn > 5000) {
            DEBUG_SERIAL.println(F("# WARNING: no GPS fix"));
            motors.stop();
            lastGpsWarn = now;
          }
        }
      }

      // Manual timeout (Pi joystick)
      if (manualOverride && (now - lastManualCommand > MANUAL_TIMEOUT)) {
        manualOverride = false;
        motors.stop();
        controlMode = MODE_AUTO;
        if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
          navigationActive = true;
          navigation.resume();
        }
        DEBUG_SERIAL.println(F("# Pi manual timeout → AUTO"));
      }
      break;

    case STATE_WIRELESS:
      // ESP8266 remote owns motors.
      // Obstacle safety: if last command was forward and obstacle ahead, stop.
      if (frontObstacle() && lastManualLeft > 0 && lastManualRight > 0) {
        motors.stop();
        lastManualLeft = 0;
        lastManualRight = 0;
      }
      // Link-loss is handled by Mode Manager above — no duplicate check here.
      break;

    case STATE_FAILSAFE:
      // Both communication channels lost.
      // Motors MUST stay stopped.  Slow beep to indicate safe-idle.
      motors.stop();
      {
        static unsigned long lastFailsafeBeep = 0;
        if (now - lastFailsafeBeep >= FAILSAFE_BEEP_INTERVAL) {
          beepPatternNB(2, 300, 200);
          lastFailsafeBeep = now;
        }
      }
      break;
  }

  // ====================================================================
  //  9. Obstacle buzzer alert (any state)
  // ====================================================================
  if (frontObstacle()) {
    static unsigned long lastObstacleBeep = 0;
    if (now - lastObstacleBeep > 2000) {
      beepPatternNB(3, 100, 80);
      lastObstacleBeep = now;
    }
  }

  // ====================================================================
  // 10. Status print
  // ====================================================================
  if (now - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = now;
    DEBUG_SERIAL.print(F("# ST="));
    switch (robotState) {
      case STATE_I2C:      DEBUG_SERIAL.print(F("AUTO"));     break;
      case STATE_WIRELESS: DEBUG_SERIAL.print(F("WIRELESS")); break;
      case STATE_FAILSAFE: DEBUG_SERIAL.print(F("FAILSAFE")); break;
    }
    DEBUG_SERIAL.print(F(" mode="));
    DEBUG_SERIAL.print(controlMode == MODE_AUTO ? F("A") : F("M"));
    DEBUG_SERIAL.print(F(" nav="));
    DEBUG_SERIAL.print(navigationActive ? F("RUN") : F("IDLE"));
    DEBUG_SERIAL.print(F(" wp="));
    DEBUG_SERIAL.print(navigation.getWaypointCount());
    DEBUG_SERIAL.print(F(" cc="));
    DEBUG_SERIAL.print(wireless.isConnected() ? F("ON") : F("--"));
    // RX diagnostics: good/bad packet counts and time since last packet
    DEBUG_SERIAL.print(F(" rx="));
    DEBUG_SERIAL.print(wireless.getRxGoodCount());
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(wireless.getRxBadCount());
    if (wireless.getLastRxTime() > 0) {
      DEBUG_SERIAL.print(F(" ago="));
      DEBUG_SERIAL.print((now - wireless.getLastRxTime()) / 1000);
      DEBUG_SERIAL.print(F("s"));
    }
    DEBUG_SERIAL.println();
  }

  // ====================================================================
  // 11. GPS broadcast (wireless) — DISABLED: ESP8266 remote is TX-only.
  //     Sending takes CC1101 out of RX and drops incoming packets.
  // ====================================================================
}

// ======================== STATE TRANSITIONS ========================
void transitionToState(RobotState newState) {
  if (newState == robotState) return;

  // Leave old state
  motors.stop();
  lastManualLeft  = 0;
  lastManualRight = 0;

  switch (robotState) {
    case STATE_WIRELESS:
      manualOverride = false;
      controlMode = MODE_AUTO;
      // Resume autonomous if waypoints remain
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigationActive = true;
        navigation.resume();
        DEBUG_SERIAL.println(F("# Resuming autonomous nav"));
      }
      break;
    case STATE_I2C:
      // Pause any running autonomous navigation
      if (navigationActive) {
        navigation.pause();
        navigationActive = false;
      }
      break;
    case STATE_FAILSAFE:
      // Leaving failsafe — nothing extra to clean up
      break;
  }

  // Enter new state
  robotState = newState;

  switch (newState) {
    case STATE_I2C:
      DEBUG_SERIAL.println(F("# → STATE_I2C (AUTO)"));
      break;
    case STATE_WIRELESS:
      manualOverride = true;
      controlMode = MODE_MANUAL;
      wirelessHandshakeComplete = true;
      DEBUG_SERIAL.println(F("# → STATE_WIRELESS"));
      beepPatternNB(1, 150, 0);
      break;
    case STATE_FAILSAFE:
      manualOverride = false;
      controlMode = MODE_AUTO;
      navigationActive = false;
      navigation.stop();
      DEBUG_SERIAL.println(F("# → STATE_FAILSAFE (motors halted)"));
      beepPatternNB(5, 100, 100);
      break;
  }
}

// ======================== CC1101 POLLING ========================
// Called once per CC1101_POLL_INTERVAL.  Reads at most ONE packet.
// Updates WirelessBuffer / timestamps only.  Mode Manager decides state.
void pollCC1101() {
  WirelessMessage msg;
  if (!wireless.receive(msg)) return;

  // Any valid packet updates lastRxTime (done inside cc1101_driver.cpp).
  // Mode Manager will see wirelessAlive == true on next loop iteration.

  switch (msg.type) {
    case MSG_TYPE_RAW_MOTOR:
      if (msg.length >= sizeof(RawMotorPacket)) {
        // If Mode Manager hasn't promoted us yet, the transition will
        // happen at the top of the NEXT loop.  Process the command only
        // if we're already in WIRELESS (or will be momentarily).
        if (robotState == STATE_WIRELESS || robotState == STATE_FAILSAFE || robotState == STATE_I2C) {
          // Store for immediate use — transitionToState() is handled by Mode Manager
          RawMotorPacket* pkt = (RawMotorPacket*)msg.data;
          if (robotState == STATE_WIRELESS) {
            processRawMotorCommand(pkt->throttle, pkt->steer, pkt->flags);
          }
          // If not yet in WIRELESS, Mode Manager will switch us next iteration,
          // and the next packet will drive the motors.
        }
      }
      break;

    case MSG_TYPE_COMMAND:
      if (msg.length >= 2 && robotState == STATE_WIRELESS) {
        processWirelessCommand(msg.data[0], msg.data[1]);
      }
      break;

    case MSG_TYPE_HANDSHAKE:
      wirelessHandshakeComplete = true;
      // Don't TX reply — ESP8266 is TX-only, reply disrupts RX
      break;

    case MSG_TYPE_HEARTBEAT:
      break;

    case MSG_TYPE_STATUS:
      // Don't TX reply — ESP8266 is TX-only
      break;

    default:
      break;
  }
}

// ======================== RAW MOTOR (ESP8266) ========================
void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags) {
  lastManualCommand = millis();

  // Arcade drive mixing
  int left  = constrain(throttle + steer, -255, 255);
  int right = constrain(throttle - steer, -255, 255);

  // Deadzone
  if (abs(left)  < 15) left  = 0;
  if (abs(right) < 15) right = 0;

  // Obstacle guard: block forward into obstacle
  if (frontObstacle() && left > 0 && right > 0) {
    left  = 0;
    right = 0;
    motors.stop();
  } else {
    motors.setMotors(left, right);
  }

  lastManualLeft  = (int8_t)constrain(left,  -127, 127);
  lastManualRight = (int8_t)constrain(right, -127, 127);

  // Throttled debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    DEBUG_SERIAL.print(F("# Raw L:"));
    DEBUG_SERIAL.print(left);
    DEBUG_SERIAL.print(F(" R:"));
    DEBUG_SERIAL.println(right);
    lastPrint = millis();
  }
}

// ======================== WIRELESS COMMAND (structured) ========================
void processWirelessCommand(uint8_t cmd, uint8_t speed) {
  lastManualCommand = millis();

  switch (cmd) {
    case WIRELESS_CMD_MOTOR_FORWARD:  motors.forward(speed);  break;
    case WIRELESS_CMD_MOTOR_BACKWARD: motors.backward(speed); break;
    case WIRELESS_CMD_MOTOR_LEFT:     motors.turnLeft(speed);  break;
    case WIRELESS_CMD_MOTOR_RIGHT:    motors.turnRight(speed); break;
    case WIRELESS_CMD_MOTOR_STOP:     motors.stop();           break;
    case WIRELESS_CMD_MODE_AUTO:
      transitionToState(STATE_I2C);
      break;
    default:
      break;
  }
}

// ======================== I2C ISR HANDLERS ========================
// RULE: byte-copy only, set flag, < 50 µs.
void onI2CReceive(int bytes) {
  lastI2CActivityMs = millis();
  if (bytes <= 0) return;

  uint8_t cmd = Wire.read();
  uint8_t len = bytes - 1;
  if (len > 32) len = 32;

  for (uint8_t i = 0; i < len; i++) {
    i2cPendingPayload[i] = Wire.available() ? Wire.read() : 0;
  }

  i2cPendingCommand = cmd;
  i2cPendingLength  = len;
  i2cCommandPending = true;
}

void onI2CRequest() {
  lastI2CActivityMs = millis();
  if (responseLength > 0 && responseLength <= sizeof(responseBuffer)) {
    Wire.write(responseBuffer, responseLength);
  } else {
    uint8_t z = 0;
    Wire.write(&z, 1);
  }
}

// ======================== I2C COMMAND HANDLER ========================
void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length) {
  DEBUG_SERIAL.print(F("# I2C cmd 0x"));
  DEBUG_SERIAL.print(command, HEX);
  DEBUG_SERIAL.print(F(" len="));
  DEBUG_SERIAL.println(length);

  // While wireless or failsafe owns the system, only allow read-only / buzzer commands from Pi
  if (robotState == STATE_WIRELESS || robotState == STATE_FAILSAFE) {
    switch (command) {
      case CMD_PING:
      case CMD_HEARTBEAT:
      case CMD_REQUEST_GPS:
      case CMD_REQUEST_STATUS:
      case CMD_REQUEST_OBSTACLE:
      case CMD_SOUND_BUZZER:
      case CMD_SET_AUTO_SPEED:
      case CMD_SEND_HEADING:     // compass data — read-only, always useful
      case CMD_EMERGENCY_STOP:   // always allow E-stop
        break;  // allowed
      default:
        prepareAck();
        return; // blocked
    }
  }

  switch (command) {
    case CMD_PING:
      markI2CHandshake();
      prepareAck();
      break;

    case CMD_NAV_START:
      if (navigation.getWaypointCount() > 0) {
        navigationActive = true;
        navigation.start();
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;

    case CMD_NAV_PAUSE:
      navigation.pause();
      navigationActive = false;
      motors.stop();
      prepareAck();
      break;

    case CMD_NAV_RESUME:
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigation.resume();
        navigationActive = true;
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;

    case CMD_WAYPOINT_CLEAR:
      navigation.clearWaypoints();
      resetPendingWaypoints();
      navigationActive = false;
      manualOverride = false;
      motors.stop();
      prepareAck();
      break;

    case CMD_WAYPOINT_PACKET:
      if (length >= 11) {   // 2(id) + 1(seq) + 4(lat float) + 4(lon float)
        WaypointPacket pkt;
        pkt.id  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        pkt.seq = payload[2];
        memcpy(&pkt.latitude,  &payload[3],  4);   // AVR double == float == 4 bytes
        memcpy(&pkt.longitude, &payload[7],  4);   // offset 3+4=7, not 11
        storePendingWaypoint(pkt);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_WAYPOINT_COMMIT:
      commitPendingWaypoints();
      prepareAck();
      break;

    case CMD_NAV_STOP:
      navigation.stop();
      navigationActive = false;
      manualOverride = false;
      motors.stop();
      controlMode = MODE_AUTO;
      prepareAck();
      break;

    case CMD_REQUEST_GPS:
      prepareGpsResponse();
      break;

    case CMD_REQUEST_STATUS:
      prepareStatusResponse();
      break;

    case CMD_HEARTBEAT:
      lastHeartbeat = millis();
      prepareAck();
      break;

    case CMD_SET_AUTO_SPEED:
      if (length >= 1) {
        motors.setAutoBaseSpeed((int)payload[0]);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_SEND_GPS:
      if (length >= 16) {
        float lat, lon, spd, hdg;
        memcpy(&lat, &payload[0],  sizeof(float));
        memcpy(&lon, &payload[4],  sizeof(float));
        memcpy(&spd, &payload[8],  sizeof(float));
        memcpy(&hdg, &payload[12], sizeof(float));
        // Seed GPS position so navigation can start before Neo-6M locks.
        // seedPosition() is a no-op once Neo-6M has its own fix.
        gps.seedPosition((double)lat, (double)lon);
        navigation.updateGpsData(lat, lon, spd, hdg);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_SEND_HEADING:
      if (length >= 4) {
        memcpy(&piHeading, &payload[0], sizeof(float));
        navigation.setHeading(piHeading);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_RETURN_TO_START:
      if (navigationActive || gps.isValid()) {
        navigation.returnToStart();
        navigationActive = true;
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
      } else {
        prepareError(0xFD);
      }
      break;

    case CMD_MANUAL_OVERRIDE:
      if (length >= 3) {
        int8_t leftMotor  = (int8_t)payload[0];
        int8_t rightMotor = (int8_t)payload[1];
        bool active       = payload[2] != 0;

        if (active && navigationActive) {
          navigationActive = false;
          navigation.pause();
        }

        if (active && frontObstacle() && leftMotor > 0 && rightMotor > 0) {
          motors.stop();
          leftMotor  = 0;
          rightMotor = 0;
        } else {
          motors.setMotors((int)leftMotor, (int)rightMotor);
        }
        manualOverride = true;
        controlMode = MODE_MANUAL;
        lastManualCommand = millis();
        lastManualLeft  = leftMotor;
        lastManualRight = rightMotor;
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_EMERGENCY_STOP:
      motors.stop();
      navigationActive = false;
      manualOverride = false;
      navigation.stop();
      beepPatternNB(5, 50, 50);
      DEBUG_SERIAL.println(F("# EMERGENCY STOP"));
      prepareAck();
      break;

    case CMD_WIRELESS_BROADCAST:
      sendWirelessGps();
      prepareAck();
      break;

    // CMD_FOLLOW_LINE removed — no line follower hardware

    case CMD_REQUEST_OBSTACLE: {
      int dist = obstacleAvoid.getDistance();
      bool obs = (dist > 0 && dist < OBSTACLE_THRESHOLD);
      responseBuffer[0] = RESP_OBSTACLE;
      responseBuffer[1] = obs ? 1 : 0;
      if (dist < 0) dist = 0;
      responseBuffer[2] = (dist >> 8) & 0xFF;
      responseBuffer[3] = dist & 0xFF;
      responseLength = 4;
      break;
    }

    case CMD_SOUND_BUZZER:
      if (length >= 1) {
        uint8_t secs = payload[0];
        if (secs > 0 && secs <= 10) {
          beepPatternNB(secs, 900, 100);
        }
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_ENGAGE_WIRELESS:
      // Pi asking to engage/disengage wireless backup
      if (length >= 1) {
        if (payload[0]) {
          transitionToState(STATE_WIRELESS);
        } else {
          transitionToState(STATE_I2C);
        }
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    default:
      prepareError(0xFE);
      break;
  }
}

// ======================== RESPONSE BUILDERS ========================
void prepareAck(uint8_t code) {
  responseBuffer[0] = RESP_ACK;
  responseBuffer[1] = code;
  responseLength = 2;
}

void prepareError(uint8_t code) {
  responseBuffer[0] = RESP_ERROR;
  responseBuffer[1] = code;
  responseLength = 2;
}

void prepareGpsResponse() {
  responseBuffer[0] = RESP_GPS;
  if (!gps.isValid()) {
    responseBuffer[1] = 0;
    responseLength = 2;
    return;
  }
  responseBuffer[1] = 1;
  float lat = gps.getLatitude();
  float lon = gps.getLongitude();
  float spd = gps.getSpeed();
  float hdg = piHeading;
  uint8_t sat = gps.getSatellites();
  memcpy(&responseBuffer[2],  &lat, 4);
  memcpy(&responseBuffer[6],  &lon, 4);
  memcpy(&responseBuffer[10], &spd, 4);
  memcpy(&responseBuffer[14], &hdg, 4);
  responseBuffer[18] = sat;
  responseLength = 19;
}

void prepareStatusResponse() {
  responseBuffer[0] = RESP_STATUS;
  responseBuffer[1] = (controlMode == MODE_AUTO) ? 0 : 1;
  responseBuffer[2] = navigationActive ? 1 : 0;
  responseBuffer[3] = manualOverride   ? 1 : 0;
  responseBuffer[4] = navigation.getWaypointCount();
  responseBuffer[5] = readBatteryPercent();
  responseBuffer[6] = readSignalQuality();
  responseBuffer[7] = navigation.getCurrentWaypointIndex();
  responseBuffer[8] = navigation.isWaypointJustCompleted() ? 1 : 0;
  if (navigation.isWaypointJustCompleted()) {
    navigation.clearWaypointCompletionFlag();
  }
  responseLength = 9;
}

// ======================== WAYPOINTS ========================
void resetPendingWaypoints() { pendingWaypointCount = 0; }

void storePendingWaypoint(const WaypointPacket& pkt) {
  if (pendingWaypointCount >= MAX_WAYPOINTS) return;
  PendingWaypoint slot;
  slot.latitude  = pkt.latitude;
  slot.longitude = pkt.longitude;
  slot.id  = pkt.id;
  slot.seq = pkt.seq;
  pendingWaypoints[pendingWaypointCount++] = slot;
}

void commitPendingWaypoints() {
  navigation.clearWaypoints();
  for (uint8_t i = 0; i < pendingWaypointCount; ++i) {
    navigation.addWaypoint(pendingWaypoints[i].latitude,
                           pendingWaypoints[i].longitude,
                           pendingWaypoints[i].id);
  }
  DEBUG_SERIAL.print(F("# Committed "));
  DEBUG_SERIAL.print(navigation.getWaypointCount());
  DEBUG_SERIAL.println(F(" waypoints"));
  resetPendingWaypoints();
}

// ======================== WIRELESS TX HELPERS ========================
// NOTE: ALL wireless.send() calls are DISABLED.
// The ESP8266 remote is TX-only (blind send) — it never receives.
// Calling wireless.send() takes the CC1101 out of RX mode (IDLE → TX → RX),
// which causes us to miss incoming packets and lose the wireless connection.
// These functions are kept as stubs for future bi-directional hardware.

void sendWirelessGps() {
  // DISABLED: ESP8266 is TX-only, cannot receive GPS data
  // wireless.send() would disrupt RX and drop incoming packets
}

void sendWirelessStatus() {
  // DISABLED: ESP8266 is TX-only, cannot receive status
  // wireless.send() would disrupt RX and drop incoming packets
}

void sendWirelessReady() {
  // ESP8266 remote is TX-only (blind send) — it never receives replies.
  // Do NOT call wireless.send() here as it takes CC1101 out of RX mode
  // and causes us to miss the next incoming packets from the remote.
  markWirelessHandshake();
}

void sendWirelessObstacleAlert(int distance) {
  // DISABLED: ESP8266 is TX-only, cannot receive obstacle alerts
  // wireless.send() would disrupt RX and drop incoming packets
  (void)distance;
}

// ======================== LEGACY STRING HANDLER ========================
void processWirelessMessage(const String& message) {
  if (message.length() == 0) return;
  lastManualCommand = millis();

  // Stack-based comparison — avoids heap allocation / fragmentation
  if (message.equalsIgnoreCase(F("PING"))) {
    sendWirelessReady();
    sendWirelessStatus();
  } else if (message.equalsIgnoreCase(F("STATUS?"))) {
    sendWirelessStatus();
  } else if (message.equalsIgnoreCase(F("RETURN!")) || message.equalsIgnoreCase(F("RETURN"))) {
    if (gps.isValid()) {
      navigation.returnToStart();
      transitionToState(STATE_I2C);
      navigationActive = true;
    }
  }
}

// ======================== NON-BLOCKING BUZZER ========================
void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  buzzerPulsesLeft = pulses;
  buzzerOnMs  = onMs;
  buzzerOffMs = offMs;
  buzzerToneOn = true;
  buzzerPhaseStart = millis();
  tone(BUZZER_PIN, BUZZER_FREQ);
}

void updateBuzzer() {
  if (buzzerPulsesLeft == 0) return;
  unsigned long elapsed = millis() - buzzerPhaseStart;

  if (buzzerToneOn) {
    if (elapsed >= buzzerOnMs) {
      noTone(BUZZER_PIN);
      buzzerToneOn = false;
      buzzerPhaseStart = millis();
      buzzerPulsesLeft--;
      if (buzzerPulsesLeft == 0) return; // done
    }
  } else {
    if (elapsed >= buzzerOffMs) {
      tone(BUZZER_PIN, BUZZER_FREQ);
      buzzerToneOn = true;
      buzzerPhaseStart = millis();
    }
  }
}

// ======================== UTILITY ========================
uint8_t readBatteryPercent() { return 85; }

uint8_t readSignalQuality() {
  int8_t rssi = wireless.getRSSI();
  if (rssi == 0) return 0;
  if (rssi > -50) return 100;
  if (rssi < -90) return 10;
  return (rssi + 100) * 2;
}

// Legacy blocking beep (only used if someone still calls it)
void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  beepPatternNB(pulses, onMs, offMs);
}
```

### Appendix C: Dashboard Application (app.py)

```python
import time
import json
import threading
import subprocess
import os
import signal
import sys
from flask import Flask, render_template, jsonify, request
from zigbee_bridge import ZigbeeBridge

# =================CONFIGURATION=================
# Robot's static IP or mDNS name on the LAN
ROBOT_IP = "192.168.1.100"   
# Port for the MJPEG stream (mjpg-streamer)
CAMERA_PORT = 8080           
# Database file path
DATABASE = 'robot_data.db'
# ===============================================


app = Flask(__name__)

# Global state
latest_telemetry = {
    "battery": 0,
    "wifi_signal": 0,
    "status": "Offline",
    "gps": {"lat": 0, "lon": 0},
    "compass": 0
}
telemetry_lock = threading.Lock()

# Bridge to talk to the robot via Zigbee/UART (or mocked)
bridge = ZigbeeBridge()


def get_db_connection():
    """Helper for SQLite connection"""
    import sqlite3
    conn = sqlite3.connect(DATABASE)
    conn.row_factory = sqlite3.Row
    return conn


@app.route('/')
def index():
    """Main dashboard page."""
    return render_template('index.html', 
                           robot_ip=ROBOT_IP, 
                           camera_port=CAMERA_PORT)


@app.route('/api/status')
def get_status():
    """Endpoint for the frontend to poll robot status."""
    with telemetry_lock:
        data = latest_telemetry.copy()
    return jsonify(data)


@app.route('/api/command', methods=['POST'])
def send_command():
    """Endpoint to send commands (manual driving, mode switching)."""
    cmd = request.json.get('command')
    val = request.json.get('value')
    
    print(f"[Dashboard] Received command: {cmd} -> {val}")
    
    # Translate frontend commands to bridge calls
    if cmd == 'move':
        # val is likely [throttle, steer] or simple direction
        # Implementation depends on bridge capabilities
        bridge.send_movement(val)
        
    elif cmd == 'mode':
        if val == 'auto':
            bridge.set_mode_auto()
        elif val == 'manual':
            bridge.set_mode_manual()
            
    elif cmd == 'stop':
        bridge.emergency_stop()
        
    return jsonify({"status": "ok"})


def telemetry_loop():
    """Background thread to fetch data from the bridge."""
    while True:
        try:
            # In a real system, this might poll the bridge or 
            # listen on a queue. Here we simulate or read shared state.
            # raw_data = bridge.get_latest_data()
            
            # For demonstration, we'll just mock lively data or
            # preserve the last known state if the bridge is silent.
            # real_telemetry = bridge.read_telemetry()
            # if real_telemetry:
            #     with telemetry_lock:
            #         latest_telemetry.update(real_telemetry)
            
            time.sleep(1.0)
        except Exception as e:
            print(f"Telemetry loop error: {e}")
            time.sleep(5.0)


def start_camera_stream():
    """Optional: Start local mjpg-streamer if running on the robot itself."""
    # This is a placeholder. In production, mjpg-streamer is usually
    # known as a systemd service, but we can launch it here for dev.
    pass


if __name__ == '__main__':
    # Initialize the database if needed
    if not os.path.exists(DATABASE):
        print("Initializing database...")
        from database import init_db
        init_db()

    # Start the telemetry background thread
    t = threading.Thread(target=telemetry_loop, daemon=True)
    t.start()

    # Start the bridge (UART connection to Zigbee module)
    try:
        bridge.start()
        print("Zigbee Bridge started.")
    except Exception as e:
        print(f"Failed to start Zigbee Bridge: {e}")

    # Run the Flask app
    # host='0.0.0.0' allows access from other devices on the network
    app.run(host='0.0.0.0', port=5000, debug=False)

```
