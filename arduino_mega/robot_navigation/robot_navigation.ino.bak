/*
 * Environmental Monitoring Robot - Navigation Controller (Arduino Mega 2560)
 *
 * Responsibilities:
 *  - Waypoint navigation using GPS + compass
 *  - Obstacle avoidance via HC-SR04
 *  - Dual control: autonomous (from Raspberry Pi) or manual (ZigBee joystick)
 *  - I2C slave interface for Raspberry Pi master (address 0x08)
 *  - ZigBee fail-safe manual override channel (Serial2)
 *  - Buzzer feedback for communication status
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "gps_handler.h"
#include "compass_handler.h"
#include "navigation.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"

// ----------------------------- Hardware Mapping -----------------------------
#define DEBUG_SERIAL   Serial
#define GPS_SERIAL     Serial1     // RX1 (19), TX1 (18)
#define ZIGBEE_SERIAL  Serial2     // RX2 (17), TX2 (16)

const uint32_t DEBUG_BAUD = 115200;
const uint32_t GPS_BAUD = 9600;
const uint32_t ZIGBEE_BAUD = 57600;

#define I2C_ADDRESS 0x08
#define BUZZER_PIN 10

// ----------------------------- I2C Protocol ---------------------------------
const uint8_t CMD_PING            = 0x01;
const uint8_t CMD_NAV_START       = 0x02;
const uint8_t CMD_NAV_STOP        = 0x03;
const uint8_t CMD_NAV_PAUSE       = 0x04;
const uint8_t CMD_NAV_RESUME      = 0x05;
const uint8_t CMD_WAYPOINT_CLEAR  = 0x10;
const uint8_t CMD_WAYPOINT_PACKET = 0x11;
const uint8_t CMD_WAYPOINT_COMMIT = 0x12;
const uint8_t CMD_REQUEST_GPS     = 0x20;
const uint8_t CMD_REQUEST_STATUS  = 0x21;
const uint8_t CMD_HEARTBEAT       = 0x30;

const uint8_t RESP_ACK    = 0x80;
const uint8_t RESP_GPS    = 0x81;
const uint8_t RESP_STATUS = 0x82;
const uint8_t RESP_ERROR  = 0xFF;

// Error codes for RESP_ERROR
const uint8_t ERR_NO_WAYPOINTS = 0x01;
const uint8_t ERR_BUFFER_FULL  = 0x02;
const uint8_t ERR_PACKET_SIZE  = 0x03;

struct WaypointPacket {
  uint16_t id;
  uint8_t seq;
  float latitude;
  float longitude;
};

struct PendingWaypoint {
  float latitude;
  float longitude;
  uint16_t id;
  uint8_t seq;
};

// ------------------------------- Components ----------------------------------
GPSHandler gps;
CompassHandler compass;
Navigation navigation;
MotorControl motors;
ObstacleAvoidance obstacleAvoid;

// ------------------------------- State ---------------------------------------
uint8_t responseBuffer[32];
uint8_t responseLength = 0;

PendingWaypoint pendingWaypoints[MAX_WAYPOINTS];
uint8_t pendingWaypointCount = 0;

bool navigationActive = false;
bool manualOverride = false;
bool i2cHandshakeComplete = false;
bool zigbeeHandshakeComplete = false;

unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 2000; // ms

unsigned long lastZigbeeGps = 0;
const unsigned long ZIGBEE_GPS_INTERVAL = 1200; // ms

unsigned long lastManualCommand = 0;
const unsigned long MANUAL_TIMEOUT = 1500; // ms

unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_TIMEOUT = 5000; // ms

int manualSpeed = 180;

String zigbeeBuffer;

enum ControlMode { MODE_AUTO = 0, MODE_MANUAL = 1 };
ControlMode controlMode = MODE_AUTO;

// ------------------------- Forward Declarations ------------------------------
void onI2CReceive(int bytes);
void onI2CRequest();
void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length);
void prepareAck(uint8_t code = 0);
void prepareError(uint8_t code);
void prepareGpsResponse();
void prepareStatusResponse();
void resetPendingWaypoints();
void storePendingWaypoint(const WaypointPacket& packet);
void commitPendingWaypoints();

void enterManualMode();
void exitManualMode(bool resumeAutonomous);
void processManualTimeout();

void handleZigbee();
void processZigbeeMessage(const String& message);
void sendZigbeeGps();
void sendZigbeeStatus();
void sendZigbeeReady();

void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void markI2CHandshake();
void markZigbeeHandshake();
void checkReadyTone();

uint8_t readBatteryPercent();
uint8_t readSignalQuality();

// -----------------------------------------------------------------------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL) {
    delay(5);
  }

  DEBUG_SERIAL.println(F("# Arduino Mega - Navigation Controller"));
  DEBUG_SERIAL.println(F("# Booting subsystems..."));

  GPS_SERIAL.begin(GPS_BAUD);
  ZIGBEE_SERIAL.begin(ZIGBEE_BAUD);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  if (gps.begin(GPS_SERIAL)) {
    DEBUG_SERIAL.println(F("# GPS initialized"));
  } else {
    DEBUG_SERIAL.println(F("# ERROR: GPS init failed"));
  }

  if (compass.begin()) {
    DEBUG_SERIAL.println(F("# Compass initialized"));
  } else {
    DEBUG_SERIAL.println(F("# ERROR: Compass init failed"));
  }

  motors.begin();
  obstacleAvoid.begin();
  navigation.begin(&gps, &compass, &motors, &obstacleAvoid);

  DEBUG_SERIAL.println(F("# Systems online. Awaiting I2C and ZigBee handshakes."));
}

// -----------------------------------------------------------------------------
void loop() {
  gps.update();
  compass.update();
  obstacleAvoid.update();

  if (!manualOverride && navigationActive) {
    navigation.update();

    if (navigation.isComplete()) {
      navigationActive = false;
      motors.stop();
      DEBUG_SERIAL.println(F("# All waypoints completed"));
      sendZigbeeStatus();
    }
  }

  handleZigbee();
  processManualTimeout();

  if (zigbeeHandshakeComplete && (millis() - lastZigbeeGps >= ZIGBEE_GPS_INTERVAL)) {
    sendZigbeeGps();
    lastZigbeeGps = millis();
  }

  if (millis() - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = millis();
    DEBUG_SERIAL.print(F("# Status -> mode:"));
    DEBUG_SERIAL.print(controlMode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
    DEBUG_SERIAL.print(F(" nav:"));
    DEBUG_SERIAL.print(navigationActive ? F("RUN") : (navigation.isComplete() ? F("DONE") : F("IDLE")));
    DEBUG_SERIAL.print(F(" waypoints:"));
    DEBUG_SERIAL.println(navigation.getWaypointCount());
  }

  if (manualOverride && (millis() - lastManualCommand > MANUAL_TIMEOUT)) {
    exitManualMode(true);
  }

  if (navigationActive && (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT)) {
    DEBUG_SERIAL.println(F("# Warning: heartbeat timeout"));
  }
}

// ---------------------------- I2C Handling -----------------------------------
void onI2CReceive(int bytes) {
  if (bytes <= 0) {
    return;
  }

  uint8_t command = Wire.read();
  uint8_t payload[32];
  uint8_t length = 0;

  while (Wire.available() && length < sizeof(payload)) {
    payload[length++] = Wire.read();
  }

  handleI2CCommand(command, payload, length);
  markI2CHandshake();
}

void onI2CRequest() {
  if (responseLength == 0) {
    prepareAck();
  }

  Wire.write(responseBuffer, responseLength);
  responseLength = 0;
}

void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length) {
  switch (command) {
    case CMD_PING:
      prepareAck();
      break;

    case CMD_NAV_START:
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigation.resume();
        navigationActive = true;
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
        sendZigbeeStatus();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;

    case CMD_NAV_STOP:
      navigation.stop();
      navigationActive = false;
      manualOverride = false;
      motors.stop();
      controlMode = MODE_AUTO;
      prepareAck();
      sendZigbeeStatus();
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
        sendZigbeeStatus();
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
      sendZigbeeStatus();
      break;

    case CMD_WAYPOINT_PACKET:
      if (length < sizeof(WaypointPacket)) {
        prepareError(ERR_PACKET_SIZE);
        break;
      }
      if (pendingWaypointCount >= MAX_WAYPOINTS) {
        prepareError(ERR_BUFFER_FULL);
        break;
      }
      WaypointPacket packet;
      memcpy(&packet, payload, sizeof(WaypointPacket));
      storePendingWaypoint(packet);
      prepareAck();
      break;

    case CMD_WAYPOINT_COMMIT:
      commitPendingWaypoints();
      prepareAck();
      sendZigbeeStatus();
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

    default:
      prepareError(0xFE);
      break;
  }
}

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
  float latitude = gps.getLatitude();
  float longitude = gps.getLongitude();
  float speed = gps.getSpeed();
  float heading = compass.getHeading();
  uint8_t satellites = gps.getSatellites();

  memcpy(&responseBuffer[2], &latitude, sizeof(float));
  memcpy(&responseBuffer[6], &longitude, sizeof(float));
  memcpy(&responseBuffer[10], &speed, sizeof(float));
  memcpy(&responseBuffer[14], &heading, sizeof(float));
  responseBuffer[18] = satellites;
  responseLength = 19;
}

void prepareStatusResponse() {
  responseBuffer[0] = RESP_STATUS;
  responseBuffer[1] = (controlMode == MODE_AUTO) ? 0 : 1;
  responseBuffer[2] = navigationActive ? 1 : 0;
  responseBuffer[3] = manualOverride ? 1 : 0;
  responseBuffer[4] = navigation.getWaypointCount();
  responseBuffer[5] = readBatteryPercent();
  responseBuffer[6] = readSignalQuality();
  responseLength = 7;
}

void resetPendingWaypoints() {
  pendingWaypointCount = 0;
}

void storePendingWaypoint(const WaypointPacket& packet) {
  PendingWaypoint& slot = pendingWaypoints[pendingWaypointCount++];
  slot.latitude = packet.latitude;
  slot.longitude = packet.longitude;
  slot.id = packet.id;
  slot.seq = packet.seq;
}

void commitPendingWaypoints() {
  if (pendingWaypointCount == 0) {
    navigation.clearWaypoints();
    return;
  }

  for (uint8_t i = 1; i < pendingWaypointCount; ++i) {
    PendingWaypoint key = pendingWaypoints[i];
    int8_t j = i - 1;
    while (j >= 0 && pendingWaypoints[j].seq > key.seq) {
      pendingWaypoints[j + 1] = pendingWaypoints[j];
      --j;
    }
    pendingWaypoints[j + 1] = key;
  }

  navigation.clearWaypoints();
  for (uint8_t i = 0; i < pendingWaypointCount; ++i) {
    navigation.addWaypoint(pendingWaypoints[i].latitude,
                           pendingWaypoints[i].longitude,
                           pendingWaypoints[i].id);
  }

  pendingWaypointCount = 0;
  navigationActive = false;
  manualOverride = false;
}

// --------------------------- Manual Control ----------------------------------
void enterManualMode() {
  if (!manualOverride) {
    navigation.pause();
    navigationActive = false;
    controlMode = MODE_MANUAL;
    manualOverride = true;
    DEBUG_SERIAL.println(F("# Manual override engaged"));
    sendZigbeeStatus();
  }
  lastManualCommand = millis();
}

void exitManualMode(bool resumeAutonomous) {
  if (!manualOverride) {
    return;
  }
  manualOverride = false;
  motors.stop();
  controlMode = MODE_AUTO;
  DEBUG_SERIAL.println(F("# Manual override released"));

  if (resumeAutonomous && navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
    navigation.resume();
    navigationActive = true;
  }

  sendZigbeeStatus();
}

void processManualTimeout() {
  if (!manualOverride) {
    return;
  }

  if (millis() - lastManualCommand > MANUAL_TIMEOUT) {
    exitManualMode(true);
  }
}

// ---------------------------- ZigBee Handling --------------------------------
void handleZigbee() {
  while (ZIGBEE_SERIAL.available()) {
    char c = ZIGBEE_SERIAL.read();

    if (c == '\r' || c == '\n') {
      if (zigbeeBuffer.length() > 0) {
        processZigbeeMessage(zigbeeBuffer);
        zigbeeBuffer = "";
      }
    } else {
      zigbeeBuffer += c;
      if (zigbeeBuffer.length() > 80) {
        zigbeeBuffer.remove(0, zigbeeBuffer.length() - 80);
      }
    }
  }
}

void processZigbeeMessage(const String& message) {
  if (message.length() == 0) {
    return;
  }

  if (message.startsWith("HELLO")) {
    markZigbeeHandshake();
    ZIGBEE_SERIAL.println(F("ACK,HELLO"));
    sendZigbeeStatus();
    return;
  }

  if (!message.startsWith("MCTL")) {
    return;
  }

  int commaIndex = message.indexOf(',');
  if (commaIndex < 0) {
    return;
  }

  String command = message.substring(commaIndex + 1);
  command.trim();
  command.toUpperCase();

  if (command.startsWith("SPEED")) {
    int valueIndex = command.indexOf('=');
    if (valueIndex > 0) {
      int value = command.substring(valueIndex + 1).toInt();
      manualSpeed = constrain(value, 80, 255);
      DEBUG_SERIAL.print(F("# Manual speed set to "));
      DEBUG_SERIAL.println(manualSpeed);
    }
    return;
  }

  if (command == "FWD") {
    enterManualMode();
    motors.forward(manualSpeed);
  } else if (command == "REV") {
    enterManualMode();
    motors.backward(manualSpeed);
  } else if (command == "LEFT") {
    enterManualMode();
    motors.turnLeft(manualSpeed);
  } else if (command == "RIGHT") {
    enterManualMode();
    motors.turnRight(manualSpeed);
  } else if (command == "STOP") {
    enterManualMode();
    motors.stop();
  } else if (command == "AUTO" || command == "RELEASE") {
    exitManualMode(true);
  } else if (command == "PAUSE") {
    enterManualMode();
    motors.stop();
  }

  lastManualCommand = millis();
  sendZigbeeStatus();
}

void sendZigbeeGps() {
  if (!gps.isValid()) {
    ZIGBEE_SERIAL.println(F("GPS,INVALID"));
    return;
  }

  ZIGBEE_SERIAL.print(F("GPS,"));
  ZIGBEE_SERIAL.print(gps.getLatitude(), 6);
  ZIGBEE_SERIAL.print(',');
  ZIGBEE_SERIAL.print(gps.getLongitude(), 6);
  ZIGBEE_SERIAL.print(',');
  ZIGBEE_SERIAL.print(gps.getSpeed(), 2);
  ZIGBEE_SERIAL.print(',');
  ZIGBEE_SERIAL.print(compass.getHeading(), 2);
  ZIGBEE_SERIAL.print(',');
  ZIGBEE_SERIAL.print(controlMode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
  ZIGBEE_SERIAL.println();
}

void sendZigbeeStatus() {
  if (!zigbeeHandshakeComplete) {
    return;
  }
  ZIGBEE_SERIAL.print(F("MODE,"));
  ZIGBEE_SERIAL.print(controlMode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
  ZIGBEE_SERIAL.print(',');
  ZIGBEE_SERIAL.print(navigationActive ? F("RUN") : (navigation.isComplete() ? F("DONE") : F("IDLE")));
  ZIGBEE_SERIAL.print(',');
  ZIGBEE_SERIAL.println(navigation.getWaypointCount());
}

void sendZigbeeReady() {
  ZIGBEE_SERIAL.println(F("SYS,READY"));
}

// ---------------------------- Utilities --------------------------------------
void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < pulses; ++i) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (offMs > 0 && i < pulses - 1) {
      delay(offMs);
    }
  }
}

void markI2CHandshake() {
  if (!i2cHandshakeComplete) {
    i2cHandshakeComplete = true;
    beepPattern(1, 150, 0);
    DEBUG_SERIAL.println(F("# I2C handshake confirmed"));
    checkReadyTone();
  }
}

void markZigbeeHandshake() {
  if (!zigbeeHandshakeComplete) {
    zigbeeHandshakeComplete = true;
    beepPattern(2, 120, 150);
    DEBUG_SERIAL.println(F("# ZigBee handshake confirmed"));
    checkReadyTone();
    sendZigbeeReady();
  }
}

void checkReadyTone() {
  if (i2cHandshakeComplete && zigbeeHandshakeComplete) {
    beepPattern(1, 600, 0);
    DEBUG_SERIAL.println(F("# System ready"));
  }
}

uint8_t readBatteryPercent() {
  // Placeholder for actual battery monitoring hardware
  return 90;
}

uint8_t readSignalQuality() {
  // Placeholder for RF signal quality measurement
  return 100;
}
