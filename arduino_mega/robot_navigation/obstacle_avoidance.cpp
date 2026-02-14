/*
 * Obstacle Avoidance — NON-BLOCKING Implementation
 *
 * The ultrasonic sensor uses a state machine instead of pulseIn():
 *   US_IDLE          → wait for CHECK_INTERVAL
 *   US_TRIGGER       → 10 µs trigger pulse (instantaneous)
 *   US_WAIT_ECHO_HIGH→ poll echo pin until it goes HIGH (start timing)
 *   US_WAIT_ECHO_LOW → poll echo pin until it goes LOW  (compute distance)
 *
 * Each call to update() costs < 10 µs (just a digitalRead + compare).
 * The scanPath() multi-step machine moves the servo and reads at 3 angles.
 */

#include "obstacle_avoidance.h"

ObstacleAvoidance::ObstacleAvoidance() {
    distance = -1;
    obstacleDetected = false;
    lastCheck = 0;
    lastServoMove = 0;
    currentServoAngle = SERVO_CENTER;
    servoAttached = false;
    usPhase = US_IDLE;
    usTriggerTime = 0;
    usEchoStart = 0;
    usTimeout = 0;
    scanStep = SCAN_NONE;
    scanStepTime = 0;
    scanReady = false;
    memset(&pendingScan, 0, sizeof(pendingScan));
}

void ObstacleAvoidance::begin() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    pinMode(ULTRASONIC_ECHO, INPUT);

    // Servo
    servo.attach(SERVO_PIN);
    servoAttached = true;
    servo.write(SERVO_CENTER);
    currentServoAngle = SERVO_CENTER;
    // No delay() — the servo will reach center before the first ping fires
    // because CHECK_INTERVAL (100 ms) >> servo travel time for small angle.
    lastServoMove = millis();

    Serial.println(F("# Obstacle avoidance (non-blocking) initialized"));
    Serial.println(F("# - HC-SR04 ultrasonic (pins 30-31, servo-scanned)"));
}

// ============ main update — call every loop iteration ============
void ObstacleAvoidance::update() {
    updateUltrasonic();  // non-blocking ping state machine
    updateScan();        // non-blocking scan state machine
}

// ============ non-blocking ultrasonic state machine ============
void ObstacleAvoidance::updateUltrasonic() {
    const unsigned long nowUs = micros();
    const unsigned long nowMs = millis();

    switch (usPhase) {
      case US_IDLE:
        if (nowMs - lastCheck >= CHECK_INTERVAL && scanStep == SCAN_NONE) {
          triggerPing();
        }
        break;

      case US_TRIGGER:
        // Trigger pulse was sent in triggerPing().
        // Immediately transition to waiting for echo HIGH.
        usPhase = US_WAIT_ECHO_HIGH;
        usTimeout = nowUs + 30000UL;  // 30 ms max total wait
        break;

      case US_WAIT_ECHO_HIGH:
        if (digitalRead(ULTRASONIC_ECHO) == HIGH) {
          usEchoStart = nowUs;
          usPhase = US_WAIT_ECHO_LOW;
        } else if ((long)(nowUs - usTimeout) >= 0) {
          // Timeout waiting for echo to start
          distance = -1;
          obstacleDetected = false;
          lastCheck = nowMs;
          usPhase = US_IDLE;
        }
        break;

      case US_WAIT_ECHO_LOW:
        if (digitalRead(ULTRASONIC_ECHO) == LOW) {
          unsigned long duration = nowUs - usEchoStart;
          int dist = (int)(duration * 0.034f / 2.0f);
          distance = dist;
          obstacleDetected = (dist > 0 && dist < OBSTACLE_THRESHOLD);
          lastCheck = nowMs;
          usPhase = US_IDLE;
        } else if ((long)(nowUs - usTimeout) >= 0) {
          // Echo stuck HIGH — no return signal
          distance = -1;
          obstacleDetected = false;
          lastCheck = nowMs;
          usPhase = US_IDLE;
        }
        break;
    }
}

// ---- triggerPing() — fires the 10 µs trigger and switches state ----
void ObstacleAvoidance::triggerPing() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    usTriggerTime = micros();
    usPhase = US_TRIGGER;
}

// ============ non-blocking scan state machine ============
void ObstacleAvoidance::startScan() {
    if (scanStep != SCAN_NONE) return;  // already scanning
    scanReady = false;
    scanStep = SCAN_CENTER_MOVE;
    scanStepTime = millis();
    moveServoTo(SERVO_CENTER);
}

bool ObstacleAvoidance::isScanComplete() const {
    return scanReady;
}

PathScan ObstacleAvoidance::getScanResult() {
    scanReady = false;
    return pendingScan;
}

void ObstacleAvoidance::updateScan() {
    if (scanStep == SCAN_NONE || scanStep == SCAN_DONE) return;

    const unsigned long now = millis();
    const bool servoSettled = (now - scanStepTime >= SERVO_SETTLE);

    switch (scanStep) {
      case SCAN_CENTER_MOVE:
        // Servo was moved in startScan(); wait for settle
        if (servoSettled) {
          triggerPing();
          scanStep = SCAN_CENTER_READ;
        }
        break;

      case SCAN_CENTER_READ:
        if (usPhase == US_IDLE) {
          pendingScan.centerDist = distance;
          moveServoTo(SERVO_LEFT);
          scanStepTime = now;
          scanStep = SCAN_LEFT_MOVE;
        }
        break;

      case SCAN_LEFT_MOVE:
        if (servoSettled) {
          triggerPing();
          scanStep = SCAN_LEFT_READ;
        }
        break;

      case SCAN_LEFT_READ:
        if (usPhase == US_IDLE) {
          pendingScan.leftDist = distance;
          pendingScan.leftClear = (distance == -1 || distance > OBSTACLE_THRESHOLD);
          moveServoTo(SERVO_RIGHT);
          scanStepTime = now;
          scanStep = SCAN_RIGHT_MOVE;
        }
        break;

      case SCAN_RIGHT_MOVE:
        if (servoSettled) {
          triggerPing();
          scanStep = SCAN_RIGHT_READ;
        }
        break;

      case SCAN_RIGHT_READ:
        if (usPhase == US_IDLE) {
          pendingScan.rightDist = distance;
          pendingScan.rightClear = (distance == -1 || distance > OBSTACLE_THRESHOLD);
          pendingScan.irDetected = false;  // KY-032 removed
          pendingScan.irDistance = 0;       // KY-032 removed
          moveServoTo(SERVO_CENTER);
          scanStepTime = now;
          scanStep = SCAN_RETURN_CENTER;
        }
        break;

      case SCAN_RETURN_CENTER:
        if (servoSettled) {
          scanReady = true;
          scanStep = SCAN_NONE;
        }
        break;

      default:
        scanStep = SCAN_NONE;
        break;
    }
}

// ============ Legacy blocking scanPath() — compatibility shim ============
// Navigation code may still call this.  Internally it blocks with the old
// pattern so navigation.cpp keeps working until it's also refactored.
PathScan ObstacleAvoidance::scanPath() {
    PathScan scan;

    // Center
    moveServoTo(SERVO_CENTER);
    delay(SERVO_SETTLE);
    digitalWrite(ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    long d = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
    scan.centerDist = (d == 0) ? -1 : (int)(d * 0.034f / 2.0f);

    // Left
    moveServoTo(SERVO_LEFT);
    delay(SERVO_SETTLE);
    digitalWrite(ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    d = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
    scan.leftDist = (d == 0) ? -1 : (int)(d * 0.034f / 2.0f);
    scan.leftClear = (scan.leftDist == -1 || scan.leftDist > OBSTACLE_THRESHOLD);

    // Right
    moveServoTo(SERVO_RIGHT);
    delay(SERVO_SETTLE);
    digitalWrite(ULTRASONIC_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    d = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
    scan.rightDist = (d == 0) ? -1 : (int)(d * 0.034f / 2.0f);
    scan.rightClear = (scan.rightDist == -1 || scan.rightDist > OBSTACLE_THRESHOLD);

    scan.irDetected = false;  // KY-032 removed
    scan.irDistance = 0;       // KY-032 removed

    moveServoTo(SERVO_CENTER);
    return scan;
}

// ============ simple helpers ============
bool ObstacleAvoidance::isObstacleDetected() {
    return obstacleDetected;
}

int ObstacleAvoidance::getDistance() {
    return distance;
}

void ObstacleAvoidance::lookCenter() { moveServoTo(SERVO_CENTER); }
void ObstacleAvoidance::lookLeft()   { moveServoTo(SERVO_LEFT); }
void ObstacleAvoidance::lookRight()  { moveServoTo(SERVO_RIGHT); }

bool ObstacleAvoidance::isServoReady() {
    return servoAttached && (millis() - lastServoMove >= SERVO_SETTLE);
}

void ObstacleAvoidance::moveServoTo(int angle) {
    if (servoAttached) {
        servo.write(angle);
        currentServoAngle = angle;
        lastServoMove = millis();
    }
}
