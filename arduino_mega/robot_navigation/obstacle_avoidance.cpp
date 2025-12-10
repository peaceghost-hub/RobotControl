/*
 * Obstacle Avoidance Implementation
 */

#include "obstacle_avoidance.h"

ObstacleAvoidance::ObstacleAvoidance() {
    distance = 0;
    obstacleDetected = false;
    irObstacleDetected = false;
    irValue = 0;
    lastCheck = 0;
    lastServoMove = 0;
    currentServoAngle = SERVO_CENTER;
    servoAttached = false;
    ky032Attached = false;
}

void ObstacleAvoidance::begin() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    
    // Initialize KY-032 infrared sensor
    pinMode(KY032_DO_PIN, INPUT);      // Digital output
    // Analog pin A0 is read-only via analogRead(), no pinMode needed
    ky032Attached = true;
    
    // Try to attach servo with fault tolerance
    servo.attach(SERVO_PIN);
    servoAttached = true;
    servo.write(SERVO_CENTER);
    currentServoAngle = SERVO_CENTER;
    delay(500);  // Let servo reach center
    
    Serial.println(F("# Obstacle avoidance with servo + dual sensors initialized"));
    Serial.println(F("# - HC-SR04 ultrasonic (pins 8-9, servo-scanned)"));
    Serial.println(F("# - KY-032 infrared (pin 2 digital, A0 analog)"));
}

void ObstacleAvoidance::update() {
    if (millis() - lastCheck >= CHECK_INTERVAL) {
        distance = measureDistance();
        obstacleDetected = (distance > 0 && distance < OBSTACLE_THRESHOLD);
        
        // Update KY-032 infrared sensor
        updateIRSensor();
        
        lastCheck = millis();
    }
}

bool ObstacleAvoidance::isObstacleDetected() {
    // Combined logic: obstacle if either sensor detects
    // HC-SR04 provides detailed distance, KY-032 provides immediate warning
    return obstacleDetected || irObstacleDetected;
}

int ObstacleAvoidance::getDistance() {
    return distance;
}

bool ObstacleAvoidance::isIRObstacleDetected() {
    return irObstacleDetected;
}

int ObstacleAvoidance::getIRDistance() {
    // Map ADC value (0-1023) to approximate distance
    // KY-032 is typically max ~5cm range
    // Higher ADC = closer object (inverse relationship)
    if (irValue < 100) return 100;  // No detection, far away
    if (irValue > KY032_DETECTION_THRESHOLD) return 5;  // Close detection
    
    // Linear interpolation for intermediate values
    return map(irValue, 100, KY032_DETECTION_THRESHOLD, 50, 5);
}

int ObstacleAvoidance::getIRAnalogValue() {
    return irValue;
}

int ObstacleAvoidance::measureDistance() {
    // Send ultrasonic pulse
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    // Read echo
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);  // 30ms timeout
    
    if (duration == 0) {
        return -1;  // No echo received
    }
    
    // Calculate distance in cm
    int dist = duration * 0.034 / 2;
    
    return dist;
}

PathScan ObstacleAvoidance::scanPath() {
    PathScan scan;
    
    // Scan center
    lookCenter();
    delay(SERVO_DELAY);
    scan.centerDist = measureDistance();
    
    // Scan left
    lookLeft();
    delay(SERVO_DELAY);
    scan.leftDist = measureDistance();
    scan.leftClear = (scan.leftDist == -1 || scan.leftDist > OBSTACLE_THRESHOLD);
    
    // Scan right
    lookRight();
    delay(SERVO_DELAY);
    scan.rightDist = measureDistance();
    scan.rightClear = (scan.rightDist == -1 || scan.rightDist > OBSTACLE_THRESHOLD);
    
    // Add IR detection result (doesn't change with servo angle, fixed forward-facing)
    scan.irDetected = irObstacleDetected;
    scan.irDistance = irValue;
    
    // Return to center
    lookCenter();
    
    return scan;
}

void ObstacleAvoidance::lookCenter() {
    moveServoTo(SERVO_CENTER);
}

void ObstacleAvoidance::lookLeft() {
    moveServoTo(SERVO_LEFT);
}

void ObstacleAvoidance::lookRight() {
    moveServoTo(SERVO_RIGHT);
}

bool ObstacleAvoidance::isServoReady() {
    return servoAttached && (millis() - lastServoMove >= SERVO_DELAY);
}

void ObstacleAvoidance::moveServoTo(int angle) {
    if (servoAttached) {
        servo.write(angle);
        currentServoAngle = angle;
        lastServoMove = millis();
    }
}

int ObstacleAvoidance::measureDistanceAt(int angle) {
    moveServoTo(angle);
    delay(SERVO_DELAY);
    return measureDistance();
}

void ObstacleAvoidance::updateIRSensor() {
    // Fault tolerance: gracefully handle KY-032 failure
    if (!ky032Attached) {
        irObstacleDetected = false;
        irValue = 0;
        return;
    }
    
    // Read analog value from KY-032 (0-1023)
    // Higher value = closer object (inverse relationship)
    irValue = analogRead(KY032_AO_PIN);
    
    // Detect obstacle based on analog threshold
    irObstacleDetected = (irValue > KY032_DETECTION_THRESHOLD);
    
    // Alternative: could also check digital pin
    // bool digitalDetection = digitalRead(KY032_DO_PIN) == HIGH;
    // Use analog method for smoother response with configurable threshold
}
