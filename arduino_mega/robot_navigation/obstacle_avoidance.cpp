/*
 * Obstacle Avoidance Implementation
 */

#include "obstacle_avoidance.h"

ObstacleAvoidance::ObstacleAvoidance() {
    distance = 0;
    obstacleDetected = false;
    lastCheck = 0;
}

void ObstacleAvoidance::begin() {
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
}

void ObstacleAvoidance::update() {
    if (millis() - lastCheck >= CHECK_INTERVAL) {
        distance = measureDistance();
        obstacleDetected = (distance > 0 && distance < OBSTACLE_THRESHOLD);
        lastCheck = millis();
    }
}

bool ObstacleAvoidance::isObstacleDetected() {
    return obstacleDetected;
}

int ObstacleAvoidance::getDistance() {
    return distance;
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
