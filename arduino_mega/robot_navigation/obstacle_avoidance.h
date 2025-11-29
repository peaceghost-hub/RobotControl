/*
 * Obstacle Avoidance Header
 * HC-SR04 Ultrasonic Sensor
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <Arduino.h>

#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO 9
#define OBSTACLE_THRESHOLD 30  // cm

class ObstacleAvoidance {
private:
    int distance;
    bool obstacleDetected;
    unsigned long lastCheck;
    const unsigned long CHECK_INTERVAL = 200;  // ms
    
public:
    ObstacleAvoidance();
    void begin();
    void update();
    bool isObstacleDetected();
    int getDistance();
    
private:
    int measureDistance();
};

#endif
