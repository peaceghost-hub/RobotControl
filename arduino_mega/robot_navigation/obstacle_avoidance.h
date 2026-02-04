 /*
 * Obstacle Avoidance Header
 * HC-SR04 Ultrasonic Sensor + KY-032 Infrared Sensor
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <Arduino.h>
#include <Servo.h>

// ==================== SENSOR PINS ====================
// Ultrasonic (HC-SR04): Servo-mounted
#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO 9

// Servo Motor: Controls ultrasonic sensor direction
#define SERVO_PIN 11

// Infrared (KY-032): Fixed front-facing obstacle detection
// ├─ DO (Digital Out) → Pin 2  (HIGH when obstacle detected)
// ├─ AO (Analog Out)  → A0     (Analog 0: 0-1023, lower = closer obstacle)
// ├─ GND → GND
// └─ VCC → 5V
#define KY032_DO_PIN 2     // Digital output (active HIGH when obstacle detected)
#define KY032_AO_PIN A0    // Analog output (0-1023, inversely proportional to distance)
// Line follower module pins are defined in globals.h (LINE_FOLLOWER_OUT, LINE_FOLLOWER_ENA)

// Servo positions for ultrasonic scanning
#define SERVO_CENTER 90
#define SERVO_LEFT 160
#define SERVO_RIGHT 20

// Detection thresholds
#define OBSTACLE_THRESHOLD 20      // cm (ultrasonic)
#define KY032_DETECTION_THRESHOLD 600  // ADC value (analog threshold, lower = farther)

struct PathScan {
    int centerDist;
    int leftDist;
    int rightDist;
    bool leftClear;
    bool rightClear;
    bool irDetected;      // KY-032 infrared detection result
    int irDistance;       // KY-032 distance proxy (ADC value)
};

class ObstacleAvoidance {
private:
    Servo servo;
    int distance;
    bool obstacleDetected;
    bool irObstacleDetected;  // KY-032 detection flag
    int irValue;              // KY-032 analog reading (0-1023)
    unsigned long lastCheck;
    unsigned long lastServoMove;
    int currentServoAngle;
    bool servoAttached;
    bool ky032Attached;       // KY-032 initialization flag
    const unsigned long CHECK_INTERVAL = 100;  // ms (reduced from 200ms for real-time response)
    const unsigned long SERVO_DELAY = 300;  // ms for servo to stabilize
    
public:
    ObstacleAvoidance();
    void begin();
    void update();
    bool isObstacleDetected();
    int getDistance();
    bool isIRObstacleDetected();      // New: KY-032 detection
    int getIRDistance();              // New: KY-032 distance proxy
    int getIRAnalogValue();           // New: Raw ADC reading
    PathScan scanPath();
    void lookCenter();
    void lookLeft();
    void lookRight();
    bool isServoReady();
    
private:
    int measureDistance();
    int measureDistanceAt(int angle);
    void moveServoTo(int angle);
    void updateIRSensor();            // New: Update KY-032 reading
};

#endif
