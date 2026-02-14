 /*
 * Obstacle Avoidance Header — NON-BLOCKING
 * HC-SR04 Ultrasonic Sensor (servo-scanned)
 *
 * Blueprint rules:
 *   - No pulseIn(), no delay() — everything runs in < 20 µs per call.
 *   - Ultrasonic uses a two-phase state machine:
 *       Phase 0: fire trigger pulse, record micros().
 *       Phase 1: poll echo pin; compute distance when echo ends.
 *   - scanPath() is broken into a multi-step state machine driven by
 *     repeated calls to update().
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <Arduino.h>
#include <Servo.h>

// ==================== SENSOR PINS ====================
#define ULTRASONIC_TRIG 30
#define ULTRASONIC_ECHO 31
#define SERVO_PIN 11

// Servo positions
#define SERVO_CENTER 90
#define SERVO_LEFT   160
#define SERVO_RIGHT  20

// Detection thresholds
#define OBSTACLE_THRESHOLD 30  // cm

struct PathScan {
    int centerDist;
    int leftDist;
    int rightDist;
    bool leftClear;
    bool rightClear;
    bool irDetected;   // always false — KY-032 removed
    int irDistance;    // always 0    — KY-032 removed
};

class ObstacleAvoidance {
private:
    Servo servo;
    int distance;                  // latest ultrasonic cm
    bool obstacleDetected;
    unsigned long lastCheck;
    unsigned long lastServoMove;
    int currentServoAngle;
    bool servoAttached;

    // ---- non-blocking ultrasonic state machine ----
    enum UsPhase : uint8_t { US_IDLE, US_TRIGGER, US_WAIT_ECHO_HIGH, US_WAIT_ECHO_LOW };
    UsPhase usPhase;
    unsigned long usTriggerTime;   // micros() when trigger fired
    unsigned long usEchoStart;     // micros() when echo went HIGH
    unsigned long usTimeout;       // absolute micros deadline

    // ---- non-blocking scan state machine ----
    enum ScanStep : uint8_t { SCAN_NONE, SCAN_CENTER_MOVE, SCAN_CENTER_READ,
                               SCAN_LEFT_MOVE, SCAN_LEFT_READ,
                               SCAN_RIGHT_MOVE, SCAN_RIGHT_READ,
                               SCAN_RETURN_CENTER, SCAN_DONE };
    ScanStep scanStep;
    unsigned long scanStepTime;
    PathScan pendingScan;
    bool scanReady;                // true once pendingScan is valid

    static const unsigned long CHECK_INTERVAL = 100;
    static const unsigned long SERVO_SETTLE   = 300;  // ms for servo to stabilise
    
public:
    ObstacleAvoidance();
    void begin();
    void update();                 // call every loop — always < 20 µs
    bool isObstacleDetected();
    int  getDistance();

    // Non-blocking scan API
    void startScan();              // kick off a full L/C/R scan
    bool isScanComplete() const;   // true when pendingScan is valid
    PathScan getScanResult();      // returns last completed scan

    // Legacy blocking scan (kept for navigation.cpp compatibility)
    PathScan scanPath();

    void lookCenter();
    void lookLeft();
    void lookRight();
    bool isServoReady();
    
private:
    void triggerPing();            // starts non-blocking ping
    void updateUltrasonic();       // state machine tick
    void updateScan();             // scan state machine tick
    void moveServoTo(int angle);
};

#endif
