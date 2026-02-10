/*
 * Motor Control Header
 * Controls DC motors using L298N driver
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Motor pins - Adjust based on your wiring
#define MOTOR_LEFT_EN   5
#define MOTOR_LEFT_IN1  22
#define MOTOR_LEFT_IN2  23

#define MOTOR_RIGHT_EN  6
#define MOTOR_RIGHT_IN1 24
#define MOTOR_RIGHT_IN2 25

#define MAX_SPEED 255
#define MIN_SPEED 100

class MotorControl {
private:
    int speedLeft;
    int speedRight;
    int autoBaseSpeed;
    const float TURN_MS_PER_DEG = 10.0f; // approximate timing per degree
    
    // Helper: set motor with signed speed (-255 to +255)
    void setMotor(int speed, uint8_t inA, uint8_t inB, uint8_t enPWM);
    
public:
    MotorControl();
    void begin();

    void setAutoBaseSpeed(int speed);
    int getAutoBaseSpeed() const;
    
    void forward(int speed = 200);
    void backward(int speed = 200);
    void turnLeft(int speed = 150);
    void turnRight(int speed = 150);
    void turnDegrees(int degrees, int speed = 150);
    void stop();
    
    // Arcade drive: set motors from throttle and steer values
    void arcadeDrive(int throttle, int steer);
    
    void setMotors(int left, int right);
    void adjustForHeading(float currentHeading, float targetHeading);
};

#endif
