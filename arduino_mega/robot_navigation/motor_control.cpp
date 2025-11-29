/*
 * Motor Control Implementation
 */

#include "motor_control.h"

MotorControl::MotorControl() {
    speedLeft = 0;
    speedRight = 0;
}

void MotorControl::begin() {
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);
    
    stop();
}

void MotorControl::forward(int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Left motor forward
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_EN, speed);
    
    // Right motor forward
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_EN, speed);
    
    speedLeft = speed;
    speedRight = speed;
}

void MotorControl::backward(int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Left motor backward
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    analogWrite(MOTOR_LEFT_EN, speed);
    
    // Right motor backward
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    analogWrite(MOTOR_RIGHT_EN, speed);
    
    speedLeft = -speed;
    speedRight = -speed;
}

void MotorControl::turnLeft(int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Left motor backward
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    analogWrite(MOTOR_LEFT_EN, speed);
    
    // Right motor forward
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_EN, speed);
}

void MotorControl::turnRight(int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Left motor forward
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_EN, speed);
    
    // Right motor backward
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    analogWrite(MOTOR_RIGHT_EN, speed);
}

void MotorControl::stop() {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_EN, 0);
    
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_EN, 0);
    
    speedLeft = 0;
    speedRight = 0;
}

void MotorControl::setMotors(int left, int right) {
    left = constrain(left, -MAX_SPEED, MAX_SPEED);
    right = constrain(right, -MAX_SPEED, MAX_SPEED);
    
    // Left motor
    if (left > 0) {
        digitalWrite(MOTOR_LEFT_IN1, HIGH);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        analogWrite(MOTOR_LEFT_EN, left);
    } else if (left < 0) {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, HIGH);
        analogWrite(MOTOR_LEFT_EN, -left);
    } else {
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        analogWrite(MOTOR_LEFT_EN, 0);
    }
    
    // Right motor
    if (right > 0) {
        digitalWrite(MOTOR_RIGHT_IN1, HIGH);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        analogWrite(MOTOR_RIGHT_EN, right);
    } else if (right < 0) {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, HIGH);
        analogWrite(MOTOR_RIGHT_EN, -right);
    } else {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        analogWrite(MOTOR_RIGHT_EN, 0);
    }
    
    speedLeft = left;
    speedRight = right;
}

void MotorControl::adjustForHeading(float currentHeading, float targetHeading) {
    // Calculate heading error
    float error = targetHeading - currentHeading;
    
    // Normalize error to -180 to 180
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    // PID-like control (simplified)
    int baseSpeed = 180;
    int correction = constrain((int)(error * 2), -80, 80);
    
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    
    setMotors(leftSpeed, rightSpeed);
}
