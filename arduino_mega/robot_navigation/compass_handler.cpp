#include "compass_handler.h"
#include <Arduino.h>
#include <Wire.h>

CompassHandler::CompassHandler() 
  : heading(0), 
    declination(0),  // Set based on your location
    initialized(false) {
}

bool CompassHandler::begin() {
    if (!compass.begin()) {
        return false;
    }
    compass.setMagGain(HMC5883_MAGGAIN_1_3);  // 1.3 gain
    initialized = true;
    return true;
}

void CompassHandler::update() {
    if (!initialized) return;
    
    // Get a new sensor event
    compass.getEvent(&event);
    
    // Calculate heading when the magnetometer is level, then the direction of the heading
    // is equal to: atan2(y, x) in radians
    float headingRad = atan2(event.magnetic.y, event.magnetic.x);
    
    // Add declination (convert to radians and add)
    headingRad += declination;
    
    // Correct for when signs are reversed
    if (headingRad < 0) {
        headingRad += 2 * PI;
    }
    
    // Check for wrap due to addition of declination
    if (headingRad > 2 * PI) {
        headingRad -= 2 * PI;
    }
    
    // Convert to degrees
    heading = headingRad * 180 / PI;
}

float CompassHandler::getHeading() const {
    return heading;
}

void CompassHandler::setDeclination(float dec) {
    declination = dec * (PI / 180);  // Convert from degrees to radians
}

void CompassHandler::calibrate() {
    // Calibration would go here
    // This typically involves rotating the device in a figure-8 pattern
    // and collecting min/max values for each axis
}
