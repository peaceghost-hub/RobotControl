/*
 * Compass Handler Implementation
 */

#include "compass_handler.h"

CompassHandler::CompassHandler() {
    heading = 0;
    declination = 0;  // Set based on your location
    initialized = false;
}

bool CompassHandler::begin() {
    Wire.begin();
    
    compass = HMC5883L();
    
    if (!compass.begin()) {
        Serial.println(F("# Compass init failed"));
        return false;
    }
    
    // Set measurement mode
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_30HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
    
    initialized = true;
    return true;
}

void CompassHandler::update() {
    if (!initialized) return;
    
    Vector norm = compass.readNormalize();
    
    // Calculate heading
    float headingRad = atan2(norm.YAxis, norm.XAxis);
    
    // Apply declination
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
    heading = headingRad * 180.0 / PI;
}

float CompassHandler::getHeading() {
    return heading;
}

void CompassHandler::setDeclination(float dec) {
    // Declination in radians
    // Find your declination: https://www.magnetic-declination.com/
    declination = dec * PI / 180.0;
}

void CompassHandler::calibrate() {
    Serial.println(F("# Compass calibration: Rotate robot 360 degrees"));
    // Add calibration routine here
    delay(10000);
    Serial.println(F("# Calibration complete"));
}
