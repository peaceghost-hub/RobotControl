#include "compass_handler.h"
#include <Arduino.h>
#include <Wire.h>

CompassHandler::CompassHandler() 
  : compassType(COMPASS_NONE),
    qmcAddress(0),
    heading(0), 
    declination(0),  // Set based on your location
    initialized(false) {
}

bool CompassHandler::begin() {
    // Try HMC5883L first (0x1E)
    if (beginHmc()) {
        return true;
    }

    // Try QMC5883L at common addresses (0x2C reported, also 0x0D)
    if (beginQmc(0x2C)) {
        return true;
    }

    if (beginQmc(0x0D)) {
        return true;
    }

    return false;
}

void CompassHandler::update() {
    if (!initialized) return;

    float headingRad = 0.0f;

    if (compassType == COMPASS_HMC5883L) {
        // Get a new sensor event
        compass.getEvent(&event);
        headingRad = atan2(event.magnetic.y, event.magnetic.x);
    } else if (compassType == COMPASS_QMC5883L) {
        int16_t x = 0, y = 0, z = 0;
        if (!readQmcRaw(x, y, z)) {
            return;
        }
        headingRad = atan2((float)y, (float)x);
    } else {
        return;
    }
    
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

bool CompassHandler::isValid() const {
    return initialized;
}

void CompassHandler::setDeclination(float dec) {
    declination = dec * (PI / 180);  // Convert from degrees to radians
}

void CompassHandler::calibrate() {
    // Calibration would go here
    // This typically involves rotating the device in a figure-8 pattern
    // and collecting min/max values for each axis
}

bool CompassHandler::beginHmc() {
    if (!compass.begin()) {
        return false;
    }
    compass.setMagGain(HMC5883_MAGGAIN_1_3);  // 1.3 gain
    compassType = COMPASS_HMC5883L;
    initialized = true;
    return true;
}

bool CompassHandler::beginQmc(uint8_t address) {
    // QMC5883L basic setup (continuous mode)
    Wire.beginTransmission(address);
    Wire.write(0x0B); // Set/Reset period register
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    Wire.beginTransmission(address);
    Wire.write(0x09); // Control register
    Wire.write(0x1D); // OSR=512, RNG=2G, ODR=100Hz, MODE=Continuous
    if (Wire.endTransmission() != 0) {
        return false;
    }

    compassType = COMPASS_QMC5883L;
    qmcAddress = address;
    initialized = true;
    return true;
}

bool CompassHandler::readQmcRaw(int16_t& x, int16_t& y, int16_t& z) {
    if (!initialized || compassType != COMPASS_QMC5883L) {
        return false;
    }

    Wire.beginTransmission(qmcAddress);
    Wire.write(0x00); // Data register start
    if (Wire.endTransmission() != 0) {
        return false;
    }

    Wire.requestFrom(qmcAddress, (uint8_t)6);
    if (Wire.available() < 6) {
        return false;
    }

    uint8_t xL = Wire.read();
    uint8_t xH = Wire.read();
    uint8_t yL = Wire.read();
    uint8_t yH = Wire.read();
    uint8_t zL = Wire.read();
    uint8_t zH = Wire.read();

    x = (int16_t)(xH << 8 | xL);
    y = (int16_t)(yH << 8 | yL);
    z = (int16_t)(zH << 8 | zL);
    return true;
}
