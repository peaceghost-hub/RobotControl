/*
 * Compass Handler Header
 * HMC5883L 3-Axis Magnetometer
 */

#ifndef COMPASS_HANDLER_H
#define COMPASS_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L.h>

class CompassHandler {
private:
    HMC5883L compass;
    float heading;
    float declination;  // Magnetic declination for location
    bool initialized;
    
public:
    CompassHandler();
    bool begin();
    void update();
    float getHeading();
    void setDeclination(float dec);
    void calibrate();
};

#endif
