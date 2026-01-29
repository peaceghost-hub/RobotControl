/*
 * Compass Handler Header
 * HMC5883L 3-Axis Magnetometer
 */

#ifndef COMPASS_HANDLER_H
#define COMPASS_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Define PI if not already defined
#ifndef PI
#define PI 3.14159265358979323846
#endif

class CompassHandler {
public:
    CompassHandler();
    bool begin();
    void update();
    float getHeading() const;
    bool isValid() const;
    void setDeclination(float dec);
    void calibrate();

private:
    enum CompassType {
        COMPASS_NONE,
        COMPASS_HMC5883L,
        COMPASS_QMC5883L
    };

    bool beginHmc();
    bool beginQmc(uint8_t address);
    bool readQmcRaw(int16_t& x, int16_t& y, int16_t& z);

    CompassType compassType;
    uint8_t qmcAddress;
    Adafruit_HMC5883_Unified compass;
    sensors_event_t event;
    float heading;
    float declination;
    bool initialized;
};

#endif // COMPASS_HANDLER_H
