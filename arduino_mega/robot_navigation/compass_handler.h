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

// Software I2C defaults (can be overridden in globals.h)
#ifndef COMPASS_USE_SOFT_I2C
#define COMPASS_USE_SOFT_I2C 0
#endif

#ifndef COMPASS_SDA_PIN
#define COMPASS_SDA_PIN SDA
#endif

#ifndef COMPASS_SCL_PIN
#define COMPASS_SCL_PIN SCL
#endif

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
    const char* getTypeName() const;
    uint8_t getAddress() const;
    void setDeclination(float dec);
    void calibrate();

private:
    enum CompassType {
        COMPASS_NONE,
        COMPASS_HMC5883L,
        COMPASS_QMC5883L
    };

    // Software I2C helpers (defined even if not used to keep class stable)
    void i2cInit();
    void i2cStart();
    void i2cStop();
    bool i2cWriteByte(uint8_t data);
    uint8_t i2cReadByte(bool ack);
    bool readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length);
    bool writeRegister(uint8_t address, uint8_t reg, uint8_t value);

    bool beginHmc();
    bool beginQmc(uint8_t address);
    bool readQmcRaw(int16_t& x, int16_t& y, int16_t& z);
    bool readHmcRaw(int16_t& x, int16_t& y, int16_t& z);

    CompassType compassType;
    uint8_t qmcAddress;
    uint8_t hmcAddress;
    uint8_t sdaPin;
    uint8_t sclPin;
    Adafruit_HMC5883_Unified compass;
    sensors_event_t event;
    float heading;
    float declination;
    bool initialized;
};

#endif // COMPASS_HANDLER_H
