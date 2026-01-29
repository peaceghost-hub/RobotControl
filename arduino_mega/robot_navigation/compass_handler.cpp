#include "compass_handler.h"
#include <Arduino.h>
#include <Wire.h>

CompassHandler::CompassHandler() 
  : compassType(COMPASS_NONE),
    qmcAddress(0),
#if COMPASS_USE_SOFT_I2C
    hmcAddress(0),
    sdaPin(COMPASS_SDA_PIN),
    sclPin(COMPASS_SCL_PIN),
#endif
    heading(0), 
    declination(0),  // Set based on your location
    initialized(false) {
}

bool CompassHandler::begin() {
#if COMPASS_USE_SOFT_I2C
    i2cInit();
#endif
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

    // Some modules respond at 0x0C
    if (beginQmc(0x0C)) {
        return true;
    }

    return false;
}

void CompassHandler::update() {
    if (!initialized) return;

    float headingRad = 0.0f;

    if (compassType == COMPASS_HMC5883L) {
        int16_t x = 0, y = 0, z = 0;
        if (!readHmcRaw(x, y, z)) {
            return;
        }
        headingRad = atan2((float)y, (float)x);
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

const char* CompassHandler::getTypeName() const {
    switch (compassType) {
        case COMPASS_HMC5883L:
            return "HMC5883L";
        case COMPASS_QMC5883L:
            return "QMC5883L";
        default:
            return "UNKNOWN";
    }
}

uint8_t CompassHandler::getAddress() const {
    if (compassType == COMPASS_QMC5883L) {
        return qmcAddress;
    }
#if COMPASS_USE_SOFT_I2C
    if (compassType == COMPASS_HMC5883L) {
        return hmcAddress;
    }
#endif
    return 0;
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
#if COMPASS_USE_SOFT_I2C
    hmcAddress = 0x1E;

    // Config registers for HMC5883L
    if (!writeRegister(hmcAddress, 0x00, 0x70)) { // 8-average, 15 Hz
        return false;
    }
    if (!writeRegister(hmcAddress, 0x01, 0x20)) { // Gain 1.3
        return false;
    }
    if (!writeRegister(hmcAddress, 0x02, 0x00)) { // Continuous measurement
        return false;
    }

    compassType = COMPASS_HMC5883L;
    initialized = true;
    return true;
#else
    if (!compass.begin()) {
        return false;
    }
    compass.setMagGain(HMC5883_MAGGAIN_1_3);  // 1.3 gain
    compassType = COMPASS_HMC5883L;
    initialized = true;
    return true;
#endif
}

bool CompassHandler::beginQmc(uint8_t address) {
#if COMPASS_USE_SOFT_I2C
    // QMC5883L basic setup (continuous mode)
    if (!writeRegister(address, 0x0B, 0x01)) { // Set/Reset period
        return false;
    }

    if (!writeRegister(address, 0x09, 0x1D)) { // OSR=512, RNG=2G, ODR=100Hz, Continuous
        return false;
    }

    compassType = COMPASS_QMC5883L;
    qmcAddress = address;
    initialized = true;
    return true;
#else
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
#endif
}

bool CompassHandler::readQmcRaw(int16_t& x, int16_t& y, int16_t& z) {
    if (!initialized || compassType != COMPASS_QMC5883L) {
        return false;
    }
#if COMPASS_USE_SOFT_I2C
    uint8_t buffer[6];
    if (!readRegisters(qmcAddress, 0x00, buffer, 6)) {
        return false;
    }

    uint8_t xL = buffer[0];
    uint8_t xH = buffer[1];
    uint8_t yL = buffer[2];
    uint8_t yH = buffer[3];
    uint8_t zL = buffer[4];
    uint8_t zH = buffer[5];
#else
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
#endif

    x = (int16_t)(xH << 8 | xL);
    y = (int16_t)(yH << 8 | yL);
    z = (int16_t)(zH << 8 | zL);
    return true;
}

bool CompassHandler::readHmcRaw(int16_t& x, int16_t& y, int16_t& z) {
    if (!initialized || compassType != COMPASS_HMC5883L) {
        return false;
    }

#if COMPASS_USE_SOFT_I2C
    uint8_t buffer[6];
    if (!readRegisters(hmcAddress, 0x03, buffer, 6)) {
        return false;
    }

    uint8_t xH = buffer[0];
    uint8_t xL = buffer[1];
    uint8_t zH = buffer[2];
    uint8_t zL = buffer[3];
    uint8_t yH = buffer[4];
    uint8_t yL = buffer[5];

    x = (int16_t)(xH << 8 | xL);
    y = (int16_t)(yH << 8 | yL);
    z = (int16_t)(zH << 8 | zL);
    return true;
#else
    // Adafruit library uses Wire directly
    compass.getEvent(&event);
    x = (int16_t)event.magnetic.x;
    y = (int16_t)event.magnetic.y;
    z = (int16_t)event.magnetic.z;
    return true;
#endif
}

#if COMPASS_USE_SOFT_I2C
void CompassHandler::i2cInit() {
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
}

void CompassHandler::i2cStart() {
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(5);
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
}

void CompassHandler::i2cStop() {
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(5);
    pinMode(sdaPin, INPUT_PULLUP);
    delayMicroseconds(5);
}

bool CompassHandler::i2cWriteByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            pinMode(sdaPin, INPUT_PULLUP);
        } else {
            pinMode(sdaPin, OUTPUT);
            digitalWrite(sdaPin, LOW);
        }
        delayMicroseconds(2);
        pinMode(sclPin, INPUT_PULLUP);
        delayMicroseconds(5);
        pinMode(sclPin, OUTPUT);
        digitalWrite(sclPin, LOW);
        delayMicroseconds(2);
        data <<= 1;
    }

    // Read ACK
    pinMode(sdaPin, INPUT_PULLUP);
    delayMicroseconds(2);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(5);
    bool ack = (digitalRead(sdaPin) == LOW);
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(2);
    return ack;
}

uint8_t CompassHandler::i2cReadByte(bool ack) {
    uint8_t data = 0;
    pinMode(sdaPin, INPUT_PULLUP);

    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        pinMode(sclPin, INPUT_PULLUP);
        delayMicroseconds(5);
        if (digitalRead(sdaPin)) {
            data |= 0x01;
        }
        pinMode(sclPin, OUTPUT);
        digitalWrite(sclPin, LOW);
        delayMicroseconds(2);
    }

    // Send ACK/NACK
    if (ack) {
        pinMode(sdaPin, OUTPUT);
        digitalWrite(sdaPin, LOW);
    } else {
        pinMode(sdaPin, INPUT_PULLUP);
    }
    delayMicroseconds(2);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(5);
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(2);
    pinMode(sdaPin, INPUT_PULLUP);
    return data;
}

bool CompassHandler::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
    i2cStart();
    if (!i2cWriteByte((address << 1) | 0)) {
        i2cStop();
        return false;
    }
    if (!i2cWriteByte(reg)) {
        i2cStop();
        return false;
    }
    if (!i2cWriteByte(value)) {
        i2cStop();
        return false;
    }
    i2cStop();
    return true;
}

bool CompassHandler::readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length) {
    i2cStart();
    if (!i2cWriteByte((address << 1) | 0)) {
        i2cStop();
        return false;
    }
    if (!i2cWriteByte(reg)) {
        i2cStop();
        return false;
    }

    i2cStart();
    if (!i2cWriteByte((address << 1) | 1)) {
        i2cStop();
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = i2cReadByte(i < (length - 1));
    }

    i2cStop();
    return true;
}
#endif
