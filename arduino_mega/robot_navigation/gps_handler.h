/*
 * GPS Handler Header
 * Handles GPS module (NEO-6M or similar)
 */

#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPSHandler {
private:
    TinyGPSPlus gps;
    HardwareSerial* gpsSerial;
    bool valid;
    float latitude;
    float longitude;
    float altitude;
    float speed;
    uint8_t satellites;
    
public:
    GPSHandler();
    bool begin(HardwareSerial& serial);
    void update();
    bool isValid();
    
    float getLatitude();
    float getLongitude();
    float getAltitude();
    float getSpeed();
    uint8_t getSatellites();
    
    void printInfo();
};

#endif
