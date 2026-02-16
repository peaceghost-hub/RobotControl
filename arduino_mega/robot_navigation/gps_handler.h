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
    double latitude;
    double longitude;
    float altitude;
    float speed;
    uint8_t satellites;
    uint8_t lastProcessedChars;
    uint16_t droppedChars;
    uint16_t maxBacklog;
    bool seeded;           // true if position was seeded from Pi/SIM7600E
    bool neoHasFixedOnce;  // true once Neo-6M gets its first real fix

    static const uint8_t MAX_CHARS_PER_UPDATE = 32;
    
public:
    GPSHandler();
    bool begin(HardwareSerial& serial);
    void update();
    bool isValid();
    bool isSeeded() const;  // true if current position is from seed (not Neo-6M)
    
    // Accept a position from Pi (SIM7600E) as a seed.
    // Used until Neo-6M gets its own fix, then ignored.
    void seedPosition(double lat, double lon);
    
    double getLatitude();
    double getLongitude();
    float getAltitude();
    float getSpeed();
    uint8_t getSatellites();
    uint8_t getLastProcessedChars() const;
    uint16_t getDroppedChars() const;
    uint16_t getMaxBacklog() const;
    
    void printInfo();
};

#endif
