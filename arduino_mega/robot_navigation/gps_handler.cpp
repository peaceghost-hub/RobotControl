/*
 * GPS Handler Implementation
 */

#include "gps_handler.h"

GPSHandler::GPSHandler() {
    valid = false;
    latitude = 0;
    longitude = 0;
    altitude = 0;
    speed = 0;
    satellites = 0;
    lastProcessedChars = 0;
    droppedChars = 0;
    maxBacklog = 0;
    seeded = false;
    neoHasFixedOnce = false;
}

bool GPSHandler::begin(HardwareSerial& serial) {
    gpsSerial = &serial;
    return true;
}

void GPSHandler::update() {
    if (gpsSerial == nullptr) return;

    // Bounded parser work per loop pass to prevent loop starvation.
    int avail = gpsSerial->available();
    if (avail > (int)maxBacklog) {
        maxBacklog = (uint16_t)avail;
    }

    uint8_t processed = 0;
    while (gpsSerial->available() > 0 && processed < MAX_CHARS_PER_UPDATE) {
        char c = gpsSerial->read();
        gps.encode(c);
        processed++;
    }
    lastProcessedChars = processed;

    // If UART backlog stays larger than our budget, count a soft drop event.
    // (We are not discarding bytes here; this is a diagnostic indicator.)
    if (gpsSerial->available() > 0) {
        droppedChars++;
    }
    
    // Update cached values if valid
    if (gps.location.isValid()) {
        valid = true;
        seeded = false;          // Neo-6M fix is authoritative — stop using seed
        neoHasFixedOnce = true;
        latitude = gps.location.lat();   // TinyGPS++ returns double
        longitude = gps.location.lng();  // TinyGPS++ returns double
        altitude = gps.altitude.meters();
        speed = gps.speed.mps();  // meters per second
        satellites = gps.satellites.value();
    } else if (!seeded) {
        // No Neo-6M fix AND no seed — truly invalid
        valid = false;
    }
    // If seeded==true and Neo has no fix, 'valid' stays true from seedPosition()
}

bool GPSHandler::isValid() {
    return valid;  // true from Neo-6M fix OR from seed
}

bool GPSHandler::isSeeded() const {
    return seeded;
}

void GPSHandler::seedPosition(double lat, double lon) {
    // Only use seed if Neo-6M hasn't fixed yet.
    // Once Neo-6M locks, its data is always preferred.
    if (neoHasFixedOnce) return;
    latitude = lat;
    longitude = lon;
    if (!seeded) {
        // Print only on first seed to avoid log spam (called every 2s)
        Serial.println(F("# GPS seeded from Pi/SIM7600E"));
    }
    seeded = true;
    valid = true;
}

double GPSHandler::getLatitude() {
    return latitude;
}

double GPSHandler::getLongitude() {
    return longitude;
}

float GPSHandler::getAltitude() {
    return altitude;
}

float GPSHandler::getSpeed() {
    return speed;
}

uint8_t GPSHandler::getSatellites() {
    return satellites;
}

uint8_t GPSHandler::getLastProcessedChars() const {
    return lastProcessedChars;
}

uint16_t GPSHandler::getDroppedChars() const {
    return droppedChars;
}

uint16_t GPSHandler::getMaxBacklog() const {
    return maxBacklog;
}

void GPSHandler::printInfo() {
    Serial.print(F("GPS: "));
    if (isValid()) {
        Serial.print(F("Lat="));
        Serial.print(latitude, 6);
        Serial.print(F(" Lon="));
        Serial.print(longitude, 6);
        Serial.print(F(" Sats="));
        Serial.println(satellites);
    } else {
        Serial.println(F("No fix"));
    }
}
