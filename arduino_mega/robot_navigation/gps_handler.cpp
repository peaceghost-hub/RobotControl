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
}

bool GPSHandler::begin(HardwareSerial& serial) {
    gpsSerial = &serial;
    return true;
}

void GPSHandler::update() {
    // Read GPS data
    while (gpsSerial->available() > 0) {
        char c = gpsSerial->read();
        gps.encode(c);
    }
    
    // Update cached values if valid
    if (gps.location.isValid()) {
        valid = true;
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitude = gps.altitude.meters();
        speed = gps.speed.mps();  // meters per second
        satellites = gps.satellites.value();
    } else {
        valid = false;
    }
}

bool GPSHandler::isValid() {
    return valid && gps.location.isValid();
}

float GPSHandler::getLatitude() {
    return latitude;
}

float GPSHandler::getLongitude() {
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
