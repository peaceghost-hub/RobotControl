/*
 * Navigation Implementation
 */

#include "navigation.h"

Navigation::Navigation() {
    waypointCount = 0;
    currentWaypointIndex = 0;
    navigationActive = false;
    obstacleDetectedTime = 0;
    inAvoidanceMode = false;
}

void Navigation::begin(GPSHandler* g, CompassHandler* c, MotorControl* m, ObstacleAvoidance* o) {
    gps = g;
    compass = c;
    motors = m;
    obstacleAvoid = o;
}

void Navigation::addWaypoint(float lat, float lon, int id) {
    if (waypointCount < MAX_WAYPOINTS) {
        waypoints[waypointCount].latitude = lat;
        waypoints[waypointCount].longitude = lon;
        waypoints[waypointCount].id = id;
        waypoints[waypointCount].reached = false;
        waypointCount++;
    }
}

void Navigation::clearWaypoints() {
    waypointCount = 0;
    currentWaypointIndex = 0;
}

int Navigation::getWaypointCount() {
    return waypointCount;
}

int Navigation::getCurrentWaypointIndex() {
    return currentWaypointIndex;
}

void Navigation::start() {
    navigationActive = true;
    currentWaypointIndex = 0;
}

void Navigation::stop() {
    navigationActive = false;
    motors->stop();
}

void Navigation::resume() {
    if (waypointCount > 0 && currentWaypointIndex < waypointCount) {
        navigationActive = true;
    }
}

void Navigation::pause() {
    navigationActive = false;
}

void Navigation::update() {
    if (!navigationActive || !gps->isValid()) {
        return;
    }
    
    // Check for obstacles
    if (obstacleAvoid->isObstacleDetected()) {
        handleObstacleAvoidance();
        return;
    }
    
    // If we were in avoidance mode, resume normal navigation
    if (inAvoidanceMode) {
        inAvoidanceMode = false;
        Serial.println(F("# Obstacle cleared, resuming navigation"));
    }
    
    // Navigate to current waypoint
    navigateToWaypoint();
}

bool Navigation::isComplete() {
    return (currentWaypointIndex >= waypointCount);
}

void Navigation::navigateToWaypoint() {
    if (currentWaypointIndex >= waypointCount) {
        return;
    }
    
    Waypoint& current = waypoints[currentWaypointIndex];
    
    // Get current position
    float currentLat = gps->getLatitude();
    float currentLon = gps->getLongitude();
    
    // Calculate distance to waypoint
    float distance = calculateDistance(currentLat, currentLon, 
                                      current.latitude, current.longitude);
    
    // Check if waypoint reached
    if (distance < WAYPOINT_RADIUS) {
        current.reached = true;
        motors->stop();
        
        Serial.print(F("# Waypoint "));
        Serial.print(currentWaypointIndex + 1);
        Serial.println(F(" reached!"));
        
        currentWaypointIndex++;
        
        if (currentWaypointIndex >= waypointCount) {
            navigationActive = false;
        }
        
        return;
    }
    
    // Calculate bearing to waypoint
    float targetBearing = calculateBearing(currentLat, currentLon,
                                          current.latitude, current.longitude);
    
    // Get current heading
    float currentHeading = compass->getHeading();
    
    // Adjust motors to follow bearing
    motors->adjustForHeading(currentHeading, targetBearing);
    
    // Debug output
    if (millis() % 5000 < 100) {  // Print every 5 seconds
        Serial.print(F("# Dist: "));
        Serial.print(distance, 1);
        Serial.print(F("m, Bearing: "));
        Serial.print(targetBearing, 1);
        Serial.print(F("°, Heading: "));
        Serial.print(currentHeading, 1);
        Serial.println(F("°"));
    }
}

void Navigation::handleObstacleAvoidance() {
    if (!inAvoidanceMode) {
        inAvoidanceMode = true;
        obstacleDetectedTime = millis();
        Serial.print(F("# Obstacle detected at "));
        Serial.print(obstacleAvoid->getDistance());
        Serial.println(F("cm - scanning for clear path..."));
    }
    
    // Stop immediately
    motors->stop();
    delay(300);
    
    // Scan for clear path
    Serial.println(F("# Scanning path..."));
    PathScan scan = obstacleAvoid->scanPath();
    
    Serial.print(F("# Scan: L="));
    Serial.print(scan.leftDist);
    Serial.print(F("cm C="));
    Serial.print(scan.centerDist);
    Serial.print(F("cm R="));
    Serial.print(scan.rightDist);
    Serial.println(F("cm"));
    
    // Choose best path
    if (scan.leftClear && scan.leftDist > scan.rightDist) {
        // Turn left
        Serial.println(F("# Turning left to avoid obstacle"));
        motors->turnLeft(150);
        delay(800);
        motors->forward(150);
        delay(1000);
    } else if (scan.rightClear) {
        // Turn right
        Serial.println(F("# Turning right to avoid obstacle"));
        motors->turnRight(150);
        delay(800);
        motors->forward(150);
        delay(1000);
    } else {
        // No clear path - rotate more aggressively
        Serial.println(F("# No clear path, rotating..."));
        motors->turnLeft(180);
        delay(1500);
        // Scan again
        PathScan rescan = obstacleAvoid->scanPath();
        if (rescan.leftClear || rescan.rightClear) {
            motors->forward(150);
            delay(800);
        } else {
            // Still blocked, try opposite direction
            motors->turnRight(180);
            delay(2000);
        }
    }
    
    motors->stop();
}

float Navigation::calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    // Haversine formula
    const float R = 6371000.0; // Earth radius in meters
    
    float dLat = (lat2 - lat1) * DEG_TO_RAD;
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
              sin(dLon/2) * sin(dLon/2);
    
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c;
}

float Navigation::calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    // Calculate bearing between two points
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    float y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
    float x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
              sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);
    
    float bearing = atan2(y, x) * RAD_TO_DEG;
    
    // Normalize to 0-360
    bearing = fmod((bearing + 360.0), 360.0);
    
    return bearing;
}
