/*
 * Navigation Implementation
 */

#include "navigation.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI/180.0)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0/PI)
#endif

Navigation::Navigation() {
    waypointCount = 0;
    currentWaypointIndex = 0;
    navigationActive = false;
    obstacleDetectedTime = 0;
    inAvoidanceMode = false;
    historyCount = 0;
    currentHistoryIndex = 0;
    returningToStart = false;
    currentHeading = 0.0;
    lastPositionSave = 0;
    waypointJustCompleted = false;
    lastCompletedWaypointId = -1;
    savedTargetBearing = 0.0;
    avoidStep = AV_IDLE;
    avoidStepTime = 0;
    avoidTurnDirection = 0;
    avoidTurnAngle = 0;
}

void Navigation::begin(GPSHandler* g, MotorControl* m, ObstacleAvoidance* o) {
    gps = g;
    motors = m;
    obstacleAvoid = o;
}

void Navigation::addWaypoint(double lat, double lon, int id) {
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
    inAvoidanceMode = false;
    avoidStep = AV_IDLE;          // Cancel any in-progress obstacle avoidance
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

    // If obstacle avoidance state machine is active, run it instead of navigation
    if (avoidStep != AV_IDLE) {
        handleObstacleAvoidance();
        return;
    }
    
    // CRITICAL: Check for obstacles FIRST - highest priority!
    // Stop immediately if obstacle detected during autonomous navigation
    if (obstacleAvoid->isObstacleDetected()) {
        motors->stop();  // Immediate stop before handling avoidance
        handleObstacleAvoidance();  // Enters AV_STOP_PAUSE on first call
        return;
    }
    
    // Periodically save position for RETURN feature
    if (millis() - lastPositionSave >= 5000) {
        saveCurrentPosition();
        lastPositionSave = millis();
    }
    
    // If we were in avoidance mode, resume normal navigation
    if (inAvoidanceMode) {
        inAvoidanceMode = false;
        Serial.println(F("# Obstacle cleared, resuming navigation"));
    }
    
    // Handle RETURN mode if active
    if (returningToStart) {
        handleReturnNavigation();
        return;
    }

    // Navigate to current waypoint (heading from Pi)
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
    double currentLat = gps->getLatitude();
    double currentLon = gps->getLongitude();
    
    // Calculate distance to waypoint
    double distance = calculateDistance(currentLat, currentLon, 
                                      current.latitude, current.longitude);
    
    // Check if waypoint reached
    if (distance < WAYPOINT_RADIUS) {
        current.reached = true;
        motors->stop();
        
        Serial.print(F("# Waypoint "));
        Serial.print(currentWaypointIndex + 1);
        Serial.println(F(" reached!"));
        
        // Set completion flag for status reporting
        // Pi reads this via CMD_REQUEST_STATUS → prepareStatusResponse()
        waypointJustCompleted = true;
        lastCompletedWaypointId = current.id;
        
        currentWaypointIndex++;
        
        if (currentWaypointIndex >= waypointCount) {
            navigationActive = false;
        }
        
        return;
    }
    
    // Calculate bearing to waypoint
    double targetBearing = calculateBearing(currentLat, currentLon,
                                          current.latitude, current.longitude);
    
    // Get current heading (from Pi compass via setHeading)
    float heading = currentHeading;
    
    // Adjust motors to follow bearing
    motors->adjustForHeading(heading, (float)targetBearing);
    
    // Debug output (reliable timer instead of millis() modulo trick)
    static unsigned long lastNavDebug = 0;
    if (millis() - lastNavDebug >= 5000) {
        lastNavDebug = millis();
        Serial.print(F("# Dist: "));
        Serial.print((float)distance, 1);
        Serial.print(F("m, Bearing: "));
        Serial.print((float)targetBearing, 1);
        Serial.print(F("°, Heading: "));
        Serial.print(heading, 1);
        Serial.println(F("°"));
    }
}

void Navigation::handleObstacleAvoidance() {
    // ================================================================
    // NON-BLOCKING obstacle avoidance state machine.
    // Each call runs < 1 ms — never blocks the main loop / I2C / heartbeat.
    // ================================================================
    const unsigned long now = millis();

    // ---------- First entry: initialise ----------
    if (avoidStep == AV_IDLE) {
        inAvoidanceMode = true;
        obstacleDetectedTime = now;

        // Save current target bearing so we can resume toward waypoint later
        if (currentWaypointIndex < waypointCount) {
            double lat = gps->getLatitude();
            double lon = gps->getLongitude();
            Waypoint& target = waypoints[currentWaypointIndex];
            savedTargetBearing = (float)calculateBearing(lat, lon, target.latitude, target.longitude);
        }

        Serial.print(F("# Obstacle detected at "));
        Serial.print(obstacleAvoid->getDistance());
        Serial.println(F("cm — non-blocking avoidance started"));

        motors->stop();
        avoidStep = AV_STOP_PAUSE;
        avoidStepTime = now;
        return;
    }

    switch (avoidStep) {

    // ---- 1. Brief 500 ms pause for sensor stability ----
    case AV_STOP_PAUSE:
        if (now - avoidStepTime >= 500) {
            // No servo scanner — turn robot body right 90° to find clear path.
            // The fixed-mount ultrasonic will face the new direction after the turn.
            Serial.println(F("# Turning right 90° to find clear path"));
            avoidTurnDirection = 1;
            avoidTurnAngle = 90;
            motors->startTurnDegrees(90, 140);
            avoidStep = AV_TURN;
            avoidStepTime = now;
        }
        break;

    // ---- 3a. Waiting for the primary 90° turn to finish ----
    case AV_TURN:
        if (motors->isTurnComplete()) {
            // Move forward cautiously for 1.5 s to clear obstacle zone
            Serial.println(F("# Moving forward to clear obstacle zone"));
            motors->startTimedForward(100, 1500);
            avoidStep = AV_FORWARD;
            avoidStepTime = now;
        }
        break;

    // ---- 4. Moving forward after turn ----
    case AV_FORWARD:
        if (motors->isTimedForwardComplete()) {
            // Pause 300 ms then recheck
            motors->stop();
            avoidStep = AV_FORWARD_PAUSE;
            avoidStepTime = now;
        }
        break;

    // ---- 5. Brief pause before recheck ----
    case AV_FORWARD_PAUSE:
        if (now - avoidStepTime >= 300) {
            obstacleAvoid->update();
            avoidStep = AV_RECHECK;
            avoidStepTime = now;
        }
        break;

    // ---- 6. Recheck if still blocked ----
    case AV_RECHECK: {
        obstacleAvoid->update();
        int finalCheck = obstacleAvoid->getDistance();
        if (finalCheck > 0 && finalCheck < OBSTACLE_CRITICAL_DISTANCE) {
            // Still blocked — escalate to sharp right 120° turn
            Serial.println(F("# Still blocked — trying sharp right 120°"));
            motors->startTurnDegrees(120, 140);
            avoidStep = AV_SHARP_RIGHT_TURN;
            avoidStepTime = now;
        } else {
            avoidStep = AV_DONE;
        }
        break;
    }

    // ---- Sharp right 120° turn ----
    case AV_SHARP_RIGHT_TURN:
        if (motors->isTurnComplete()) {
            // Pause + recheck
            avoidStep = AV_SHARP_RIGHT_CHECK;
            avoidStepTime = now;
        }
        break;

    case AV_SHARP_RIGHT_CHECK: {
        if (now - avoidStepTime < 300) break;  // 300 ms pause
        obstacleAvoid->update();
        int checkDist = obstacleAvoid->getDistance();
        if (checkDist == -1 || checkDist > OBSTACLE_SAFE_DISTANCE) {
            Serial.println(F("# Sharp right turn successful"));
            // Move forward cautiously
            motors->startTimedForward(100, 1500);
            avoidStep = AV_FORWARD;
            avoidStepTime = now;
        } else {
            // Undo sharp right: turn back -120°
            Serial.println(F("# Sharp right failed, undoing..."));
            motors->startTurnDegrees(-120, 140);
            avoidStep = AV_UNDO_SHARP_RIGHT;
            avoidStepTime = now;
        }
        break;
    }

    case AV_UNDO_SHARP_RIGHT:
        if (motors->isTurnComplete()) {
            // Now try sharp left -120°
            Serial.println(F("# Attempting sharp LEFT 120°"));
            motors->startTurnDegrees(-120, 140);
            avoidStep = AV_SHARP_LEFT_TURN;
            avoidStepTime = now;
        }
        break;

    case AV_SHARP_LEFT_TURN:
        if (motors->isTurnComplete()) {
            avoidStep = AV_SHARP_LEFT_CHECK;
            avoidStepTime = now;
        }
        break;

    case AV_SHARP_LEFT_CHECK: {
        if (now - avoidStepTime < 300) break;
        obstacleAvoid->update();
        int checkDist = obstacleAvoid->getDistance();
        if (checkDist == -1 || checkDist > OBSTACLE_SAFE_DISTANCE) {
            Serial.println(F("# Sharp left turn successful"));
            motors->startTimedForward(100, 1500);
            avoidStep = AV_FORWARD;
            avoidStepTime = now;
        } else {
            Serial.println(F("# No clear path found — staying stopped"));
            motors->stop();
            avoidStep = AV_IDLE;
            // inAvoidanceMode stays true
        }
        break;
    }

    // ---- Done: obstacle cleared, resume navigation ----
    case AV_DONE:
        Serial.println(F("# Obstacle cleared, resuming navigation to waypoint"));
        inAvoidanceMode = false;
        avoidStep = AV_IDLE;
        break;

    default:
        avoidStep = AV_IDLE;
        break;
    }
}

double Navigation::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Haversine formula — double precision required for GPS coordinates
    const double R = 6371000.0; // Earth radius in meters
    
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    double a = sin(dLat/2.0) * sin(dLat/2.0) +
              cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
              sin(dLon/2.0) * sin(dLon/2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    
    return R * c;
}

double Navigation::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    // Calculate bearing between two points — double precision
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    double y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
    double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
              sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);
    
    double bearing = atan2(y, x) * RAD_TO_DEG;
    
    // Normalize to 0-360
    bearing = fmod((bearing + 360.0), 360.0);
    
    return bearing;
}

// ================= Heading from Pi =================

void Navigation::setHeading(float h) {
    currentHeading = h;
}

void Navigation::saveCurrentPosition() {
    if (!gps || !gps->isValid()) return;
    HistoryPoint p;
    p.latitude = gps->getLatitude();
    p.longitude = gps->getLongitude();
    p.timestamp = millis();
    if (historyCount < MAX_HISTORY_POINTS) {
        history[historyCount++] = p;
    } else {
        history[currentHistoryIndex] = p;
        currentHistoryIndex = (currentHistoryIndex + 1) % MAX_HISTORY_POINTS;
    }
}

void Navigation::returnToStart() {
    if (historyCount == 0) return;
    returningToStart = true;
}

bool Navigation::isReturning() const {
    return returningToStart;
}

void Navigation::clearHistory() {
    historyCount = 0;
    currentHistoryIndex = 0;
}

void Navigation::updateGpsData(double lat, double lon, float speed, float heading) {
    // Store heading from Pi for navigation
    currentHeading = heading;
    (void)speed;
    HistoryPoint p{lat, lon, millis()};
    if (historyCount < MAX_HISTORY_POINTS) {
        history[historyCount++] = p;
    } else {
        history[currentHistoryIndex] = p;
        currentHistoryIndex = (currentHistoryIndex + 1) % MAX_HISTORY_POINTS;
    }
}

void Navigation::handleReturnNavigation() {
    if (historyCount == 0) {
        returningToStart = false;
        return;
    }
    // Choose the oldest point as the start (index 0 if not wrapped)
    int idx = (historyCount < MAX_HISTORY_POINTS) ? 0 : currentHistoryIndex;
    HistoryPoint target = history[idx];

    double lat = gps->getLatitude();
    double lon = gps->getLongitude();
    double distance = calculateDistance(lat, lon, target.latitude, target.longitude);

    if (distance < WAYPOINT_RADIUS) {
        returningToStart = false;
        motors->stop();
        Serial.println(F("# Returned to start"));
        return;
    }

    double bearing = calculateBearing(lat, lon, target.latitude, target.longitude);
    float heading = currentHeading;  // From Pi compass
    motors->adjustForHeading(heading, (float)bearing);
}

// Waypoint completion tracking methods
bool Navigation::isWaypointJustCompleted() {
    return waypointJustCompleted;
}

int Navigation::getLastCompletedWaypointId() {
    return lastCompletedWaypointId;
}

void Navigation::clearWaypointCompletionFlag() {
    waypointJustCompleted = false;
    lastCompletedWaypointId = -1;
}
