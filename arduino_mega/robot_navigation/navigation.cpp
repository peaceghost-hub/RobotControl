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
    useCompass = true;
    lastPositionSave = 0;
    waypointJustCompleted = false;
    lastCompletedWaypointId = -1;
    savedTargetBearing = 0.0;
    avoidStep = AV_IDLE;
    avoidStepTime = 0;
    avoidTurnDirection = 0;
    avoidTurnAngle = 0;
    gpsNudgeActive = false;
    gpsNudgeEndTime = 0;
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

    // Navigate to current waypoint
    if (useCompass) {
        navigateToWaypoint();
    } else {
        navigateToWaypointGpsOnly();
    }
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
        
        // Set completion flag for status reporting
        waypointJustCompleted = true;
        lastCompletedWaypointId = current.id;
        
        // Send waypoint completion to Pi
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(CMD_WAYPOINT_COMPLETED);
        Wire.write((uint8_t)(current.id >> 8));   // High byte of waypoint ID
        Wire.write((uint8_t)(current.id & 0xFF)); // Low byte of waypoint ID
        Wire.endTransmission();
        
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
            float lat = gps->getLatitude();
            float lon = gps->getLongitude();
            Waypoint& target = waypoints[currentWaypointIndex];
            savedTargetBearing = calculateBearing(lat, lon, target.latitude, target.longitude);
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
            // Kick off non-blocking scan
            Serial.println(F("# Scanning for obstacle-free path..."));
            obstacleAvoid->startScan();
            avoidStep = AV_SCAN_WAIT;
            avoidStepTime = now;
        }
        break;

    // ---- 2. Wait for the non-blocking scan to finish ----
    case AV_SCAN_WAIT: {
        obstacleAvoid->update();   // drive the scan state machine
        if (!obstacleAvoid->isScanComplete()) break;

        PathScan scan = obstacleAvoid->getScanResult();
        float currentHeading = compass->getHeading();
        avoidTurnDirection = 0;
        avoidTurnAngle = 0;

        // Prefer right if clear
        if (scan.rightClear && scan.rightDist > OBSTACLE_SAFE_DISTANCE) {
            avoidTurnDirection = 1;
            avoidTurnAngle = 90;
            Serial.println(F("# Clear path found: RIGHT 90°"));
        }
        // Then left
        else if (scan.leftClear && scan.leftDist > OBSTACLE_SAFE_DISTANCE) {
            avoidTurnDirection = -1;
            avoidTurnAngle = 90;
            Serial.println(F("# Clear path found: LEFT 90°"));
        }
        // Both blocked → try sharp right 120°
        else {
            Serial.println(F("# Both sides blocked, attempting sharp RIGHT 120°"));
            avoidTurnDirection = 1;
            avoidTurnAngle = 120;
            motors->startTurnDegrees(120, 140);
            avoidStep = AV_SHARP_RIGHT_TURN;
            avoidStepTime = now;
            break;
        }

        // Execute the chosen 90° turn (non-blocking)
        int deg = avoidTurnAngle * avoidTurnDirection;
        motors->startTurnDegrees(deg, 140);
        avoidStep = AV_TURN;
        avoidStepTime = now;
        break;
    }

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
            Serial.println(F("# Still blocked after avoidance — stopping"));
            motors->stop();
            // Stay in avoidance; will retry on next update() when sensor clears
            avoidStep = AV_IDLE;
            // inAvoidanceMode stays true so update() re-enters if obstacle persists
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

// ================= Additional features / stubs =================

void Navigation::setUseCompass(bool enabled) {
    useCompass = enabled;
}

bool Navigation::getUseCompass() const {
    return useCompass;
}

void Navigation::fallbackToGpsOnly() {
    useCompass = false;
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

void Navigation::updateGpsData(float lat, float lon, float speed, float heading) {
    // Minimal stub: Record position into history for RETURN and trust compass heading
    // A fuller implementation would feed data into GPSHandler.
    (void)speed; (void)heading;
    HistoryPoint p{lat, lon, millis()};
    if (historyCount < MAX_HISTORY_POINTS) {
        history[historyCount++] = p;
    } else {
        history[currentHistoryIndex] = p;
        currentHistoryIndex = (currentHistoryIndex + 1) % MAX_HISTORY_POINTS;
    }
}

void Navigation::navigateToWaypointGpsOnly() {
    // Fallback: use target bearing as surrogate heading correction
    if (currentWaypointIndex >= waypointCount) return;

    // If a non-blocking nudge is running, wait for it to finish
    if (gpsNudgeActive) {
        if (millis() >= gpsNudgeEndTime) {
            motors->stop();
            gpsNudgeActive = false;
        }
        return;  // Either still nudging or just stopped — re-evaluate next call
    }

    Waypoint& current = waypoints[currentWaypointIndex];
    float lat = gps->getLatitude();
    float lon = gps->getLongitude();
    float distance = calculateDistance(lat, lon, current.latitude, current.longitude);
    if (distance < WAYPOINT_RADIUS) {
        current.reached = true;
        motors->stop();
        currentWaypointIndex++;
        if (currentWaypointIndex >= waypointCount) navigationActive = false;
        return;
    }
    float targetBearing = calculateBearing(lat, lon, current.latitude, current.longitude);
    // Without compass, approximate by short non-blocking nudges and re-evaluation
    motors->forward(150);
    gpsNudgeActive = true;
    gpsNudgeEndTime = millis() + 250;  // 250 ms nudge
}

void Navigation::handleReturnNavigation() {
    if (historyCount == 0) {
        returningToStart = false;
        return;
    }
    // Choose the oldest point as the start (index 0 if not wrapped)
    int idx = (historyCount < MAX_HISTORY_POINTS) ? 0 : currentHistoryIndex;
    HistoryPoint target = history[idx];

    float lat = gps->getLatitude();
    float lon = gps->getLongitude();
    float distance = calculateDistance(lat, lon, target.latitude, target.longitude);

    if (distance < WAYPOINT_RADIUS) {
        returningToStart = false;
        motors->stop();
        Serial.println(F("# Returned to start"));
        return;
    }

    float bearing = calculateBearing(lat, lon, target.latitude, target.longitude);
    float heading = compass->getHeading();
    motors->adjustForHeading(heading, bearing);
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
