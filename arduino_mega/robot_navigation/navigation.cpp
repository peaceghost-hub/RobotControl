/*
 * Navigation Implementation — v2
 *
 * 3-Phase steering with PD controller, simplified obstacle avoidance,
 * heading freshness watchdog, stall detection, consecutive-arrival,
 * adaptive speed, GPS speed gate.
 *
 * GOLDEN RULE: Every handler < 100 µs.  Zero delay().  Zero blocking.
 */

#include "navigation.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI/180.0)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0/PI)
#endif

// ===================== Constructor =====================

Navigation::Navigation() {
    waypointCount = 0;
    currentWaypointIndex = 0;
    navigationActive = false;
    navState = NAV_IDLE;
    stateEntryTime = 0;
    currentHeading = 0.0f;
    lastHeadingUpdateMs = 0;
    lastHeadingError = 0.0f;
    consecutiveArrivals = 0;
    avoidAttempts = 0;
    avoidTurnDir = 1;
    stallCheckLat = 0.0;
    stallCheckLon = 0.0;
    stallCheckTime = 0;
    stallRecoveries = 0;
    lastGoodBearing = 0.0f;
    lastBearingValid = false;
    navStartTime = 0;
    historyCount = 0;
    currentHistoryIndex = 0;
    returningToStart = false;
    waypointJustCompleted = false;
    lastCompletedWaypointId = -1;
    lastPositionSave = 0;
}

// ===================== Setup =====================

void Navigation::begin(GPSHandler* g, MotorControl* m, ObstacleAvoidance* o) {
    gps = g;
    motors = m;
    obstacleAvoid = o;
}

// ===================== Heading from Pi =====================

void Navigation::setHeading(float h) {
    currentHeading = h;
    lastHeadingUpdateMs = millis();   // freshness timestamp
}

// ===================== Waypoint Management =====================

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

// ===================== Navigation Control =====================

void Navigation::start() {
    navigationActive = true;
    currentWaypointIndex = 0;
    navStartTime = millis();
    resetWaypointCounters();
    lastHeadingError = 0.0f;
    lastBearingValid = false;
    enterState(NAV_DRIVE_TO_TARGET);
    Serial.println(F("# NAV: started"));
}

void Navigation::stop() {
    navigationActive = false;
    enterState(NAV_IDLE);
    motors->stop();
    Serial.println(F("# NAV: stopped"));
}

void Navigation::resume() {
    if (waypointCount > 0 && currentWaypointIndex < waypointCount) {
        navigationActive = true;
        enterState(NAV_DRIVE_TO_TARGET);
    }
}

void Navigation::pause() {
    navigationActive = false;
    enterState(NAV_IDLE);
}

bool Navigation::isComplete() {
    return (navState == NAV_COMPLETE) || (currentWaypointIndex >= waypointCount);
}

Navigation::NavState Navigation::getNavState() const {
    return navState;
}

// ===================== State machine helpers =====================

void Navigation::enterState(NavState newState) {
    navState = newState;
    stateEntryTime = millis();
}

void Navigation::resetWaypointCounters() {
    consecutiveArrivals = 0;
    avoidAttempts = 0;
    stallRecoveries = 0;
    stallCheckTime = millis();
    if (gps && gps->isValid()) {
        stallCheckLat = gps->getLatitude();
        stallCheckLon = gps->getLongitude();
    }
}

float Navigation::normalizeHeadingError(float error) {
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

int Navigation::speedForDistance(double distance) {
    if (distance > SLOWDOWN_RADIUS_FAR)  return motors->getAutoBaseSpeed();  // default 120
    if (distance > SLOWDOWN_RADIUS_NEAR) return 100;
    return 80;
}

// ===================== Main Update =====================

void Navigation::update() {
    // ---- Gate: must be active with valid GPS ----
    if (!navigationActive || !gps->isValid()) {
        return;
    }

    const unsigned long now = millis();

    // ---- SAFETY: Heading freshness watchdog ----
    // If Pi hasn't sent a heading in 500 ms, something is wrong.
    // Driving with stale heading = driving blind → stop immediately.
    if (lastHeadingUpdateMs > 0 && (now - lastHeadingUpdateMs > HEADING_STALE_LIMIT)) {
        // Only stop + warn if we were actively driving
        if (navState == NAV_TURN_TO_TARGET || navState == NAV_DRIVE_TO_TARGET) {
            motors->stop();
            static unsigned long lastStaleWarn = 0;
            if (now - lastStaleWarn > 3000) {
                Serial.println(F("# NAV: heading STALE — motors stopped"));
                lastStaleWarn = now;
            }
        }
        return;  // Skip this cycle — heading is not fresh
    }

    // ---- SAFETY: Navigation timeout (10 min) ----
    if (now - navStartTime > NAV_TIMEOUT) {
        Serial.println(F("# NAV: TIMEOUT — stopping"));
        stop();
        navigationActive = false;
        return;
    }

    // ---- SAFETY: Stall detection ----
    // Only check during active forward motion states
    if (navState == NAV_TURN_TO_TARGET || navState == NAV_DRIVE_TO_TARGET) {
        if (now - stallCheckTime > STALL_CHECK_INTERVAL) {
            double curLat = gps->getLatitude();
            double curLon = gps->getLongitude();
            double moved = calculateDistance(stallCheckLat, stallCheckLon, curLat, curLon);

            // Update checkpoint for next interval
            stallCheckLat = curLat;
            stallCheckLon = curLon;
            stallCheckTime = now;

            if (moved < STALL_DISTANCE) {
                stallRecoveries++;
                Serial.print(F("# NAV: STALL detected (moved "));
                Serial.print((float)moved, 1);
                Serial.print(F("m) — recovery #"));
                Serial.println(stallRecoveries);

                if (stallRecoveries > MAX_STALL_RECOVERIES) {
                    Serial.println(F("# NAV: too many stalls — skipping waypoint"));
                    advanceWaypoint();
                    return;
                }

                // Enter stall recovery: reverse briefly
                motors->stop();
                enterState(NAV_STALLED);
                return;
            }
        }
    }

    // ---- Periodically save position for RETURN feature ----
    if (now - lastPositionSave >= 5000) {
        saveCurrentPosition();
        lastPositionSave = now;
    }

    // ---- Handle RETURN mode ----
    if (returningToStart) {
        handleReturnNavigation();
        return;
    }

    // ---- OBSTACLE CHECK — highest priority (except during avoidance) ----
    if (navState != NAV_AVOID_STOP && navState != NAV_AVOID_TURN &&
        navState != NAV_AVOID_DRIVE && navState != NAV_AVOID_RECHECK &&
        navState != NAV_STALLED) {
        if (obstacleAvoid->isObstacleDetected()) {
            motors->stop();
            avoidAttempts++;
            Serial.print(F("# NAV: obstacle! attempt #"));
            Serial.println(avoidAttempts);

            if (avoidAttempts > MAX_AVOID_ATTEMPTS) {
                Serial.println(F("# NAV: too many obstacles — skipping waypoint"));
                advanceWaypoint();
                return;
            }
            // Alternate turn direction each attempt
            avoidTurnDir = (avoidAttempts % 2 == 1) ? 1 : -1;
            enterState(NAV_AVOID_STOP);
            return;
        }
    }

    // ---- Run current state handler ----
    switch (navState) {
        case NAV_TURN_TO_TARGET:    handleTurnToTarget();    break;
        case NAV_DRIVE_TO_TARGET:   handleDriveToTarget();   break;
        case NAV_WAYPOINT_REACHED:  handleWaypointReached(); break;
        case NAV_AVOID_STOP:        handleAvoidStop();       break;
        case NAV_AVOID_TURN:        handleAvoidTurn();       break;
        case NAV_AVOID_DRIVE:       handleAvoidDrive();      break;
        case NAV_AVOID_RECHECK:     handleAvoidRecheck();    break;
        case NAV_STALLED:           handleStalled();         break;

        case NAV_COMPLETE:
            // All waypoints done — stop motors, set flag
            motors->stop();
            navigationActive = false;
            break;

        case NAV_IDLE:
        default:
            break;
    }
}

// ================================================================
//  PHASE 1 (legacy): Turn-to-target is now handled INSIDE
//  handleDriveToTarget via graduated arc steering.
//  The state NAV_TURN_TO_TARGET is kept for API compat but
//  immediately falls through to NAV_DRIVE_TO_TARGET.
// ================================================================

void Navigation::handleTurnToTarget() {
    // Legacy state — redirect to unified drive handler.
    // This prevents any rotation-in-place behavior.
    enterState(NAV_DRIVE_TO_TARGET);
}

// ================================================================
//  PHASE 2+3: PD proportional steering (+ deadband)
// ================================================================

void Navigation::handleDriveToTarget() {
    if (currentWaypointIndex >= waypointCount) {
        enterState(NAV_COMPLETE);
        return;
    }

    Waypoint& wp = waypoints[currentWaypointIndex];
    double curLat = gps->getLatitude();
    double curLon = gps->getLongitude();

    // --- Distance ---
    double distance = calculateDistance(curLat, curLon, wp.latitude, wp.longitude);

    // --- Waypoint reached check (consecutive confirmation) ---
    if (distance < WAYPOINT_RADIUS) {
        consecutiveArrivals++;
        if (consecutiveArrivals >= CONSECUTIVE_ARRIVALS) {
            motors->stop();
            enterState(NAV_WAYPOINT_REACHED);
            return;
        }
    } else {
        consecutiveArrivals = 0;
    }

    // --- Bearing to waypoint (with speed gate) ---
    float targetBearing;
    float gpsSpeed = gps->getSpeed();

    if (gpsSpeed < SPEED_GATE_THRESHOLD && lastBearingValid) {
        // Nearly stationary — hold last known good bearing to prevent jitter
        targetBearing = lastGoodBearing;
    } else {
        targetBearing = (float)calculateBearing(curLat, curLon, wp.latitude, wp.longitude);
        lastGoodBearing = targetBearing;
        lastBearingValid = true;
    }

    // --- Heading error ---
    float error = normalizeHeadingError(targetBearing - currentHeading);
    float absError = fabsf(error);

    // --- Adaptive base speed ---
    int baseSpeed = speedForDistance(distance);

    // --- Graduated Steering (no rotation-in-place, always forward progress) ---
    if (absError < HEADING_DEADBAND) {
        // PHASE 3: Deadband — drive straight
        motors->setMotors(baseSpeed, baseSpeed);
    } else if (absError >= TURN_IN_PLACE_THRESH) {
        // LARGE ERROR (>25°): Wide arc — outer motor fast, inner motor SLOW
        // Robot arcs toward target while ALWAYS making forward progress.
        // Inner motor runs at ~30% of outer to prevent pivot-in-place.
        int outerSpeed = constrain(baseSpeed, MIN_TURN_SPEED, MAX_TURN_SPEED);
        int innerSpeed = outerSpeed * 30 / 100;   // 30% — enough to crawl forward
        if (innerSpeed < 50) innerSpeed = 50;      // floor so wheel actually turns
        if (error > 0) {
            // Target RIGHT → left motor fast, right motor slow
            motors->setMotors(outerSpeed, innerSpeed);
        } else {
            // Target LEFT → right motor fast, left motor slow
            motors->setMotors(innerSpeed, outerSpeed);
        }
    } else {
        // PHASE 2 (5°–25°): PD proportional steering
        float dError = error - lastHeadingError;
        float correction = (STEERING_KP * error) + (STEERING_KD * dError);
        int corr = constrain((int)correction, -STEERING_MAX_CORRECTION, STEERING_MAX_CORRECTION);

        int leftSpeed  = baseSpeed + corr;
        int rightSpeed = baseSpeed - corr;

        motors->setMotors(leftSpeed, rightSpeed);
    }

    lastHeadingError = error;

    // Debug (rate-limited)
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg >= 5000) {
        lastDbg = millis();
        Serial.print(F("# DRIVE: dist="));
        Serial.print((float)distance, 1);
        Serial.print(F("m brg="));
        Serial.print(targetBearing, 1);
        Serial.print(F(" hdg="));
        Serial.print(currentHeading, 1);
        Serial.print(F(" err="));
        Serial.print(error, 1);
        Serial.print(F(" spd="));
        Serial.println(baseSpeed);
    }
}

// ================================================================
//  Waypoint Reached — advance to next
// ================================================================

void Navigation::handleWaypointReached() {
    // Brief stop already happened when we entered this state
    advanceWaypoint();
}

void Navigation::advanceWaypoint() {
    if (currentWaypointIndex < waypointCount) {
        waypoints[currentWaypointIndex].reached = true;

        Serial.print(F("# NAV: Waypoint "));
        Serial.print(currentWaypointIndex + 1);
        Serial.print(F("/"));
        Serial.print(waypointCount);
        Serial.println(F(" reached!"));

        waypointJustCompleted = true;
        lastCompletedWaypointId = waypoints[currentWaypointIndex].id;

        currentWaypointIndex++;
    }

    if (currentWaypointIndex >= waypointCount) {
        Serial.println(F("# NAV: ALL waypoints complete!"));
        enterState(NAV_COMPLETE);
        navigationActive = false;
    } else {
        // Reset per-waypoint counters for the next waypoint
        resetWaypointCounters();
        lastHeadingError = 0.0f;
        lastBearingValid = false;
        enterState(NAV_DRIVE_TO_TARGET);
    }
}

// ================================================================
//  Obstacle Avoidance — 4 states, alternating direction
//  Each handler runs < 100 µs.  NEVER blocks.
// ================================================================

void Navigation::handleAvoidStop() {
    // Wait for settle period
    if (millis() - stateEntryTime >= AVOID_STOP_DURATION) {
        // Start turning away — direction alternates per attempt
        int turnDeg = avoidTurnDir * 80;  // ~80° turn
        motors->startTurnDegrees(turnDeg, AVOID_TURN_SPEED);

        Serial.print(F("# AVOID: turning "));
        Serial.print(turnDeg > 0 ? F("RIGHT ") : F("LEFT "));
        Serial.print(abs(turnDeg));
        Serial.println(F("°"));

        enterState(NAV_AVOID_TURN);
    }
}

void Navigation::handleAvoidTurn() {
    if (motors->isTurnComplete()) {
        // Drive past the obstacle
        Serial.println(F("# AVOID: driving past"));
        motors->startTimedForward(AVOID_DRIVE_SPEED, AVOID_DRIVE_DURATION);
        enterState(NAV_AVOID_DRIVE);
    }
}

void Navigation::handleAvoidDrive() {
    if (motors->isTimedForwardComplete()) {
        // Stop and prepare to recheck
        motors->stop();
        enterState(NAV_AVOID_RECHECK);
    }
}

void Navigation::handleAvoidRecheck() {
    // Settle time before recheck
    if (millis() - stateEntryTime < AVOID_RECHECK_SETTLE) return;

    // Force a fresh ultrasonic reading
    obstacleAvoid->update();
    int dist = obstacleAvoid->getDistance();

    if (dist > 0 && dist < OBSTACLE_TRIGGER_CM) {
        // Still blocked
        Serial.println(F("# AVOID: still blocked after manoeuvre"));
        if (avoidAttempts >= MAX_AVOID_ATTEMPTS) {
            Serial.println(F("# AVOID: max attempts — skipping waypoint"));
            advanceWaypoint();
        } else {
            // Try again with opposite direction
            avoidAttempts++;
            avoidTurnDir = -avoidTurnDir;
            motors->stop();
            enterState(NAV_AVOID_STOP);
        }
    } else {
        // Clear — resume navigation (recalculates bearing automatically)
        Serial.println(F("# AVOID: clear — resuming navigation"));
        enterState(NAV_DRIVE_TO_TARGET);
    }
}

// ================================================================
//  Stall Recovery — reverse briefly, then retry
// ================================================================

void Navigation::handleStalled() {
    unsigned long elapsed = millis() - stateEntryTime;

    if (elapsed < STALL_REVERSE_DURATION) {
        // Reverse at moderate speed
        motors->setMotors(-100, -100);
    } else {
        // Done reversing — resume navigation
        motors->stop();
        Serial.println(F("# STALL: recovery done, resuming"));
        stallCheckTime = millis();
        stallCheckLat = gps->getLatitude();
        stallCheckLon = gps->getLongitude();
        enterState(NAV_DRIVE_TO_TARGET);
    }
}

// ================================================================
//  Return-to-Start Navigation
// ================================================================

void Navigation::handleReturnNavigation() {
    if (historyCount == 0) {
        returningToStart = false;
        motors->stop();
        return;
    }

    // Choose the oldest point as the start
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
    float error = normalizeHeadingError((float)bearing - currentHeading);

    // Use the same 3-phase approach for return navigation
    int baseSpeed = speedForDistance(distance);

    if (fabsf(error) > TURN_IN_PLACE_THRESH) {
        // Turn in place first
        int turnSpeed = map(constrain((int)fabsf(error), 25, 180),
                            25, 180, MIN_TURN_SPEED, MAX_TURN_SPEED);
        if (error > 0) {
            motors->setMotors(turnSpeed, -turnSpeed);
        } else {
            motors->setMotors(-turnSpeed, turnSpeed);
        }
    } else if (fabsf(error) < HEADING_DEADBAND) {
        motors->setMotors(baseSpeed, baseSpeed);
    } else {
        float correction = STEERING_KP * error;
        int corr = constrain((int)correction, -STEERING_MAX_CORRECTION, STEERING_MAX_CORRECTION);
        motors->setMotors(baseSpeed + corr, baseSpeed - corr);
    }
}

// ================================================================
//  Haversine Distance (meters)
// ================================================================

double Navigation::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;

    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;

    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(dLon / 2.0) * sin(dLon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}

// ================================================================
//  Bearing (degrees 0-360, true north)
// ================================================================

double Navigation::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * DEG_TO_RAD;

    double y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
    double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
               sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);

    double bearing = atan2(y, x) * RAD_TO_DEG;
    bearing = fmod((bearing + 360.0), 360.0);

    return bearing;
}

// ================================================================
//  Position History
// ================================================================

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

// ================================================================
//  GPS data from Pi (SIM7600E seed — also saves to history)
// ================================================================

void Navigation::updateGpsData(double lat, double lon, float speed, float heading) {
    currentHeading = heading;
    lastHeadingUpdateMs = millis();
    (void)speed;
    HistoryPoint p{lat, lon, millis()};
    if (historyCount < MAX_HISTORY_POINTS) {
        history[historyCount++] = p;
    } else {
        history[currentHistoryIndex] = p;
        currentHistoryIndex = (currentHistoryIndex + 1) % MAX_HISTORY_POINTS;
    }
}

// ================================================================
//  Waypoint Completion Tracking
// ================================================================

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
