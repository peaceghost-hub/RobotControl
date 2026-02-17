/*
 * Navigation Header
 * GPS Waypoint Navigation — 3-Phase Steering + PD Controller
 *
 * Architecture v2:
 *   - Single NavState state machine (10 states) replaces old bool + AvoidStep
 *   - Phase 1: Large error (>25°) — wide arc (outer fast, inner slow)
 *   - Phase 2: PD proportional steering (Kp=2, Kd=0.5)
 *   - Phase 3: Deadband < 5° — drive straight
 *   - Simplified 4-state obstacle avoidance (alternating L/R)
 *   - Heading freshness watchdog (500 ms)
 *   - Stall detection (30 s / 3 m)
 *   - Consecutive-arrival confirmation (3 readings)
 *   - Adaptive speed by distance
 *   - GPS speed gate for bearing stability
 *
 *   GOLDEN RULE: Every handler runs < 100 µs.  Zero delay().
 *                Zero blocking.  Cannot freeze the Mega main loop.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "globals.h"
#include "gps_handler.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"

// ===================== TUNING CONSTANTS =====================

// Waypoint geometry
#define WAYPOINT_RADIUS         3.0   // meters — "reached" threshold
#define SLOWDOWN_RADIUS_FAR    15.0   // meters — start reducing speed
#define SLOWDOWN_RADIUS_NEAR    8.0   // meters — further reduce speed

// Steering thresholds
#define HEADING_DEADBAND        5.0f  // degrees — "close enough", go straight
#define TURN_IN_PLACE_THRESH   25.0f  // degrees — stop and rotate first
#define MIN_TURN_SPEED         100    // PWM for gentle turn-in-place
#define MAX_TURN_SPEED         160    // PWM for aggressive turn-in-place

// PD controller gains
#define STEERING_KP            2.0f
#define STEERING_KD            0.5f
#define STEERING_MAX_CORRECTION 80    // max differential PWM

// Obstacle avoidance
#define OBSTACLE_TRIGGER_CM     30    // cm — trigger avoidance
#define AVOID_STOP_DURATION    300    // ms — settle after stop
#define AVOID_TURN_DURATION    800    // ms — time spent turning away
#define AVOID_TURN_SPEED       140    // PWM during avoidance turn
#define AVOID_DRIVE_DURATION  1500    // ms — time spent driving past
#define AVOID_DRIVE_SPEED      100    // PWM during avoidance forward
#define AVOID_RECHECK_SETTLE   300    // ms — settle before recheck
#define MAX_AVOID_ATTEMPTS       3    // skip waypoint after N failures

// Safety
#define HEADING_STALE_LIMIT    500UL  // ms — stop if heading older than this
#define STALL_CHECK_INTERVAL 30000UL  // ms — check for movement every 30 s
#define STALL_DISTANCE          3.0   // meters — must move this far
#define MAX_STALL_RECOVERIES     3    // skip waypoint after N stalls
#define STALL_REVERSE_DURATION 1000   // ms — reverse when stalled
#define NAV_TIMEOUT          600000UL // ms — 10 min max navigation time
#define CONSECUTIVE_ARRIVALS     3    // # of in-radius readings to confirm
#define SPEED_GATE_THRESHOLD   0.3f   // m/s — hold bearing when slower

// Position history for RETURN feature
#define MAX_HISTORY_POINTS     100

// ===================== DATA STRUCTURES =====================

struct Waypoint {
    double latitude;
    double longitude;
    int id;
    bool reached;
};

struct HistoryPoint {
    double latitude;
    double longitude;
    unsigned long timestamp;
};

// ===================== NAVIGATION CLASS =====================

class Navigation {
public:
    // ---- Navigation state machine ----
    enum NavState : uint8_t {
        NAV_IDLE,              // No active navigation
        NAV_TURN_TO_TARGET,    // Phase 1: turn in place (|error| > 25°)
        NAV_DRIVE_TO_TARGET,   // Phase 2+3: PD steering toward waypoint
        NAV_WAYPOINT_REACHED,  // Brief stop, advance to next
        NAV_AVOID_STOP,        // Obstacle: stop + settle
        NAV_AVOID_TURN,        // Obstacle: turn away
        NAV_AVOID_DRIVE,       // Obstacle: drive past
        NAV_AVOID_RECHECK,     // Obstacle: check if clear
        NAV_COMPLETE,          // All waypoints reached
        NAV_STALLED            // Recovery attempt (reversing)
    };

    Navigation();
    void begin(GPSHandler* g, MotorControl* m, ObstacleAvoidance* o);
    void setHeading(float h);

    // Waypoint management
    void addWaypoint(double lat, double lon, int id);
    void clearWaypoints();
    int getWaypointCount();
    int getCurrentWaypointIndex();

    // Navigation control
    void start();
    void stop();
    void resume();
    void pause();
    void update();
    bool isComplete();
    NavState getNavState() const;

    // Waypoint completion tracking
    bool isWaypointJustCompleted();
    int getLastCompletedWaypointId();
    void clearWaypointCompletionFlag();

    // Waypoint history & return feature
    void saveCurrentPosition();
    void returnToStart();
    bool isReturning() const;
    void clearHistory();

    // Update GPS data from Pi (for redundancy)
    void updateGpsData(double lat, double lon, float speed, float heading);

private:
    GPSHandler* gps;
    MotorControl* motors;
    ObstacleAvoidance* obstacleAvoid;

    // Waypoints
    Waypoint waypoints[MAX_WAYPOINTS];
    int waypointCount;
    int currentWaypointIndex;
    bool navigationActive;

    // State machine
    NavState navState;
    unsigned long stateEntryTime;      // millis() when current state entered

    // Heading from Pi compass
    float currentHeading;
    unsigned long lastHeadingUpdateMs;  // set inside setHeading()

    // PD controller
    float lastHeadingError;

    // Waypoint arrival confirmation
    uint8_t consecutiveArrivals;

    // Obstacle avoidance
    uint8_t avoidAttempts;             // per-waypoint attempt counter
    int8_t  avoidTurnDir;              // +1 right, -1 left — alternates

    // Stall detection
    double stallCheckLat;
    double stallCheckLon;
    unsigned long stallCheckTime;
    uint8_t stallRecoveries;           // per-waypoint recovery counter

    // Speed gate — hold last good bearing when nearly stationary
    float lastGoodBearing;
    bool  lastBearingValid;

    // Navigation timeout
    unsigned long navStartTime;

    // Position history for return-to-start
    HistoryPoint history[MAX_HISTORY_POINTS];
    int historyCount;
    int currentHistoryIndex;
    bool returningToStart;

    // Waypoint completion tracking
    bool waypointJustCompleted;
    int lastCompletedWaypointId;

    // Position save timer
    unsigned long lastPositionSave;

    // ---- Private methods ----
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    double calculateBearing(double lat1, double lon1, double lat2, double lon2);
    float  normalizeHeadingError(float error);
    int    speedForDistance(double distance);
    void   enterState(NavState newState);
    void   resetWaypointCounters();

    // State handlers — each runs < 100 µs, NEVER blocks
    void handleTurnToTarget();
    void handleDriveToTarget();
    void handleWaypointReached();
    void handleAvoidStop();
    void handleAvoidTurn();
    void handleAvoidDrive();
    void handleAvoidRecheck();
    void handleStalled();
    void handleReturnNavigation();
    void advanceWaypoint();
};

#endif
