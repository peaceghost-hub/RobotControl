/*
 * Navigation Header
 * GPS Waypoint Navigation System with Fallback & History Tracking
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "globals.h"
#include "gps_handler.h"
#include "compass_handler.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"

#define WAYPOINT_RADIUS 5.0  // meters - distance to consider waypoint reached
#define MAX_HISTORY_POINTS 100  // Store last 100 positions for RETURN feature

struct Waypoint {
    float latitude;
    float longitude;
    int id;
    bool reached;
};

struct HistoryPoint {
    float latitude;
    float longitude;
    unsigned long timestamp;
};

class Navigation {
private:
    GPSHandler* gps;
    CompassHandler* compass;
    MotorControl* motors;
    ObstacleAvoidance* obstacleAvoid;
    
    Waypoint waypoints[MAX_WAYPOINTS];
    int waypointCount;
    int currentWaypointIndex;
    bool navigationActive;
    
    // Waypoint history for RETURN feature
    HistoryPoint history[MAX_HISTORY_POINTS];
    int historyCount;
    int currentHistoryIndex;
    bool returningToStart;
    
    // Navigation mode control
    bool useCompass;  // If false, use GPS-only navigation
    
    unsigned long obstacleDetectedTime;
    bool inAvoidanceMode;
    unsigned long lastPositionSave;
    
public:
    Navigation();
    void begin(GPSHandler* g, CompassHandler* c, MotorControl* m, ObstacleAvoidance* o);
    
    void addWaypoint(float lat, float lon, int id);
    void clearWaypoints();
    int getWaypointCount();
    int getCurrentWaypointIndex();
    
    void start();
    void stop();
    void resume();
    void pause();
    void update();
    bool isComplete();

    // Compass fallback & GPS-only navigation
    void setUseCompass(bool enabled);
    bool getUseCompass() const;
    void fallbackToGpsOnly();  // Called when compass fails
    
    // Waypoint history & return feature
    void saveCurrentPosition();
    void returnToStart();
    bool isReturning() const;
    void clearHistory();
    
    // Update GPS data from Pi (for redundancy)
    void updateGpsData(float lat, float lon, float speed, float heading);
    
private:
    float calculateDistance(float lat1, float lon1, float lat2, float lon2);
    float calculateBearing(float lat1, float lon1, float lat2, float lon2);
    void navigateToWaypoint();
    void navigateToWaypointGpsOnly();  // Without compass
    void handleObstacleAvoidance();
    void handleReturnNavigation();
};

#endif
