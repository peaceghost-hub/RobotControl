/*
 * Navigation Header
 * GPS Waypoint Navigation System
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "gps_handler.h"
#include "compass_handler.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"

#define MAX_WAYPOINTS 20
#define WAYPOINT_RADIUS 5.0  // meters - distance to consider waypoint reached

struct Waypoint {
    float latitude;
    float longitude;
    int id;
    bool reached;
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
    
    unsigned long obstacleDetectedTime;
    bool inAvoidanceMode;
    
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
    
private:
    float calculateDistance(float lat1, float lon1, float lat2, float lon2);
    float calculateBearing(float lat1, float lon1, float lat2, float lon2);
    void navigateToWaypoint();
    void handleObstacleAvoidance();
};

#endif
