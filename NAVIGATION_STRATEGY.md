# Autonomous GPS Waypoint Navigation — How It Works

## Overview

The robot navigates autonomously to a series of GPS waypoints using a closed-loop
control system. The Pi acts as the command coordinator, while the Arduino Mega
executes the actual navigation logic in real-time using onboard GPS and compass.

---

## The Complete Flow (Step by Step)

### 1. User Adds Waypoints (Dashboard → DB)

The user enters waypoint coordinates in the dashboard UI (manually or by clicking
the map). Each waypoint is stored in the SQLite database with a latitude, longitude,
sequence number, and optional description.

**Files:** `main.js → addWaypoint()` → HTTP POST `/api/waypoints` → `app.py` → DB

### 2. User Clicks "Send Waypoints" (Dashboard → Pi → Mega via I2C)

When the user clicks the "Send Waypoints to Robot" button:

```
Browser: sendWaypointsToRobot()
  → sendRobotCommand('WAYPOINT_PUSH')
  → HTTP POST /api/commands {command: 'WAYPOINT_PUSH'}
  → DB insert (RobotCommand)
  → Pi command_loop polls → _process_command('WAYPOINT_PUSH')
  → _handle_waypoint_push()
```

The Pi then:
1. Fetches all waypoints from the dashboard API (`api_client.get_waypoints()`)
2. Sorts them by sequence number
3. Sends them to the Mega over I2C using a 3-step protocol:

```
Step 1:  CMD_WAYPOINT_CLEAR ('C')     → Mega clears its waypoint array
Step 2:  CMD_WAYPOINT_PACKET ('W') ×N → One per waypoint, containing:
           - id:        uint16 (2 bytes, little-endian)
           - sequence:  uint8  (1 byte)
           - latitude:  float  (4 bytes, little-endian IEEE754)
           - longitude: float  (4 bytes, little-endian IEEE754)
           Total: 11 bytes per waypoint
Step 3:  CMD_WAYPOINT_COMMIT ('M')    → Mega commits buffered waypoints to
                                        navigation.addWaypoint() for each
```

**Files:**
- `i2c_comm.py` → `send_waypoints()`, `_encode_waypoint_packet()`
- `robot_navigation.ino` → `receiveEvent()` handles `CMD_WAYPOINT_PACKET`, `storePendingWaypoint()`, `commitPendingWaypoints()`

### 3. User Clicks "Start Navigation" (Dashboard → Pi → Mega)

```
Browser: sendNavigationCommand('start')
  → sendRobotCommand('NAV_START')
  → Pi → I2C CMD_NAV_START ('S')
  → Mega: navigation.start(), sets navigationActive = true, controlMode = MODE_AUTO
```

The Mega requires `waypointCount > 0` to accept NAV_START; otherwise it replies with
ERR_NO_WAYPOINTS.

### 4. Navigation Loop (Mega — Real-Time, Every Loop Cycle)

The Mega's `loop()` calls `navigation.update()` during the `STATE_I2C` superloop step
when `navigationActive == true`.

**`navigation.update()` does the following on every call:**

```
1. Guard: if (!navigationActive || !gps->isValid()) → return
   Navigation only runs when GPS has a valid fix.

2. OBSTACLE CHECK (highest priority):
   if (obstacleAvoid->isObstacleDetected())
     → motors->stop()  // immediate stop
     → handleObstacleAvoidance()  // scan + avoid
     → return  // don't navigate until obstacle cleared

3. POSITION HISTORY:
   Every 5 seconds, save current GPS position to history ring buffer
   (used for RETURN TO START feature).

4. RETURN MODE:
   If returningToStart is active, call handleReturnNavigation() instead
   of normal waypoint following.

5. NAVIGATE TO WAYPOINT:
   If useCompass == true  → navigateToWaypoint()   (primary mode)
   If useCompass == false → navigateToWaypointGpsOnly()  (fallback)
```

### 5. Waypoint Following Algorithm (`navigateToWaypoint()`)

This is the core navigation function, called every loop cycle:

```
a) Get current GPS position: lat, lon from GPSHandler

b) Calculate distance to current waypoint:
   Uses HAVERSINE FORMULA:
     dLat = (targetLat - currentLat) * π/180
     dLon = (targetLon - currentLon) * π/180
     a = sin²(dLat/2) + cos(lat1) × cos(lat2) × sin²(dLon/2)
     c = 2 × atan2(√a, √(1-a))
     distance = R × c   (R = 6,371,000 meters)

c) Check if waypoint reached:
   if (distance < WAYPOINT_RADIUS)    // WAYPOINT_RADIUS = 5.0 meters
     → Mark waypoint as reached
     → motors->stop()
     → Set waypointJustCompleted flag (for status reporting to Pi)
     → Advance to next waypoint (currentWaypointIndex++)
     → If all waypoints done → navigationActive = false
     → return

d) Calculate target bearing to waypoint:
   Uses FORWARD AZIMUTH formula:
     y = sin(dLon) × cos(lat2)
     x = cos(lat1)×sin(lat2) - sin(lat1)×cos(lat2)×cos(dLon)
     bearing = atan2(y, x) × 180/π
     Normalize to 0-360°

e) Get current heading from compass:
   currentHeading = compass->getHeading()  // QMC5883L magnetometer

f) Steer toward waypoint:
   motors->adjustForHeading(currentHeading, targetBearing)
   This adjusts left/right motor speeds differentially based on
   the angular difference between current heading and target bearing.
```

### 6. Obstacle Avoidance During Navigation

If an obstacle is detected (KY-032 IR sensor or ultrasonic), the navigation
enters avoidance mode:

```
1. IMMEDIATE STOP — motors->stop()
2. Save the current target bearing (to resume later)
3. SCAN — obstacleAvoid->scanPath() checks left, right, front distances
4. DECISION TREE:
   - Right clear (>30cm)?  → Turn right 90°, move forward 1.5s
   - Left clear (>30cm)?   → Turn left 90°, move forward 1.5s
   - Both blocked?         → Try sharp right 120°, check; if still blocked,
                             try sharp left 120°, check
   - All blocked?          → Stay stopped (inAvoidanceMode remains true)
5. After avoidance maneuver, check if still blocked
6. If clear → inAvoidanceMode = false, normal navigation resumes
```

### 7. Waypoint Completion Notification (Mega → Pi → Dashboard)

When a waypoint is reached:

```
Mega: navigation.update() detects distance < 5m
  → waypointJustCompleted = true
  → lastCompletedWaypointId = waypoint.id

Pi: status_loop() → request_status() → reads waypoint_just_completed flag
  → api_client.complete_waypoint(id)
  → Dashboard marks waypoint as completed
  → WebSocket 'waypoint_completed' event → browser shows notification
```

### 8. Navigation Controls

| Button | Command | I2C Opcode | Mega Action |
|--------|---------|------------|-------------|
| Start | NAV_START | 'S' (0x53) | `navigation.start()`, `controlMode = MODE_AUTO` |
| Pause | NAV_PAUSE | 'A' (0x41) | `navigation.pause()`, motors stop |
| Resume | NAV_RESUME | 'R' (0x52) | `navigation.resume()`, only if waypoints remain |
| Stop | NAV_STOP | 'T' (0x54) | `navigation.stop()`, motors stop, mode stays AUTO |

### 9. GPS-Only Fallback (No Compass)

If the compass fails, `navigation.fallbackToGpsOnly()` sets `useCompass = false`.
The `navigateToWaypointGpsOnly()` method uses a simpler strategy:

```
- Calculate distance + bearing to waypoint (same as above)
- Drive forward 250ms, stop, re-evaluate position
- This "nudge and check" approach works but is less precise
```

### 10. Return to Start

The navigation system records GPS positions every 5 seconds into a 100-point
ring buffer. When `returnToStart()` is called:

```
- Find the oldest recorded position (approximates the start)
- Navigate toward it using compass + GPS (same as waypoint following)
- When within WAYPOINT_RADIUS (5m) of start → stop, returningToStart = false
```

---

## Key Constants

| Constant | Value | Defined In |
|----------|-------|------------|
| `WAYPOINT_RADIUS` | 5.0 meters | navigation.h |
| `MAX_WAYPOINTS` | 20 | globals.h |
| `MAX_HISTORY_POINTS` | 100 | navigation.h |
| `DEVIATION_THRESHOLD` | 15.0° | navigation.h |
| `OBSTACLE_SAFE_DISTANCE` | 30 cm | navigation.h |
| `OBSTACLE_CRITICAL_DISTANCE` | 20 cm | navigation.h |
| `MANUAL_TIMEOUT` | 1500 ms | globals.h |

---

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                         DASHBOARD                                │
│  [Add Waypoints] ─────→ DB (SQLite)                             │
│  [Send to Robot] ──→ POST /api/commands ──→ DB queue            │
│  [Start Nav]     ──→ POST /api/commands ──→ DB queue            │
│  [Manual Drive]  ──→ WS instant_command  (bypasses DB)          │
└─────────────────────────────┬────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│                       RASPBERRY PI                               │
│  command_loop (2s poll)     │    instant_command_loop (100ms)    │
│  ← GET /api/commands/pending│    ← GET /api/commands/instant    │
│          │                  │              │                      │
│          ▼                  │              ▼                      │
│  _process_command()  ←──────┴──────────────┘                     │
│     │                                                            │
│     ├─ WAYPOINT_PUSH → send_waypoints() ──→ I2C: C + W×N + M   │
│     ├─ NAV_START     → start_navigation() → I2C: 'S'           │
│     ├─ MANUAL_DRIVE  → _handle_manual_drive() → I2C: 'V'       │
│     └─ etc.                                                      │
└─────────────────────────────┬────────────────────────────────────┘
                              │ I2C (smbus2)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│                      ARDUINO MEGA                                │
│  receiveEvent() → parse command                                  │
│     │                                                            │
│     ├─ WAYPOINT_PACKET → storePendingWaypoint()                 │
│     ├─ WAYPOINT_COMMIT → commitPendingWaypoints()               │
│     │                     → navigation.addWaypoint() for each   │
│     ├─ NAV_START → navigation.start()                           │
│     └─ MANUAL_OVERRIDE → manual motor control                   │
│                                                                  │
│  loop() → STATE_I2C step:                                        │
│     navigation.update()                                          │
│       → GPS fix? → Obstacle? → Calculate distance/bearing       │
│       → motors.adjustForHeading(compass_heading, target_bearing) │
│       → Waypoint reached? → advance → notify Pi                 │
└──────────────────────────────────────────────────────────────────┘
```

## Verification: Is the Navigation Strategy Correct?

**YES.** The navigation strategy in the code is mathematically and architecturally sound:

1. ✅ **Haversine distance** — correct implementation for GPS distance on Earth's surface
2. ✅ **Forward azimuth bearing** — correct great-circle initial bearing formula
3. ✅ **Compass-based steering** — differential motor adjustment based on heading error
4. ✅ **Waypoint radius** — 5m is reasonable for consumer GPS accuracy (~2-5m CEP)
5. ✅ **Obstacle-first priority** — obstacles are checked BEFORE navigation every cycle
6. ✅ **Sequential waypoint following** — linear progression through waypoint list
7. ✅ **GPS-only fallback** — degrades gracefully if compass fails
8. ✅ **Position history** — enables return-to-start functionality
9. ✅ **Waypoint completion reporting** — status flag read by Pi, reported to dashboard
10. ✅ **Non-blocking** — `update()` runs inside the main loop without blocking delays
    (except during obstacle avoidance scanning, which uses brief `delay()` calls)

**One concern:** The obstacle avoidance `handleObstacleAvoidance()` uses `delay()` calls
(500ms, 1500ms) which DO block the main loop during scanning. This is acceptable for
safety-critical obstacle response but will pause I2C/heartbeat processing during that
window. The Mega's heartbeat timeout (5000ms by default) should accommodate this.
