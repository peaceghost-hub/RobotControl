# ZigBee Communication Protocol Reference

## Overview
XBee/ZigBee modules on Serial2 (pins 16/17) provide wireless communication between:
- **Arduino Mega** (robot navigation controller)
- **Arduino Uno** (handheld remote controller)

Baud Rate: **57600** (recommended default in this repo; configurable on the module)

## Message Format
All messages are ASCII strings terminated with newline (`\n`).

---

## Messages FROM Mega TO Remote (Status Updates)

### STATUS Messages
Robot operation status updates.

**Format:** `STATUS,<mode>,<nav_state>`

**Examples:**
```
STATUS,AUTO,RUN       # Autonomous navigation active
STATUS,MANUAL,IDLE    # Manual control, no navigation
STATUS,AUTO,DONE      # Waypoints completed
STATUS,NO_GPS,WAITING # GPS signal lost
```

**Parameters:**
- `mode`: `AUTO` or `MANUAL`
- `nav_state`: `RUN`, `IDLE`, `DONE`, `WAITING`

---

### GPS Messages
Current position and heading data.

**Format:** `GPS,<lat>,<lon>,<speed>,<heading>,<sats>`

**Example:**
```
GPS,37.7749,-122.4194,0.5,45.2,8
```

**Parameters:**
- `lat`: Latitude (decimal degrees, -90 to +90)
- `lon`: Longitude (decimal degrees, -180 to +180)
- `speed`: Speed in km/h
- `heading`: Compass heading (0-360°, 0=North)
- `sats`: Number of satellites (GPS fix quality)

**Send Rate:** Every 2 seconds

---

### OBSTACLE Alerts (Manual Mode Only)
Warns operator of obstacles when in manual control.

**Format:** `OBSTACLE,<distance>,<direction>`

**Examples:**
```
OBSTACLE,25,FRONT    # 25cm obstacle ahead
OBSTACLE,15,FRONT    # Getting closer!
OBSTACLE,8,FRONT     # Very close!
```

**Parameters:**
- `distance`: Distance in centimeters
- `direction`: Always `FRONT` (future: LEFT/RIGHT from servo scan)

**Send Rate:** Every 1 second when obstacle detected

---

### READY Message
Sent once on boot after handshake.

**Format:** `READY`

**Example:**
```
READY
```

---

## Messages FROM Remote TO Mega (Commands)

### Manual Control Commands
Direct motor control when in manual mode.

#### Movement Commands
**Format:** `MCTL,<command>[,<speed>]`

**Examples:**
```
MCTL,FORWARD,200    # Move forward at speed 200
MCTL,BACKWARD,150   # Move backward at speed 150
MCTL,LEFT,180       # Turn left at speed 180
MCTL,RIGHT,180      # Turn right at speed 180
MCTL,STOP           # Stop all motors
```

**Speed Range:** 0-255 (PWM duty cycle)

**Manual Timeout:** If no command received for 3 seconds, robot stops and can resume autonomous navigation.

---

#### Mode Switch Commands
**Format:** `MCTL,<mode>`

**Examples:**
```
MCTL,MANUAL         # Enter manual control mode
MCTL,AUTO           # Return to autonomous navigation
MCTL,RELEASE        # Same as AUTO
```

---

### Handshake Protocol
Establishes reliable connection.

**Remote sends:** `PING`
**Mega responds:** `READY`

**Example sequence:**
```
Remote: PING
Mega:   READY
Remote: STATUS?
Mega:   STATUS,AUTO,IDLE
```

---

## Arduino Uno Remote Example Code

### Basic Manual Control
```cpp
#include <SoftwareSerial.h>

SoftwareSerial xbee(2, 3); // RX, TX

void setup() {
  Serial.begin(9600);
  xbee.begin(9600);
  
  // Wait for connection
  delay(2000);
  xbee.println("PING");
}

void loop() {
  // Read commands from USB serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "w") {
      xbee.println("MCTL,FORWARD,200");
    } else if (cmd == "s") {
      xbee.println("MCTL,BACKWARD,150");
    } else if (cmd == "a") {
      xbee.println("MCTL,LEFT,180");
    } else if (cmd == "d") {
      xbee.println("MCTL,RIGHT,180");
    } else if (cmd == "x") {
      xbee.println("MCTL,STOP");
    } else if (cmd == "auto") {
      xbee.println("MCTL,AUTO");
    } else if (cmd == "manual") {
      xbee.println("MCTL,MANUAL");
    }
  }
  
  // Read status from Mega
  if (xbee.available()) {
    String msg = xbee.readStringUntil('\n');
    Serial.println(msg);
    
    // Parse obstacle alerts
    if (msg.startsWith("OBSTACLE")) {
      int dist = msg.substring(9, msg.indexOf(',', 9)).toInt();
      if (dist < 20) {
        Serial.println("*** WARNING: CLOSE OBSTACLE! ***");
      }
    }
  }
}
```

### Joystick Control
```cpp
// Analog joystick on pins A0 (X) and A1 (Y)
void loop() {
  int x = analogRead(A0);
  int y = analogRead(A1);
  
  // Deadzone
  if (abs(x - 512) < 100 && abs(y - 512) < 100) {
    xbee.println("MCTL,STOP");
    delay(100);
    return;
  }
  
  // Forward/backward
  if (y > 600) {
    xbee.println("MCTL,FORWARD,200");
  } else if (y < 400) {
    xbee.println("MCTL,BACKWARD,150");
  }
  
  // Left/right
  if (x > 600) {
    xbee.println("MCTL,RIGHT,180");
  } else if (x < 400) {
    xbee.println("MCTL,LEFT,180");
  }
  
  delay(100); // Don't spam commands
}
```

### LCD Display with Obstacle Warning
```cpp
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {
  xbee.begin(9600);
  lcd.begin(16, 2);
  lcd.print("Waiting...");
  xbee.println("PING");
}

void loop() {
  if (xbee.available()) {
    String msg = xbee.readStringUntil('\n');
    
    if (msg.startsWith("STATUS")) {
      // STATUS,AUTO,RUN
      lcd.setCursor(0, 0);
      lcd.print("Mode: ");
      lcd.print(msg.substring(7, msg.indexOf(',', 7)));
      lcd.print("    ");
    }
    else if (msg.startsWith("GPS")) {
      // GPS,lat,lon,speed,heading,sats
      int idx1 = msg.indexOf(',');
      int idx2 = msg.indexOf(',', idx1 + 1);
      int idx3 = msg.indexOf(',', idx2 + 1);
      int idx4 = msg.indexOf(',', idx3 + 1);
      
      float speed = msg.substring(idx2 + 1, idx3).toFloat();
      float heading = msg.substring(idx3 + 1, idx4).toFloat();
      
      lcd.setCursor(0, 1);
      lcd.print("S:");
      lcd.print(speed, 1);
      lcd.print(" H:");
      lcd.print(heading, 0);
      lcd.print("  ");
    }
    else if (msg.startsWith("OBSTACLE")) {
      // OBSTACLE,25,FRONT
      int dist = msg.substring(9, msg.indexOf(',', 9)).toInt();
      
      lcd.setCursor(0, 1);
      lcd.print("OBST: ");
      lcd.print(dist);
      lcd.print("cm!     ");
      
      // Flash warning
      if (dist < 20) {
        tone(buzzerPin, 1000, 100);
      }
    }
  }
}
```

---

## Timing Specifications

| Event | Interval |
|-------|----------|
| GPS updates | 2 seconds |
| Obstacle alerts | 1 second (when detected) |
| Manual timeout | 3 seconds (no command = auto resume) |
| Status updates | On state change |

---

## Troubleshooting

### No Communication
1. Check XBee power (3.3V, 250mA)
2. Verify baud rate: 9600 on both sides
3. Send `PING` from remote, expect `READY`
4. Check TX/RX pin connections (crossed: TX→RX)

### Delayed Commands
- Reduce command frequency (100-200ms minimum)
- Check for buffer overflow (messages too long)
- Verify XBee firmware is latest version

### Obstacle Alerts Not Received
- Only sent in **manual mode**
- Check ultrasonic sensor (echo returning?)
- Distance must be < 30cm to trigger

### Robot Not Responding
- Manual timeout may have occurred (send command every 2s)
- Check I2C heartbeat from Raspberry Pi
- Verify robot is in correct mode (STATUS message)

---

## Protocol Extensions (Future)

Potential additions:
- `OBSTACLE,<dist>,LEFT` - Left path scan result
- `OBSTACLE,<dist>,RIGHT` - Right path scan result
- `BATTERY,<percent>` - Battery level monitoring
- `ERROR,<code>,<msg>` - Error reporting
- `WAYPOINT,<num>,<total>` - Progress updates

---

## Security Notes

**No encryption or authentication is implemented.**

For production use, consider:
- XBee API mode with encryption enabled
- Command authentication codes
- Frequency hopping (XBee Series 2 / ZigBee PRO)
- PAN ID configuration to isolate networks
