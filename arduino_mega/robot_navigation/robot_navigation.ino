

/*
 * Environmental Monitoring Robot - Navigation Controller (Arduino Mega 2560)
 * 
 * MULTI-PROTOCOL WIRELESS SUPPORT
 * Select wireless protocol in globals.h:
 *   - WIRELESS_PROTOCOL_ZIGBEE: XBee modules (long range, reliable)
 *   - WIRELESS_PROTOCOL_LORA: LoRa modules (very long range, low power)
 *   - WIRELESS_PROTOCOL_BLE: Bluetooth (mobile device control)
 *
 * Responsibilities:
 *  - Waypoint navigation using GPS + compass
 *  - Obstacle avoidance via HC-SR04 on servo
 *  - Dual control: autonomous (from Raspberry Pi via I2C) or manual (wireless)
 *  - I2C slave interface for Raspberry Pi master (address 0x08)
 *  - Multi-protocol wireless fail-safe manual override channel
 *  - Buzzer feedback for communication status
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <SPI.h>

#include "gps_handler.h"
#include "compass_handler.h"
#include "navigation.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "globals.h"

// Select wireless implementation based on globals.h configuration
#ifdef WIRELESS_PROTOCOL_ZIGBEE
  #include "zigbee_driver.h"
  ZigBeeDriver wireless;
#elif defined(WIRELESS_PROTOCOL_LORA)
  #include "lora_driver.h"
  LoRaDriver wireless(WIRELESS_SERIAL);
#elif defined(WIRELESS_PROTOCOL_BLE)
  #include "bluetooth_driver.h"
  BluetoothDriver wireless;
#elif defined(WIRELESS_PROTOCOL_CC1101)
  #include "cc1101_driver.h"
  CC1101Driver wireless;
#else
  #error "No wireless protocol selected! Edit globals.h and uncomment WIRELESS_PROTOCOL_ZIGBEE, WIRELESS_PROTOCOL_LORA, WIRELESS_PROTOCOL_BLE, or WIRELESS_PROTOCOL_CC1101"
#endif

// Include base interface for type checking
#include "wireless_interface.h"

// ------------------------------- Components ----------------------------------
GPSHandler gps;
CompassHandler compass;
Navigation navigation;
MotorControl motors;
ObstacleAvoidance obstacleAvoid;

// Track last manual motor command so we can enforce obstacle safety even if
// the operator stops sending commands while the robot is moving.
static int8_t lastManualLeft = 0;
static int8_t lastManualRight = 0;

static inline bool frontUltrasonicObstacle30cm() {
  const int dist = obstacleAvoid.getDistance();
  return (dist > 0 && dist < OBSTACLE_THRESHOLD);
}

// ------------------------------- State ---------------------------------------
uint8_t responseBuffer[32];
uint8_t responseLength = 0;

PendingWaypoint pendingWaypoints[20];  // Adjust size as needed
uint8_t pendingWaypointCount = 0;

bool navigationActive = false;
bool manualOverride = false;
bool i2cHandshakeComplete = false;
bool wirelessHandshakeComplete = false;
bool lineFollowEnabled = false;

// Track I2C activity to avoid master transactions during slave callbacks
volatile bool i2cCallbackActive = false;
volatile unsigned long lastI2CActivityMs = 0;

// Defer I2C command handling out of interrupt context
volatile bool i2cCommandPending = false;
volatile uint8_t i2cPendingCommand = 0;
volatile uint8_t i2cPendingLength = 0;
uint8_t i2cPendingPayload[32];

unsigned long lastStatusUpdate = 0;
unsigned long lastWirelessGps = 0;
unsigned long lastManualCommand = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastWirelessUpdate = 0;
unsigned long lastWirelessCommand = 0;  // Track UNO remote activity
bool wirelessControlActive = false;      // UNO remote has priority

int manualSpeed = 180;
String wirelessBuffer;
ControlMode controlMode = MODE_AUTO;

// Heading received from Pi (since compass moved to Pi)
float piHeading = 0.0;

// Wireless compatibility: map old zigbee names to wireless
#define zigbeeHandshakeComplete wirelessHandshakeComplete
#define lastZigbeeGps lastWirelessGps

// For backward compatibility with functions still named "Zigbee"
String zigbeeBuffer;

// ========================== FORWARD DECLARATIONS ============================
void onI2CReceive(int bytes);
void onI2CRequest();
void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length);
void prepareAck(uint8_t code = 0);
void prepareError(uint8_t code);
void prepareGpsResponse();
void prepareStatusResponse();
void resetPendingWaypoints();
void storePendingWaypoint(const WaypointPacket& packet);
void commitPendingWaypoints();

void enterManualMode();
void exitManualMode(bool resumeAutonomous);
void processManualTimeout();

void handleWireless();
void processWirelessMessage(const String& message);
void sendWirelessGps();
void sendWirelessStatus();
void sendWirelessReady();
void sendWirelessObstacleAlert(int distance);

void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void markI2CHandshake();
void markWirelessHandshake();
void checkReadyTone();

uint8_t readBatteryPercent();
uint8_t readSignalQuality();

void recoverI2CBus();
bool isI2CBusStuck();
void i2cReinitSlave();

// ========================== LEGACY FUNCTION WRAPPERS ========================
// For backward compatibility, maintain old function names that call new ones
void handleZigbee() { handleWireless(); }
void processZigbeeMessage(const String& message) { processWirelessMessage(message); }
void sendZigbeeGps() { sendWirelessGps(); }
void sendZigbeeStatus() { sendWirelessStatus(); }
void sendZigbeeReady() { sendWirelessReady(); }
void markZigbeeHandshake() { markWirelessHandshake(); }

// ========================== I2C BUS RECOVERY ==============================
// Releases a stuck I2C bus by toggling SCL and issuing a STOP condition.
// This prevents the Mega from ACKing all addresses after a bus lock.
void recoverI2CBus() {
  // Ensure pull-ups are enabled
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delay(5);

  // If SDA is stuck low, clock SCL to free the bus
  if (digitalRead(SDA) == LOW) {
    for (uint8_t i = 0; i < 9; i++) {
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }
  }

  // Issue STOP condition: SDA low -> SCL high -> SDA high
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

bool isI2CBusStuck() {
  // Check hardware I2C pins on Mega: SDA=20, SCL=21
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  // Bus is considered stuck if either SDA or SCL held LOW
  return (digitalRead(SDA) == LOW) || (digitalRead(SCL) == LOW);
}

void i2cReinitSlave() {
  // Fully reset Wire/TWI and re-enter slave mode
  Wire.end();
  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  // Debug note
  DEBUG_SERIAL.println(F("# I2C watchdog: bus recovered and slave reinitialized"));
}


// ========================== SETUP ==========================================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL) {
    delay(5);
  }

  DEBUG_SERIAL.println(F("\n# ========== ARDUINO MEGA NAVIGATION CONTROLLER =========="));
  DEBUG_SERIAL.println(F("# Booting subsystems..."));
  
  // Display selected wireless protocol
  #ifdef WIRELESS_PROTOCOL_ZIGBEE
    DEBUG_SERIAL.print(F("# Wireless: ZigBee (UART Transparent) on Serial2 @ "));
    DEBUG_SERIAL.print(ZIGBEE_BAUD);
    DEBUG_SERIAL.println(F(" baud"));
  #elif defined(WIRELESS_PROTOCOL_LORA)
    DEBUG_SERIAL.print(F("# Wireless: LoRa (UART Transparent) on Serial2 @ "));
    DEBUG_SERIAL.print(LORA_BAUD);
    DEBUG_SERIAL.println(F(" baud"));
  #elif defined(WIRELESS_PROTOCOL_BLE)
    DEBUG_SERIAL.println(F("# Wireless: Bluetooth on Serial3 @ 38400 baud"));
  #endif

  GPS_SERIAL.begin(GPS_BAUD);

  // Recover I2C bus before initializing Wire (in case SDA/SCL are stuck)
  recoverI2CBus();

  // Initialize I2C as SLAVE to Raspberry Pi
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  if (gps.begin(GPS_SERIAL)) {
    DEBUG_SERIAL.println(F("# GPS initialized"));
  } else {
    DEBUG_SERIAL.println(F("# WARNING: GPS init failed - will use manual control only"));
    beepPattern(3, 200, 100);  // Triple beep warning
  }

  // Compass moved to Pi - no initialization needed here
  DEBUG_SERIAL.println(F("# Compass: Moved to Pi for I2C compatibility"));

  DEBUG_SERIAL.println(F("# I2C initialized:"));
  DEBUG_SERIAL.print(F("#   - Slave to Pi at 0x"));
  DEBUG_SERIAL.println(I2C_ADDRESS, HEX);
  DEBUG_SERIAL.println(F("#   - Compass moved to Pi"));

  motors.begin();
  obstacleAvoid.begin();
  navigation.begin(&gps, &compass, &motors, &obstacleAvoid);

  // Line follower sensor setup
  pinMode(LINE_FOLLOWER_ENA, OUTPUT);
  digitalWrite(LINE_FOLLOWER_ENA, LOW); // disabled by default
  pinMode(LINE_FOLLOWER_OUT, INPUT);
  
  // Wireless init is deferred to loop() to avoid blocking I2C slave setup
  #if !defined(WIRELESS_PROTOCOL_CC1101)
    // Non-SPI wireless can init immediately
    if (wireless.begin()) {
      DEBUG_SERIAL.print(F("# Wireless initialized: "));
      DEBUG_SERIAL.println(wireless.getProtocolName());
      sendWirelessReady();
      wireless.sendString("HELLO,MEGA_ROBOT");
    } else {
      DEBUG_SERIAL.println(F("# WARNING: Wireless init failed - I2C manual control only"));
      beepPattern(3, 200, 100);
    }
  #else
    DEBUG_SERIAL.println(F("# Wireless (CC1101): deferred init in loop"));
  #endif
  
  DEBUG_SERIAL.println(F("# Note: System continues with any component failures - graceful degradation enabled"));
  DEBUG_SERIAL.println(F("# Awaiting I2C and wireless handshakes..."));

  // Continuous startup beep for 3 seconds (audible tone)
  tone(BUZZER_PIN, 2000);
  delay(3000);
  noTone(BUZZER_PIN);

  // NOTE: I2C bus scan disabled - Arduino is configured as I2C SLAVE (0x08)
  // Scanning the bus would switch to master mode and break slave functionality
  // To scan I2C devices, run scan from Raspberry Pi: sudo i2cdetect -y 1
  
  DEBUG_SERIAL.println(F("# I2C slave mode active at address 0x08"));
  DEBUG_SERIAL.println(F("# To verify I2C devices, run 'sudo i2cdetect -y 1' from Pi"));
}

// ========================== MAIN LOOP ======================================
void loop() {
  // Deferred wireless init for CC1101 (after I2C slave is stable and only when safe)
  #if defined(WIRELESS_PROTOCOL_CC1101)
    static bool wirelessInitAttempted = false;
    if (!wirelessInitAttempted && millis() > 5000 && !i2cHandshakeComplete) {
      // Only init after 5 seconds AND no I2C activity yet (Pi may not be connected)
      wirelessInitAttempted = true;
      DEBUG_SERIAL.println(F("# Attempting CC1101 init (Pi inactive)..."));
      if (wireless.begin()) {
        DEBUG_SERIAL.println(F("# CC1101 backup control ready"));
      } else {
        DEBUG_SERIAL.println(F("# CC1101 init failed - waiting for Pi"));
      }
    }
    // If Pi connects via I2C, wireless is secondary/disabled
    if (i2cHandshakeComplete && wirelessInitAttempted) {
      static bool warnedOnce = false;
      if (!warnedOnce) {
        DEBUG_SERIAL.println(F("# Pi connected - wireless backup on standby"));
        warnedOnce = true;
      }
    }
  #endif
  
  // Update all sensors and communication
  // Note: compass.update() performs I2C master transactions to read HMC5883L
  // This is safe because:
  //   1. Arduino Wire library supports dual master/slave mode
  //   2. These transactions happen in main loop, NOT in I2C slave callbacks
  //   3. Slave callbacks (onI2CReceive/onI2CRequest) don't perform master transactions
  gps.update();

  // Process any pending I2C command from Pi (deferred from ISR)
  if (i2cCommandPending) {
    uint8_t command = 0;
    uint8_t length = 0;
    uint8_t payload[32];

    noInterrupts();
    command = i2cPendingCommand;
    length = i2cPendingLength;
    if (length > 32) {
      length = 32;
    }
    for (uint8_t i = 0; i < length; i++) {
      payload[i] = i2cPendingPayload[i];
    }
    i2cCommandPending = false;
    interrupts();

    handleI2CCommand(command, payload, length);
  }

  // Avoid I2C master transactions if Pi just accessed the slave interface
  bool i2cBusy = false;
  unsigned long lastI2C = 0;
  noInterrupts();
  i2cBusy = i2cCallbackActive;
  lastI2C = lastI2CActivityMs;
  interrupts();

  // I2C bus watchdog: if bus appears stuck, reinitialize slave
  static unsigned long lastI2CWatchdog = 0;
  if (millis() - lastI2CWatchdog > 100) {
    lastI2CWatchdog = millis();
    if (isI2CBusStuck() && !i2cBusy) {
      i2cReinitSlave();
    }
  }

  if (!i2cBusy && (millis() - lastI2C) > 5) {
    compass.update();          // Software I2C read from compass
  }
  obstacleAvoid.update();
  
  // Wireless update logic:
  // - When wirelessControlActive (manually engaged), always update (ignore I2C state)
  // - Otherwise, only update when I2C slave is completely idle (avoid SPI/I2C conflicts)
  #if defined(WIRELESS_PROTOCOL_CC1101)
    if (wirelessControlActive) {
      wireless.update();  // Priority mode: always update
    } else if (!i2cBusy && !i2cCommandPending && (millis() - lastI2C) > 50) {
      wireless.update();  // Standby mode: only when I2C idle
    }
  #else
    wireless.update();  // UART wireless has no SPI conflict
  #endif

  // Safety + alert: if an ultrasonic obstacle is detected within 20cm in front,
  // sound the buzzer like on init (triple beep), but with cooldown to not spam.
  static unsigned long lastObstacleBeep = 0;
  if (frontUltrasonicObstacle30cm()) {
    if (millis() - lastObstacleBeep > 2000) {  // Beep every 2 seconds while obstacle present
      beepPattern(3, 200, 100);  // Triple beep like on init
      lastObstacleBeep = millis();
    }
  }

  // If we're in manual mode and the last command was forward, stop immediately
  // when a front obstacle appears. Operator can still command reverse/turn.
  if (manualOverride && frontUltrasonicObstacle30cm()) {
    if (lastManualLeft > 0 && lastManualRight > 0) {
      motors.stop();
      lastManualLeft = 0;
      lastManualRight = 0;
    }
  }

  if (!manualOverride && navigationActive) {
    // Check GPS validity before navigation
    if (gps.isValid()) {
      navigation.update();

      if (navigation.isComplete()) {
        navigationActive = false;
        motors.stop();
        DEBUG_SERIAL.println(F("# All waypoints completed"));
        sendWirelessStatus();
      }
    } else {
      // No GPS fix - stop and wait
      static unsigned long lastGpsWarning = 0;
      if (millis() - lastGpsWarning > 5000) {
        DEBUG_SERIAL.println(F("# WARNING: Navigation paused - waiting for GPS fix"));
        sendWirelessStatus();
        motors.stop();
        lastGpsWarning = millis();
      }
    }
  }

  // Wireless control priority logic:
  // - If Pi (I2C) is active AND wireless not manually engaged, ignore wireless (Pi has priority)
  // - If Pi silent for 30+ seconds OR wireless manually engaged, accept wireless commands
  // - When wireless manually engaged, Mega operates independently (no I2C blocking)
  bool piActive = i2cHandshakeComplete && (millis() - lastI2CActivityMs < 30000) && !wirelessControlActive;
  
  if (!piActive || wirelessControlActive) {
    // Pi is dead/absent OR wireless control engaged from dashboard
    handleWireless();
    
    // If wireless was active but now idle for 30s (and not manually engaged), release control back to Pi
    if (wirelessControlActive && (millis() - lastWirelessCommand > 30000)) {
      // Only auto-release if it wasn't manually engaged via dashboard
      // (we'd need a flag to distinguish auto vs manual engagement, for now keep it engaged)
      // wirelessControlActive = false;
      // DEBUG_SERIAL.println(F("# Wireless idle 30s - returning to I2C mode"));
    }
  }
  
  processManualTimeout();

  // Optional line follow mode when enabled
  if (lineFollowEnabled && !manualOverride) {
    // Sensor OUT: HIGH on line, LOW off line (depends on module, invert if needed)
    int onLine = digitalRead(LINE_FOLLOWER_OUT);
    if (onLine == HIGH) {
      motors.forward(140);
    } else {
      // Simple search: wiggle to find line
      motors.turnLeft(130);
      delay(200);
      motors.turnRight(130);
      delay(200);
      motors.stop();
    }
  }

  // Alert operator of obstacles in manual mode
  if (manualOverride && obstacleAvoid.isObstacleDetected()) {
    static unsigned long lastObstacleAlert = 0;
    if (millis() - lastObstacleAlert > 1000) {  // Alert every 1 second
      int distance = obstacleAvoid.getDistance();
      sendWirelessObstacleAlert(distance);
      
      DEBUG_SERIAL.print(F("# Manual mode obstacle alert: "));
      DEBUG_SERIAL.print(distance);
      DEBUG_SERIAL.println(F("cm"));
      
      lastObstacleAlert = millis();
      // Audible warning is handled continuously by the obstacle buzzer tone.
    }
  }

  // GPS broadcasting via wireless:
  // - When wirelessControlActive, always broadcast (UNO needs GPS at control station)
  // - Otherwise, only if Pi inactive and handshake complete
  bool allowWirelessTx = wirelessControlActive;
  if (!allowWirelessTx) {
    allowWirelessTx = !i2cBusy && !i2cCommandPending && (millis() - lastI2C) > 100;
    #if defined(WIRELESS_PROTOCOL_CC1101)
      allowWirelessTx = allowWirelessTx && !i2cHandshakeComplete;
    #endif
  }
  
  if (allowWirelessTx && wirelessHandshakeComplete && (millis() - lastWirelessGps >= ZIGBEE_GPS_INTERVAL)) {
    sendWirelessGps();
    lastWirelessGps = millis();
  }

  if (millis() - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = millis();
    DEBUG_SERIAL.print(F("# Status -> mode:"));
    DEBUG_SERIAL.print(controlMode == MODE_AUTO ? F("AUTO") : F("MANUAL"));
    DEBUG_SERIAL.print(F(" nav:"));
    DEBUG_SERIAL.print(navigationActive ? F("RUN") : (navigation.isComplete() ? F("DONE") : F("IDLE")));
    DEBUG_SERIAL.print(F(" waypoints:"));
    DEBUG_SERIAL.print(navigation.getWaypointCount());
    DEBUG_SERIAL.print(F(" wireless:"));
    DEBUG_SERIAL.println(wireless.isConnected() ? F("OK") : F("OFFLINE"));
  }

  // Wireless connection state change log
  static bool lastWirelessConnected = false;
  bool nowConnected = wireless.isConnected();
  if (nowConnected != lastWirelessConnected) {
    DEBUG_SERIAL.print(F("# Wireless ("));
    DEBUG_SERIAL.print(wireless.getProtocolName());
    DEBUG_SERIAL.print(F(") "));
    DEBUG_SERIAL.println(nowConnected ? F("CONNECTED") : F("DISCONNECTED"));
    
    // Send handshake when:
    // - Wireless control manually engaged (wirelessControlActive), OR
    // - Pi is inactive (no I2C handshake or 30s timeout)
    bool canSendHandshake = wirelessControlActive;
    if (!canSendHandshake) {
      #if defined(WIRELESS_PROTOCOL_CC1101)
        bool piInactive = !i2cHandshakeComplete || (millis() - lastI2CActivityMs > 30000);
        canSendHandshake = nowConnected && piInactive && !i2cBusy && (millis() - lastI2C) > 100;
      #else
        canSendHandshake = nowConnected;
      #endif
    }
    
    if (canSendHandshake) {
      sendWirelessReady();
      wireless.sendString("HELLO,MEGA_ROBOT");
    }
    
    lastWirelessConnected = nowConnected;
  }

  if (manualOverride && (millis() - lastManualCommand > MANUAL_TIMEOUT)) {
    exitManualMode(true);
  }

  if (navigationActive && (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT)) {
    DEBUG_SERIAL.println(F("# Warning: heartbeat timeout"));
  }
}

// ========================== I2C HANDLING ===================================
// IMPORTANT: These callbacks execute when Pi reads/writes to Arduino (slave mode)
// DO NOT perform I2C master transactions (compass reads) inside these callbacks
// All compass communication happens in main loop only
void onI2CReceive(int bytes) {
  i2cCallbackActive = true;
  lastI2CActivityMs = millis();
  if (bytes <= 0) {
    i2cCallbackActive = false;
    return;
  }

  uint8_t command = Wire.read();
  uint8_t length = bytes - 1;
  if (length > 32) {
    length = 32;
  }

  for (uint8_t i = 0; i < length; i++) {
    if (Wire.available()) {
      i2cPendingPayload[i] = Wire.read();
    } else {
      i2cPendingPayload[i] = 0;
    }
  }

  i2cPendingCommand = command;
  i2cPendingLength = length;
  i2cCommandPending = true;
  i2cCallbackActive = false;
}

void onI2CRequest() {
  i2cCallbackActive = true;
  lastI2CActivityMs = millis();
  // Guard against invalid buffer/length
  if (responseLength > 0 && responseLength <= sizeof(responseBuffer)) {
    Wire.write(responseBuffer, responseLength);
  } else {
    // Send a single 0 byte to avoid bus hang if length invalid
    uint8_t zero = 0;
    Wire.write(&zero, 1);
  }
  i2cCallbackActive = false;
}

void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length) {
  // Debug: log incoming command and length
  DEBUG_SERIAL.print(F("# I2C cmd: 0x"));
  DEBUG_SERIAL.print(command, HEX);
  DEBUG_SERIAL.print(F(" len="));
  DEBUG_SERIAL.println(length);
  switch (command) {
    case CMD_PING:
      markI2CHandshake();
      prepareAck();
      break;

    case CMD_NAV_START:
      if (navigation.getWaypointCount() > 0) {
        navigationActive = true;
        navigation.start();
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
        sendWirelessStatus();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;

    case CMD_NAV_PAUSE:
      navigation.pause();
      navigationActive = false;
      motors.stop();
      prepareAck();
      break;

    case CMD_NAV_RESUME:
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigation.resume();
        navigationActive = true;
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
        sendWirelessStatus();
      } else {
        prepareError(ERR_NO_WAYPOINTS);
      }
      break;

    case CMD_WAYPOINT_CLEAR:
      navigation.clearWaypoints();
      resetPendingWaypoints();
      navigationActive = false;
      manualOverride = false;
      motors.stop();
      prepareAck();
      sendWirelessStatus();
      break;

    case CMD_WAYPOINT_PACKET:
      // Avoid struct padding issues: parse fields explicitly (LE)
      if (length >= 11) {
        WaypointPacket packet;
        packet.id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        packet.seq = payload[2];
        memcpy(&packet.latitude, &payload[3], sizeof(float));
        memcpy(&packet.longitude, &payload[7], sizeof(float));
        storePendingWaypoint(packet);
        DEBUG_SERIAL.print(F("# Waypoint recv id="));
        DEBUG_SERIAL.print(packet.id);
        DEBUG_SERIAL.print(F(" seq="));
        DEBUG_SERIAL.print(packet.seq);
        DEBUG_SERIAL.print(F(" lat="));
        DEBUG_SERIAL.print(packet.latitude, 6);
        DEBUG_SERIAL.print(F(" lon="));
        DEBUG_SERIAL.println(packet.longitude, 6);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_WAYPOINT_COMMIT:
      commitPendingWaypoints();
      prepareAck();
      sendWirelessStatus();
      break;

    case CMD_NAV_STOP:
      navigation.stop();
      navigationActive = false;
      manualOverride = false;
      motors.stop();
      controlMode = MODE_AUTO;
      prepareAck();
      sendWirelessStatus();
      break;

    case CMD_REQUEST_GPS:
      DEBUG_SERIAL.println(F("# I2C: GPS request"));
      prepareGpsResponse();
      break;

    case CMD_REQUEST_STATUS:
      DEBUG_SERIAL.println(F("# I2C: STATUS request"));
      prepareStatusResponse();
      break;

    case CMD_HEARTBEAT:
      lastHeartbeat = millis();
      DEBUG_SERIAL.println(F("# I2C: HEARTBEAT"));
      prepareAck();
      break;

    case CMD_SET_AUTO_SPEED:
      // Payload: [speed:1] in 0..255 PWM
      if (length >= 1) {
        int speed = (int)payload[0];
        motors.setAutoBaseSpeed(speed);
        DEBUG_SERIAL.print(F("# Auto speed set to "));
        DEBUG_SERIAL.println(motors.getAutoBaseSpeed());
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    // === NEW COMMANDS FOR ENHANCED FEATURES ===
    
    case CMD_SEND_GPS:
      // Pi sending GPS data as fallback (for SIM7600E or when Neo-6M unavailable)
      if (length >= 16) {
        float lat, lon, speed, heading;
        memcpy(&lat, &payload[0], sizeof(float));
        memcpy(&lon, &payload[4], sizeof(float));
        memcpy(&speed, &payload[8], sizeof(float));
        memcpy(&heading, &payload[12], sizeof(float));
        
        // Update navigation with Pi's GPS data
        navigation.updateGpsData(lat, lon, speed, heading);
        DEBUG_SERIAL.print(F("# I2C: Pi GPS lat="));
        DEBUG_SERIAL.print(lat, 6);
        DEBUG_SERIAL.print(F(" lon="));
        DEBUG_SERIAL.print(lon, 6);
        DEBUG_SERIAL.print(F(" spd="));
        DEBUG_SERIAL.print(speed, 2);
        DEBUG_SERIAL.print(F(" hdg="));
        DEBUG_SERIAL.println(heading, 1);
        
        // If compass is not responding, fallback to GPS-only navigation
        if (!compass.isValid()) {
          navigation.fallbackToGpsOnly();
          DEBUG_SERIAL.println(F("# Navigation: Compass invalid, switched to GPS-only mode"));
          beepPattern(3, 100, 100);  // Triple beep alert
        }
        
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_SEND_HEADING:
      // Pi sending compass heading
      if (length >= 4) {
        memcpy(&piHeading, &payload[0], sizeof(float));
        DEBUG_SERIAL.print(F("# I2C: Pi heading "));
        DEBUG_SERIAL.println(piHeading, 1);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    
    case CMD_RETURN_TO_START:
      // Navigate back to starting position
      if (navigationActive) {
        navigation.returnToStart();
        DEBUG_SERIAL.println(F("# Navigation: RETURN command received, navigating to start"));
        beepPattern(1, 200, 0);  // Single long beep
        prepareAck();
      } else {
        prepareError(0xFD);  // Not in navigation mode
      }
      break;
    
    case CMD_MANUAL_OVERRIDE:
      // Manual joystick control with override flag
      if (length >= 3) {
        int8_t leftMotor = (int8_t)payload[0];
        int8_t rightMotor = (int8_t)payload[1];
        bool joystickActive = payload[2] != 0;
        
        // Immediately stop autonomous navigation if joystick touched
        if (joystickActive && navigationActive) {
          navigationActive = false;
          navigation.pause();
          DEBUG_SERIAL.println(F("# Joystick detected! Autonomous nav paused"));
        }
        
        // If an obstacle is detected within 30cm in front, do not allow forward
        // motion; stop and wait for a safe direction (reverse/turn).
        const bool frontObstacle = frontUltrasonicObstacle30cm();
        const bool isForwardCmd = (leftMotor > 0 && rightMotor > 0);
        if (joystickActive && frontObstacle && isForwardCmd) {
          motors.stop();
          leftMotor = 0;
          rightMotor = 0;
        } else {
          // Set motors directly
          motors.setMotors((int)leftMotor, (int)rightMotor);
        }
        manualOverride = true;
        controlMode = MODE_MANUAL;
        lastManualCommand = millis();

        lastManualLeft = leftMotor;
        lastManualRight = rightMotor;
        
        // Broadcast position via wireless immediately (for backup logging)
        if (wireless.isConnected() && gps.isValid()) {
          sendWirelessGps();
        }
        
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;
    
    case CMD_EMERGENCY_STOP:
      // Stop everything immediately
      motors.stop();
      navigationActive = false;
      manualOverride = false;
      navigation.stop();
      beepPattern(5, 50, 50);  // Multiple rapid beeps - emergency alert
      DEBUG_SERIAL.println(F("# EMERGENCY STOP"));
      prepareAck();
      break;
    
    case CMD_WIRELESS_BROADCAST:
      // Broadcast position via wireless module
      if (length >= 16) {
        // Position data for broadcast
        sendWirelessGps();
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_FOLLOW_LINE:
      // Payload: [enable:1]
      if (length >= 1) {
        bool enable = payload[0] != 0;
        lineFollowEnabled = enable;
        digitalWrite(LINE_FOLLOWER_ENA, enable ? HIGH : LOW);
        if (enable) {
          navigationActive = false; // pause autonomous when line-following
          motors.stop();
          DEBUG_SERIAL.println(F("# Line follower ENABLED"));
        } else {
          DEBUG_SERIAL.println(F("# Line follower DISABLED"));
        }
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_REQUEST_OBSTACLE:
      // Respond with obstacle flag and distance
      responseBuffer[0] = RESP_OBSTACLE;
      {
        int dist = obstacleAvoid.getDistance();
        bool frontObstacle = (dist > 0 && dist < OBSTACLE_THRESHOLD);
        responseBuffer[1] = frontObstacle ? 1 : 0;
        if (dist < 0) dist = 0;  // Invalid distance -> 0
        responseBuffer[2] = (dist >> 8) & 0xFF;
        responseBuffer[3] = dist & 0xFF;
      }
      responseLength = 4;
      break;

    case CMD_SOUND_BUZZER:
      // Sound buzzer for specified duration (seconds)
      if (length >= 1) {
        int duration = payload[0];
        if (duration > 0 && duration <= 10) {  // Max 10 seconds
          for (int i = 0; i < duration; i++) {
            tone(BUZZER_PIN, BUZZER_FREQ);
            delay(1000);
            noTone(BUZZER_PIN);
            delay(100);  // Short pause between seconds
          }
        }
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    default:
      prepareError(0xFE);
      break;
  }
}

void prepareAck(uint8_t code) {
  responseBuffer[0] = RESP_ACK;
  responseBuffer[1] = code;
  responseLength = 2;
}

void prepareError(uint8_t code) {
  responseBuffer[0] = RESP_ERROR;
  responseBuffer[1] = code;
  responseLength = 2;
}

void prepareGpsResponse() {
  responseBuffer[0] = RESP_GPS;
  if (!gps.isValid()) {
    responseBuffer[1] = 0;
    responseLength = 2;
    return;
  }

  responseBuffer[1] = 1;
  float latitude = gps.getLatitude();
  float longitude = gps.getLongitude();
  float speed = gps.getSpeed();
  float heading = piHeading;  // Heading from Pi's compass
  uint8_t satellites = gps.getSatellites();

  memcpy(&responseBuffer[2], &latitude, sizeof(float));
  memcpy(&responseBuffer[6], &longitude, sizeof(float));
  memcpy(&responseBuffer[10], &speed, sizeof(float));
  memcpy(&responseBuffer[14], &heading, sizeof(float));
  responseBuffer[18] = satellites;
  responseLength = 19;
}

void prepareStatusResponse() {
  responseBuffer[0] = RESP_STATUS;
  responseBuffer[1] = (controlMode == MODE_AUTO) ? 0 : 1;
  responseBuffer[2] = navigationActive ? 1 : 0;
  responseBuffer[3] = manualOverride ? 1 : 0;
  responseBuffer[4] = navigation.getWaypointCount();
  responseBuffer[5] = readBatteryPercent();
  responseBuffer[6] = readSignalQuality();
  responseBuffer[7] = navigation.getCurrentWaypointIndex();  // Current waypoint being navigated to
  responseBuffer[8] = navigation.isWaypointJustCompleted() ? 1 : 0;  // Flag for recent completion
  
  // Clear the completion flag after reporting it
  if (navigation.isWaypointJustCompleted()) {
    navigation.clearWaypointCompletionFlag();
  }
  
  responseLength = 9;
}

// ========================== WAYPOINT HANDLING ==============================
void resetPendingWaypoints() {
  pendingWaypointCount = 0;
}

void storePendingWaypoint(const WaypointPacket& packet) {
  if (pendingWaypointCount >= 20) {
    return;
  }

  PendingWaypoint slot;
  slot.latitude = packet.latitude;
  slot.longitude = packet.longitude;
  slot.id = packet.id;
  slot.seq = packet.seq;

  if (pendingWaypointCount == 0) {
    DEBUG_SERIAL.println(F("# Waypoint buffer: storing"));
  }

  pendingWaypoints[pendingWaypointCount++] = slot;
}

void commitPendingWaypoints() {
  navigation.clearWaypoints();
  for (uint8_t i = 0; i < pendingWaypointCount; ++i) {
    navigation.addWaypoint(pendingWaypoints[i].latitude, pendingWaypoints[i].longitude, pendingWaypoints[i].id);
  }

  DEBUG_SERIAL.print(F("# Waypoint committed: "));
  DEBUG_SERIAL.println(navigation.getWaypointCount());
  resetPendingWaypoints();
}

// ========================== WIRELESS HANDLING ==============================
void handleWireless() {
  WirelessMessage msg;

  while (wireless.receive(msg)) {
    DEBUG_SERIAL.print(F("# Wireless RX type="));
    DEBUG_SERIAL.print(msg.type, HEX);
    DEBUG_SERIAL.print(F(" len="));
    DEBUG_SERIAL.println(msg.length);

    switch (msg.type) {
      case MSG_TYPE_COMMAND:
        if (msg.length >= 2) {
          processWirelessCommand(msg.data[0], msg.data[1]);
        }
        break;

      case MSG_TYPE_HANDSHAKE:
        if (msg.length >= 10 && memcmp(msg.data, "UNO_REMOTE", 10) == 0) {
          DEBUG_SERIAL.println(F("# UNO Remote handshake received"));
          wirelessHandshakeComplete = true;
          sendWirelessReady();
        }
        break;

      case MSG_TYPE_HEARTBEAT:
        // Just update connection status
        break;

      case MSG_TYPE_STATUS:
        // Status request
        sendWirelessStatus();
        break;

      default:
        DEBUG_SERIAL.print(F("# Unknown wireless message type: "));
        DEBUG_SERIAL.println(msg.type, HEX);
        break;
    }
  }
}

void processWirelessCommand(uint8_t command, uint8_t speed) {
  lastManualCommand = millis();
  lastWirelessCommand = millis();
  wirelessControlActive = true;  // UNO remote takes priority

  switch (command) {
    case WIRELESS_CMD_MOTOR_FORWARD:
      enterManualMode();
      motors.forward(speed);
      DEBUG_SERIAL.print(F("# Manual forward: "));
      DEBUG_SERIAL.println(speed);
      break;

    case WIRELESS_CMD_MOTOR_BACKWARD:
      enterManualMode();
      motors.backward(speed);
      DEBUG_SERIAL.print(F("# Manual backward: "));
      DEBUG_SERIAL.println(speed);
      break;

    case WIRELESS_CMD_MOTOR_LEFT:
      enterManualMode();
      motors.turnLeft(speed);
      DEBUG_SERIAL.print(F("# Manual left: "));
      DEBUG_SERIAL.println(speed);
      break;

    case WIRELESS_CMD_MOTOR_RIGHT:
      enterManualMode();
      motors.turnRight(speed);
      DEBUG_SERIAL.print(F("# Manual right: "));
      DEBUG_SERIAL.println(speed);
      break;

    case WIRELESS_CMD_MOTOR_STOP:
      enterManualMode();
      motors.stop();
      DEBUG_SERIAL.println(F("# Manual stop"));
      break;

    case WIRELESS_CMD_MODE_AUTO:
      controlMode = MODE_AUTO;
      manualOverride = false;
      DEBUG_SERIAL.println(F("# Mode: AUTO"));
      break;

    case WIRELESS_CMD_MODE_MANUAL:
      controlMode = MODE_MANUAL;
      manualOverride = true;
      DEBUG_SERIAL.println(F("# Mode: MANUAL"));
      break;

    default:
      DEBUG_SERIAL.print(F("# Unknown wireless command: "));
      DEBUG_SERIAL.println(command, HEX);
      break;
  }
}

void processWirelessMessage(const String& message) {
  if (message.length() == 0) {
    return;
  }

  String command = message;
  command.toUpperCase();

  lastManualCommand = millis();

  // Handshake / queries
  if (command == "PING") {
    // Remote is probing link health
    sendWirelessReady();
    sendWirelessStatus();
    return;
  }

  if (command == "STATUS?") {
    sendWirelessStatus();
    return;
  }

  if (command.startsWith("MCTL,FORWARD")) {
    enterManualMode();
    int speed = 200;
    int commaPos = command.indexOf(',', 5);
    if (commaPos > 0) {
      speed = command.substring(commaPos + 1).toInt();
    }
    motors.forward(speed);
    DEBUG_SERIAL.print(F("# Manual forward: "));
    DEBUG_SERIAL.println(speed);
  } 
  else if (command.startsWith("MCTL,BACKWARD")) {
    enterManualMode();
    int speed = 150;
    int commaPos = command.indexOf(',', 5);
    if (commaPos > 0) {
      speed = command.substring(commaPos + 1).toInt();
    }
    motors.backward(speed);
    DEBUG_SERIAL.print(F("# Manual backward: "));
    DEBUG_SERIAL.println(speed);
  } 
  else if (command.startsWith("MCTL,LEFT")) {
    enterManualMode();
    int speed = 180;
    int commaPos = command.indexOf(',', 5);
    if (commaPos > 0) {
      speed = command.substring(commaPos + 1).toInt();
    }
    motors.turnLeft(speed);
    DEBUG_SERIAL.print(F("# Manual left: "));
    DEBUG_SERIAL.println(speed);
  } 
  else if (command.startsWith("MCTL,RIGHT")) {
    enterManualMode();
    int speed = 180;
    int commaPos = command.indexOf(',', 5);
    if (commaPos > 0) {
      speed = command.substring(commaPos + 1).toInt();
    }
    motors.turnRight(speed);
    DEBUG_SERIAL.print(F("# Manual right: "));
    DEBUG_SERIAL.println(speed);
  } 
  else if (command.startsWith("MCTL,STOP")) {
    motors.stop();
    DEBUG_SERIAL.println(F("# Manual stop"));
  } 
  else if (command.startsWith("MCTL,MANUAL")) {
    enterManualMode();
    DEBUG_SERIAL.println(F("# Entering manual mode"));
  } 
  else if (command == "AUTO" || command == "RELEASE" || command == "MCTL,AUTO" || command == "MCTL,RELEASE") {
    exitManualMode(true);
    DEBUG_SERIAL.println(F("# Exiting manual mode"));
  }
  else if (command == "RETURN!" || command == "RETURN") {
    // Return to starting position
    if (navigationActive || gps.isValid()) {
      navigation.returnToStart();
      navigationActive = true;
      manualOverride = false;
      controlMode = MODE_AUTO;
      DEBUG_SERIAL.println(F("# RETURN command: Navigating back to start position"));
      beepPattern(1, 200, 0);
      sendWirelessStatus();
    } else {
      DEBUG_SERIAL.println(F("# ERROR: Cannot RETURN - no navigation data or GPS fix"));
    }
  }
}

void sendWirelessGps() {
  if (!wireless.isConnected()) {
    return;
  }

#ifdef WIRELESS_PROTOCOL_ZIGBEE
  // ZigBee (transparent UART): send human-readable CSV
  if (!gps.isValid()) {
    wireless.sendString("GPS,,,,,0");
    return;
  }

  char line[96];
  const float lat = gps.getLatitude();
  const float lon = gps.getLongitude();
  const float speed = gps.getSpeed();
  const float heading = compass.getHeading();
  const int sats = (int)gps.getSatellites();
  snprintf(line, sizeof(line), "GPS,%.6f,%.6f,%.2f,%.1f,%d", lat, lon, speed, heading, sats);
  wireless.sendString(line);
  return;
#endif

  WirelessMessage msg;
  msg.type = MSG_TYPE_GPS;

  if (!gps.isValid()) {
    msg.length = 1;
    msg.data[0] = 0;  // Invalid flag
  } else {
    // Format: [valid:1][lat:4][lon:4][speed:4][heading:4][sats:1]
    msg.data[0] = 1;  // Valid flag
    float lat = gps.getLatitude();
    float lon = gps.getLongitude();
    float speed = gps.getSpeed();
    float heading = compass.getHeading();
    
    memcpy(&msg.data[1], &lat, 4);
    memcpy(&msg.data[5], &lon, 4);
    memcpy(&msg.data[9], &speed, 4);
    memcpy(&msg.data[13], &heading, 4);
    msg.data[17] = gps.getSatellites();
    msg.length = 18;
  }

  wireless.send(msg);
}

void sendWirelessStatus() {
  if (!wireless.isConnected()) {
    return;
  }

#ifdef WIRELESS_PROTOCOL_ZIGBEE
  // ZigBee (transparent UART): send STATUS,<mode>,<nav_state>
  const bool hasGps = gps.isValid();
  const bool navDone = navigation.isComplete();
  const bool navRun = navigationActive;

  const char* modeStr = (controlMode == MODE_MANUAL) ? "MANUAL" : "AUTO";
  const char* stateStr = navRun ? "RUN" : (navDone ? "DONE" : "IDLE");
  if (!hasGps && navRun && controlMode == MODE_AUTO) {
    modeStr = "NO_GPS";
    stateStr = "WAITING";
  }

  char line[48];
  snprintf(line, sizeof(line), "STATUS,%s,%s", modeStr, stateStr);
  wireless.sendString(line);
  return;
#endif

  WirelessMessage msg;
  msg.type = MSG_TYPE_STATUS;
  msg.data[0] = (controlMode == MODE_AUTO) ? 0 : 1;
  msg.data[1] = navigationActive ? 1 : 0;
  msg.data[2] = manualOverride ? 1 : 0;
  msg.data[3] = navigation.getWaypointCount();
  msg.data[4] = readBatteryPercent();
  msg.data[5] = wireless.getRSSI() & 0xFF;
  msg.length = 6;

  wireless.send(msg);
}

void sendWirelessReady() {
  if (!wireless.isConnected()) {
    return;
  }

#ifdef WIRELESS_PROTOCOL_ZIGBEE
  // ZigBee (transparent UART): explicit READY keyword for remotes
  wireless.sendString("READY");
  wireless.sendString("HELLO,MEGA_ROBOT");
  markWirelessHandshake();
  return;
#endif

#ifdef WIRELESS_PROTOCOL_CC1101
  // CC1101: Send structured READY status message
  WirelessMessage msg;
  msg.type = MSG_TYPE_STATUS;
  msg.length = 5;
  memcpy(msg.data, "READY", 5);
  wireless.send(msg);
#else
  // Other protocols: structured handshake
  WirelessMessage msg;
  msg.type = MSG_TYPE_HANDSHAKE;
  msg.length = 0;
  wireless.send(msg);
#endif

  // Also emit a human-readable hello string for transparent receivers
  wireless.sendString("HELLO,MEGA_ROBOT");

  markWirelessHandshake();
}

void sendWirelessObstacleAlert(int distance) {
  if (!wireless.isConnected()) {
    return;
  }

#ifdef WIRELESS_PROTOCOL_ZIGBEE
  char line[40];
  snprintf(line, sizeof(line), "OBSTACLE,%d,FRONT", distance);
  wireless.sendString(line);
  return;
#endif

  WirelessMessage msg;
  msg.type = MSG_TYPE_OBSTACLE;
  msg.data[0] = (distance >> 8) & 0xFF;
  msg.data[1] = distance & 0xFF;
  msg.data[2] = 0;  // Direction: 0=FRONT, 1=LEFT, 2=RIGHT
  msg.length = 3;

  wireless.send(msg);
}

// ========================== MANUAL MODE HANDLING ============================
void enterManualMode() {
  if (!manualOverride) {
    manualOverride = true;
    navigationActive = false;
    controlMode = MODE_MANUAL;
    motors.stop();
    DEBUG_SERIAL.println(F("# Manual mode entered"));
    beepPattern(1, 200, 0);
  }
}

void exitManualMode(bool resumeAutonomous) {
  if (manualOverride) {
    manualOverride = false;
    motors.stop();
    controlMode = resumeAutonomous ? MODE_AUTO : MODE_AUTO;
    DEBUG_SERIAL.println(F("# Manual mode exited"));
    
    if (resumeAutonomous && navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
      navigationActive = true;
      navigation.resume();
      DEBUG_SERIAL.println(F("# Resuming autonomous navigation"));
    }
  }
}

void processManualTimeout() {
  if (!manualOverride) {
    return;
  }

  if (millis() - lastManualCommand > MANUAL_TIMEOUT) {
    exitManualMode(true);
  }
}

// ========================== HANDSHAKE & STATUS =============================
void markI2CHandshake() {
  i2cHandshakeComplete = true;
  DEBUG_SERIAL.println(F("# I2C handshake complete"));
  beepPattern(2, 100, 100);
}

void markWirelessHandshake() {
  wirelessHandshakeComplete = true;
  DEBUG_SERIAL.print(F("# Wireless handshake complete: "));
  DEBUG_SERIAL.println(wireless.getProtocolName());
  beepPattern(2, 100, 100);
}

void checkReadyTone() {
  if (i2cHandshakeComplete && wirelessHandshakeComplete) {
    beepPattern(4, 100, 100);
  }
}

// ========================== UTILITY FUNCTIONS ===============================
uint8_t readBatteryPercent() {
  // Placeholder: would read actual battery level from ADC
  return 85;
}

uint8_t readSignalQuality() {
  // Return RSSI if available from wireless module
  int8_t rssi = wireless.getRSSI();
  if (rssi == 0) return 0;
  // Convert RSSI to 0-100 quality
  // Typical range: -30 (excellent) to -100 (poor)
  if (rssi > -50) return 100;
  if (rssi < -90) return 10;
  return (rssi + 100) * 2;
}

void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < pulses; i++) {
    tone(BUZZER_PIN, 2000);
    delay(onMs);
    noTone(BUZZER_PIN);
    if (i < pulses - 1) {
      delay(offMs);
    }
  }
}
