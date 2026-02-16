

/*
 * Environmental Monitoring Robot — Navigation Controller (Arduino Mega 2560)
 *
 * =========================================================================
 *  STATE-MACHINE ARCHITECTURE  (blueprint v2 — 3-state)
 * =========================================================================
 *
 *  Three mutually-exclusive states — only ONE owner drives the motors:
 *
 *    STATE_I2C       (AUTO)     – Raspberry Pi owns the motors
 *    STATE_WIRELESS             – ESP8266 CC1101 remote owns the motors
 *    STATE_FAILSAFE             – Both comms lost, motors halted, idle
 *
 *  Transitions (centralized Mode Manager, top of loop):
 *    Wireless alive?                      → STATE_WIRELESS  (always wins)
 *    No wireless, Pi alive?               → STATE_I2C
 *    Both wireless AND Pi lost?           → STATE_FAILSAFE
 *    Either channel recovers from failsafe? → back to appropriate state
 *
 *  Rules:
 *    • Wireless (manual) ALWAYS takes priority over Pi.
 *    • Sensors, buzzer, GPS run in EVERY state.
 *    • I2C ISRs are byte-copy only (< 50 µs).  Heavy work is deferred.
 *    • Every SPI call (CC1101) is wrapped in noInterrupts()/interrupts().
 *    • CC1101 polled at most once per CC1101_POLL_INTERVAL ms.
 *    • No delay(), no pulseIn(), no while(wireless.receive()) loops.
 *    • Motors are stopped on every state transition.
 *
 *  Superloop order:
 *    1. Deferred CC1101 init
 *    2. SPI Service (poll CC1101 → update WirelessBuffer)
 *    3. I²C Service (process deferred ISR command)
 *    4. MODE MANAGER (decide AUTO / WIRELESS / FAILSAFE)
 *    5. Sensor Tasks (GPS, ultrasonic)
 *    6. Buzzer tick
 *    7. I²C bus watchdog
 *    8. Decision Layer (act on current mode)
 *    9. Obstacle buzzer
 *   10. Status print
 *   11. GPS broadcast
 * =========================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <SPI.h>

#include "gps_handler.h"
#include "navigation.h"
#include "motor_control.h"
#include "obstacle_avoidance.h"
#include "globals.h"

// ===================== CC1101 wireless driver =====================
#include "cc1101_driver.h"
CC1101Driver wireless;
#include "wireless_interface.h"

// ===================== Components =====================
GPSHandler gps;
Navigation navigation;
MotorControl motors;
ObstacleAvoidance obstacleAvoid;

// ===================== State machine =====================
RobotState robotState = STATE_I2C;            // boot → I2C

// ===================== Motor shadow (for obstacle guard) =====================
static int8_t lastManualLeft  = 0;
static int8_t lastManualRight = 0;

// ===================== Shared variables =====================
uint8_t responseBuffer[32];
uint8_t responseLength = 0;

PendingWaypoint pendingWaypoints[MAX_WAYPOINTS];
uint8_t pendingWaypointCount = 0;

bool navigationActive        = false;
bool manualOverride          = false;
bool i2cHandshakeComplete    = false;
bool wirelessHandshakeComplete = false;

// I2C ISR → main-loop deferred command
volatile bool     i2cCommandPending   = false;
volatile uint8_t  i2cPendingCommand   = 0;
volatile uint8_t  i2cPendingLength    = 0;
uint8_t           i2cPendingPayload[32];

// Volatile timestamps touched in ISR
volatile unsigned long lastI2CActivityMs = 0;

// Main-loop timestamps
unsigned long lastStatusUpdate   = 0;
unsigned long lastWirelessGps    = 0;
unsigned long lastManualCommand  = 0;
unsigned long lastHeartbeat      = 0;
unsigned long lastCC1101Poll     = 0;

int manualSpeed = 180;
ControlMode controlMode = MODE_AUTO;

// Heading from Pi
float piHeading = 0.0;

// ===================== Non-blocking buzzer =====================
static uint8_t  buzzerPulsesLeft = 0;
static uint16_t buzzerOnMs       = 0;
static uint16_t buzzerOffMs      = 0;
static bool     buzzerToneOn     = false;
static unsigned long buzzerPhaseStart = 0;

// ===================== Forward declarations =====================
void onI2CReceive(int bytes);
void onI2CRequest();
void handleI2CCommand(uint8_t cmd, const uint8_t* payload, uint8_t len);
void prepareAck(uint8_t code = 0);
void prepareError(uint8_t code);
void prepareGpsResponse();
void prepareStatusResponse();
void resetPendingWaypoints();
void storePendingWaypoint(const WaypointPacket& pkt);
void commitPendingWaypoints();

void transitionToState(RobotState newState);
void pollCC1101();
void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags);
void processWirelessCommand(uint8_t cmd, uint8_t speed);
void processWirelessMessage(const String& msg);

void sendWirelessGps();
void sendWirelessStatus();
void sendWirelessReady();
void sendWirelessObstacleAlert(int distance);

void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void updateBuzzer();

uint8_t readBatteryPercent();
uint8_t readSignalQuality();

void recoverI2CBus();
bool isI2CBusStuck();
void i2cReinitSlave();

static inline bool frontObstacle() {
  const int d = obstacleAvoid.getDistance();
  return (d > 0 && d < OBSTACLE_THRESHOLD);
}

// ======================== LEGACY WRAPPERS ========================
void handleZigbee()                             {}
void processZigbeeMessage(const String& m)      { processWirelessMessage(m); }
void sendZigbeeGps()                            { sendWirelessGps(); }
void sendZigbeeStatus()                         { sendWirelessStatus(); }
void sendZigbeeReady()                          { sendWirelessReady(); }
void markZigbeeHandshake()                      { wirelessHandshakeComplete = true; }
void markI2CHandshake() {
  i2cHandshakeComplete = true;
  DEBUG_SERIAL.println(F("# I2C handshake complete"));
  beepPatternNB(2, 100, 100);
}
void markWirelessHandshake() {
  wirelessHandshakeComplete = true;
  DEBUG_SERIAL.println(F("# Wireless handshake complete"));
  beepPatternNB(2, 100, 100);
}

// ======================== I2C BUS RECOVERY ========================
void recoverI2CBus() {
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(50);
  if (digitalRead(SDA) == LOW) {
    for (uint8_t i = 0; i < 9; i++) {
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }
  }
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

bool isI2CBusStuck() {
  // Read the TWI status register directly — NEVER call pinMode() on
  // SDA/SCL while the Wire library owns them, as that detaches TWI
  // from the pins and corrupts any in-flight transaction.
  uint8_t twsr = TWSR & 0xF8;   // mask prescaler bits
  // 0xF8 = "no relevant state" = bus is idle and healthy
  // 0x00 = bus error (SDA/SCL stuck)
  if (twsr == 0x00) return true;
  // Additionally, if the TWI is stuck in an unexpected slave state
  // for too long, consider it hung.  0xF8 (idle) and 0x60/0x80/0xA8
  // (normal slave ops) are fine; anything else is suspicious.
  if (twsr != 0xF8 && twsr != 0x60 && twsr != 0x80 && twsr != 0x88 &&
      twsr != 0xA8 && twsr != 0xB8 && twsr != 0xC0 && twsr != 0xC8) {
    return true;
  }
  return false;
}

void i2cReinitSlave() {
  Wire.end();
  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  DEBUG_SERIAL.println(F("# I2C: bus recovered, slave reinit"));
}

// ======================== SETUP ========================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL) { delayMicroseconds(100); }

  DEBUG_SERIAL.println(F("\n# ========== ARDUINO MEGA NAVIGATION CONTROLLER =========="));
  DEBUG_SERIAL.println(F("# Architecture: STATE_I2C / STATE_WIRELESS finite state machine"));

  GPS_SERIAL.begin(GPS_BAUD);

  // I2C slave init
  recoverI2CBus();
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  DEBUG_SERIAL.print(F("# I2C slave @ 0x"));
  DEBUG_SERIAL.println(I2C_ADDRESS, HEX);

  if (gps.begin(GPS_SERIAL)) {
    DEBUG_SERIAL.println(F("# GPS initialized"));
  } else {
    DEBUG_SERIAL.println(F("# WARNING: GPS init failed"));
    beepPatternNB(3, 200, 100);
  }

  DEBUG_SERIAL.println(F("# Compass: provided by Pi via I2C (CMD_SEND_HEADING)"));

  motors.begin();
  obstacleAvoid.begin();
  navigation.begin(&gps, &motors, &obstacleAvoid);

  // CC1101 init is deferred to loop (after I2C is stable)
  DEBUG_SERIAL.println(F("# CC1101: deferred init (5 s)"));

  // Startup tone — non-blocking: schedule 3 s buzz
  tone(BUZZER_PIN, BUZZER_FREQ);
  delay(3000);                       // one-time acceptable at boot only
  noTone(BUZZER_PIN);

  DEBUG_SERIAL.println(F("# Boot complete — entering main loop"));
}

// ======================== MAIN LOOP ========================
void loop() {
  const unsigned long now = millis();

  // ---------- 1. Deferred CC1101 init (once, 5 s after boot) ----------
  static bool cc1101Ready = false;
  if (!cc1101Ready && now > 5000) {
    cc1101Ready = true;
    DEBUG_SERIAL.println(F("# CC1101: init..."));
    if (wireless.begin()) {
      DEBUG_SERIAL.println(F("# CC1101: ready (RX)"));
    } else {
      DEBUG_SERIAL.println(F("# CC1101: FAILED"));
    }
  }

  // ====================================================================
  //  2. SPI SERVICE — Poll CC1101 (non-blocking, time-sliced)
  //     Updates WirelessBuffer (does NOT drive motors here).
  // ====================================================================
  if (cc1101Ready && (now - lastCC1101Poll >= CC1101_POLL_INTERVAL)) {
    lastCC1101Poll = now;
    pollCC1101();
  }

  // ---- CC1101 RX health check (every 2 seconds) ----
  // If the CC1101 drifted out of RX mode (noise, FIFO overflow, SPI
  // glitch), this forces it back.  Costs ~20 µs, no freeze risk.
  {
    static unsigned long lastRxCheck = 0;
    if (cc1101Ready && (now - lastRxCheck >= 2000)) {
      lastRxCheck = now;
      wireless.ensureRxMode();
    }
  }

  // ====================================================================
  //  3. I²C SERVICE — Process deferred command from ISR
  //     ISR only set a flag + byte-copied the payload.
  // ====================================================================
  if (i2cCommandPending) {
    uint8_t cmd, len;
    uint8_t payload[32];

    noInterrupts();
    cmd = i2cPendingCommand;
    len = i2cPendingLength;
    if (len > 32) len = 32;
    memcpy(payload, (const void*)i2cPendingPayload, len);
    i2cCommandPending = false;
    interrupts();

    handleI2CCommand(cmd, payload, len);
  }

  // ====================================================================
  //  4. MODE MANAGER — Centralized state transition logic (TOP PRIORITY)
  //     This is the ONLY place state transitions are decided at runtime.
  //     Rules:
  //       • Valid CC1101 packets recently? → WIRELESS (always wins)
  //       • No wireless but Pi active?     → AUTO (I2C)
  //       • Both lost?                     → FAILSAFE
  // ====================================================================
  {
    const bool wirelessAlive = cc1101Ready &&
                               wireless.getLastRxTime() > 0 &&
                               (now - wireless.getLastRxTime() <= WIRELESS_LINK_TIMEOUT);
    const bool piAlive       = i2cHandshakeComplete &&
                               (now - lastI2CActivityMs <= I2C_LINK_TIMEOUT);

    if (wirelessAlive) {
      // Wireless ALWAYS takes priority
      if (robotState != STATE_WIRELESS) {
        transitionToState(STATE_WIRELESS);
      }
    } else if (piAlive) {
      // Pi is alive, no wireless → AUTO / I2C
      if (robotState == STATE_WIRELESS) {
        DEBUG_SERIAL.println(F("# Wireless link lost → STATE_I2C"));
        transitionToState(STATE_I2C);
      } else if (robotState == STATE_FAILSAFE) {
        DEBUG_SERIAL.println(F("# Pi reconnected → STATE_I2C"));
        transitionToState(STATE_I2C);
      }
    } else if (i2cHandshakeComplete || wireless.getLastRxTime() > 0) {
      // Both channels were active at some point but BOTH are now lost
      if (robotState != STATE_FAILSAFE) {
        DEBUG_SERIAL.println(F("# FAILSAFE: both Pi and wireless lost"));
        transitionToState(STATE_FAILSAFE);
      }
    }
    // else: never had any connection yet → stay in boot state (STATE_I2C)
  }

  // ====================================================================
  //  5. SENSOR TASKS — Always run, every state
  // ====================================================================
  gps.update();                     // UART — non-blocking
  obstacleAvoid.update();           // non-blocking ultrasonic
  // Compass heading is received from Pi via CMD_SEND_HEADING → piHeading

  // ====================================================================
  //  6. Non-blocking buzzer tick
  // ====================================================================
  updateBuzzer();

  // ====================================================================
  //  7. I²C bus watchdog
  // ====================================================================
  if (i2cHandshakeComplete) {
    static unsigned long lastWatchdog = 0;
    if (now - lastWatchdog > 5000) {
      lastWatchdog = now;
      if (isI2CBusStuck()) {
        i2cReinitSlave();
      }
    }
  }

  // ====================================================================
  //  8. DECISION LAYER — Act on the current state
  //     Mode was already decided by the Mode Manager above.
  //     Each state reads its own buffer and drives motors accordingly.
  // ====================================================================
  switch (robotState) {

    case STATE_I2C:
      // Pi / autonomous navigation owns motors
      if (!manualOverride && navigationActive) {
        if (gps.isValid()) {
          navigation.update();
          if (navigation.isComplete()) {
            navigationActive = false;
            motors.stop();
            DEBUG_SERIAL.println(F("# Nav complete"));
          }
        } else {
          static unsigned long lastGpsWarn = 0;
          if (now - lastGpsWarn > 5000) {
            DEBUG_SERIAL.println(F("# WARNING: no GPS fix"));
            motors.stop();
            lastGpsWarn = now;
          }
        }
      }

      // Manual timeout (Pi joystick)
      if (manualOverride && (now - lastManualCommand > MANUAL_TIMEOUT)) {
        manualOverride = false;
        motors.stop();
        controlMode = MODE_AUTO;
        if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
          navigationActive = true;
          navigation.resume();
        }
        DEBUG_SERIAL.println(F("# Pi manual timeout → AUTO"));
      }
      break;

    case STATE_WIRELESS:
      // ESP8266 remote owns motors.
      // Obstacle safety: if last command was forward and obstacle ahead, stop.
      if (frontObstacle() && lastManualLeft > 0 && lastManualRight > 0) {
        motors.stop();
        lastManualLeft = 0;
        lastManualRight = 0;
      }
      // Link-loss is handled by Mode Manager above — no duplicate check here.
      break;

    case STATE_FAILSAFE:
      // Both communication channels lost.
      // Motors MUST stay stopped.  Slow beep to indicate safe-idle.
      motors.stop();
      {
        static unsigned long lastFailsafeBeep = 0;
        if (now - lastFailsafeBeep >= FAILSAFE_BEEP_INTERVAL) {
          beepPatternNB(2, 300, 200);
          lastFailsafeBeep = now;
        }
      }
      break;
  }

  // ====================================================================
  //  9. Obstacle buzzer alert (any state)
  // ====================================================================
  if (frontObstacle()) {
    static unsigned long lastObstacleBeep = 0;
    if (now - lastObstacleBeep > 2000) {
      beepPatternNB(3, 100, 80);
      lastObstacleBeep = now;
    }
  }

  // ====================================================================
  // 10. Status print
  // ====================================================================
  if (now - lastStatusUpdate >= STATUS_INTERVAL) {
    lastStatusUpdate = now;
    DEBUG_SERIAL.print(F("# ST="));
    switch (robotState) {
      case STATE_I2C:      DEBUG_SERIAL.print(F("AUTO"));     break;
      case STATE_WIRELESS: DEBUG_SERIAL.print(F("WIRELESS")); break;
      case STATE_FAILSAFE: DEBUG_SERIAL.print(F("FAILSAFE")); break;
    }
    DEBUG_SERIAL.print(F(" mode="));
    DEBUG_SERIAL.print(controlMode == MODE_AUTO ? F("A") : F("M"));
    DEBUG_SERIAL.print(F(" nav="));
    DEBUG_SERIAL.print(navigationActive ? F("RUN") : F("IDLE"));
    DEBUG_SERIAL.print(F(" wp="));
    DEBUG_SERIAL.print(navigation.getWaypointCount());
    DEBUG_SERIAL.print(F(" cc="));
    DEBUG_SERIAL.print(wireless.isConnected() ? F("ON") : F("--"));
    // RX diagnostics: good/bad packet counts and time since last packet
    DEBUG_SERIAL.print(F(" rx="));
    DEBUG_SERIAL.print(wireless.getRxGoodCount());
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(wireless.getRxBadCount());
    if (wireless.getLastRxTime() > 0) {
      DEBUG_SERIAL.print(F(" ago="));
      DEBUG_SERIAL.print((now - wireless.getLastRxTime()) / 1000);
      DEBUG_SERIAL.print(F("s"));
    }
    DEBUG_SERIAL.println();
  }

  // ====================================================================
  // 11. GPS broadcast (wireless) — DISABLED: ESP8266 remote is TX-only.
  //     Sending takes CC1101 out of RX and drops incoming packets.
  // ====================================================================
}

// ======================== STATE TRANSITIONS ========================
void transitionToState(RobotState newState) {
  if (newState == robotState) return;

  // Leave old state
  motors.stop();
  lastManualLeft  = 0;
  lastManualRight = 0;

  switch (robotState) {
    case STATE_WIRELESS:
      manualOverride = false;
      controlMode = MODE_AUTO;
      // Resume autonomous if waypoints remain
      if (navigation.getWaypointCount() > 0 && !navigation.isComplete()) {
        navigationActive = true;
        navigation.resume();
        DEBUG_SERIAL.println(F("# Resuming autonomous nav"));
      }
      break;
    case STATE_I2C:
      // Pause any running autonomous navigation
      if (navigationActive) {
        navigation.pause();
        navigationActive = false;
      }
      break;
    case STATE_FAILSAFE:
      // Leaving failsafe — nothing extra to clean up
      break;
  }

  // Enter new state
  robotState = newState;

  switch (newState) {
    case STATE_I2C:
      DEBUG_SERIAL.println(F("# → STATE_I2C (AUTO)"));
      break;
    case STATE_WIRELESS:
      manualOverride = true;
      controlMode = MODE_MANUAL;
      wirelessHandshakeComplete = true;
      DEBUG_SERIAL.println(F("# → STATE_WIRELESS"));
      beepPatternNB(1, 150, 0);
      break;
    case STATE_FAILSAFE:
      manualOverride = false;
      controlMode = MODE_AUTO;
      navigationActive = false;
      navigation.stop();
      DEBUG_SERIAL.println(F("# → STATE_FAILSAFE (motors halted)"));
      beepPatternNB(5, 100, 100);
      break;
  }
}

// ======================== CC1101 POLLING ========================
// Called once per CC1101_POLL_INTERVAL.  Reads at most ONE packet.
// Updates WirelessBuffer / timestamps only.  Mode Manager decides state.
void pollCC1101() {
  WirelessMessage msg;
  if (!wireless.receive(msg)) return;

  // Any valid packet updates lastRxTime (done inside cc1101_driver.cpp).
  // Mode Manager will see wirelessAlive == true on next loop iteration.

  switch (msg.type) {
    case MSG_TYPE_RAW_MOTOR:
      if (msg.length >= sizeof(RawMotorPacket)) {
        // If Mode Manager hasn't promoted us yet, the transition will
        // happen at the top of the NEXT loop.  Process the command only
        // if we're already in WIRELESS (or will be momentarily).
        if (robotState == STATE_WIRELESS || robotState == STATE_FAILSAFE || robotState == STATE_I2C) {
          // Store for immediate use — transitionToState() is handled by Mode Manager
          RawMotorPacket* pkt = (RawMotorPacket*)msg.data;
          if (robotState == STATE_WIRELESS) {
            processRawMotorCommand(pkt->throttle, pkt->steer, pkt->flags);
          }
          // If not yet in WIRELESS, Mode Manager will switch us next iteration,
          // and the next packet will drive the motors.
        }
      }
      break;

    case MSG_TYPE_COMMAND:
      if (msg.length >= 2 && robotState == STATE_WIRELESS) {
        processWirelessCommand(msg.data[0], msg.data[1]);
      }
      break;

    case MSG_TYPE_HANDSHAKE:
      wirelessHandshakeComplete = true;
      // Don't TX reply — ESP8266 is TX-only, reply disrupts RX
      break;

    case MSG_TYPE_HEARTBEAT:
      break;

    case MSG_TYPE_STATUS:
      // Don't TX reply — ESP8266 is TX-only
      break;

    default:
      break;
  }
}

// ======================== RAW MOTOR (ESP8266) ========================
void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags) {
  lastManualCommand = millis();

  // Arcade drive mixing
  int left  = constrain(throttle + steer, -255, 255);
  int right = constrain(throttle - steer, -255, 255);

  // Deadzone
  if (abs(left)  < 15) left  = 0;
  if (abs(right) < 15) right = 0;

  // Obstacle guard: block forward into obstacle
  if (frontObstacle() && left > 0 && right > 0) {
    left  = 0;
    right = 0;
    motors.stop();
  } else {
    motors.setMotors(left, right);
  }

  lastManualLeft  = (int8_t)constrain(left,  -127, 127);
  lastManualRight = (int8_t)constrain(right, -127, 127);

  // Throttled debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    DEBUG_SERIAL.print(F("# Raw L:"));
    DEBUG_SERIAL.print(left);
    DEBUG_SERIAL.print(F(" R:"));
    DEBUG_SERIAL.println(right);
    lastPrint = millis();
  }
}

// ======================== WIRELESS COMMAND (structured) ========================
void processWirelessCommand(uint8_t cmd, uint8_t speed) {
  lastManualCommand = millis();

  switch (cmd) {
    case WIRELESS_CMD_MOTOR_FORWARD:  motors.forward(speed);  break;
    case WIRELESS_CMD_MOTOR_BACKWARD: motors.backward(speed); break;
    case WIRELESS_CMD_MOTOR_LEFT:     motors.turnLeft(speed);  break;
    case WIRELESS_CMD_MOTOR_RIGHT:    motors.turnRight(speed); break;
    case WIRELESS_CMD_MOTOR_STOP:     motors.stop();           break;
    case WIRELESS_CMD_MODE_AUTO:
      transitionToState(STATE_I2C);
      break;
    default:
      break;
  }
}

// ======================== I2C ISR HANDLERS ========================
// RULE: byte-copy only, set flag, < 50 µs.
void onI2CReceive(int bytes) {
  lastI2CActivityMs = millis();
  if (bytes <= 0) return;

  uint8_t cmd = Wire.read();
  uint8_t len = bytes - 1;
  if (len > 32) len = 32;

  for (uint8_t i = 0; i < len; i++) {
    i2cPendingPayload[i] = Wire.available() ? Wire.read() : 0;
  }

  i2cPendingCommand = cmd;
  i2cPendingLength  = len;
  i2cCommandPending = true;
}

void onI2CRequest() {
  lastI2CActivityMs = millis();
  if (responseLength > 0 && responseLength <= sizeof(responseBuffer)) {
    Wire.write(responseBuffer, responseLength);
  } else {
    uint8_t z = 0;
    Wire.write(&z, 1);
  }
}

// ======================== I2C COMMAND HANDLER ========================
void handleI2CCommand(uint8_t command, const uint8_t* payload, uint8_t length) {
  DEBUG_SERIAL.print(F("# I2C cmd 0x"));
  DEBUG_SERIAL.print(command, HEX);
  DEBUG_SERIAL.print(F(" len="));
  DEBUG_SERIAL.println(length);

  // While wireless or failsafe owns the system, only allow read-only / buzzer commands from Pi
  if (robotState == STATE_WIRELESS || robotState == STATE_FAILSAFE) {
    switch (command) {
      case CMD_PING:
      case CMD_HEARTBEAT:
      case CMD_REQUEST_GPS:
      case CMD_REQUEST_STATUS:
      case CMD_REQUEST_OBSTACLE:
      case CMD_SOUND_BUZZER:
      case CMD_SET_AUTO_SPEED:
      case CMD_SEND_HEADING:     // compass data — read-only, always useful
      case CMD_EMERGENCY_STOP:   // always allow E-stop
        break;  // allowed
      default:
        prepareAck();
        return; // blocked
    }
  }

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
      break;

    case CMD_WAYPOINT_PACKET:
      if (length >= 19) {   // 2(id) + 1(seq) + 8(lat double) + 8(lon double)
        WaypointPacket pkt;
        pkt.id  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        pkt.seq = payload[2];
        memcpy(&pkt.latitude,  &payload[3], sizeof(double));
        memcpy(&pkt.longitude, &payload[11], sizeof(double));
        storePendingWaypoint(pkt);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_WAYPOINT_COMMIT:
      commitPendingWaypoints();
      prepareAck();
      break;

    case CMD_NAV_STOP:
      navigation.stop();
      navigationActive = false;
      manualOverride = false;
      motors.stop();
      controlMode = MODE_AUTO;
      prepareAck();
      break;

    case CMD_REQUEST_GPS:
      prepareGpsResponse();
      break;

    case CMD_REQUEST_STATUS:
      prepareStatusResponse();
      break;

    case CMD_HEARTBEAT:
      lastHeartbeat = millis();
      prepareAck();
      break;

    case CMD_SET_AUTO_SPEED:
      if (length >= 1) {
        motors.setAutoBaseSpeed((int)payload[0]);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_SEND_GPS:
      if (length >= 16) {
        float lat, lon, spd, hdg;
        memcpy(&lat, &payload[0],  sizeof(float));
        memcpy(&lon, &payload[4],  sizeof(float));
        memcpy(&spd, &payload[8],  sizeof(float));
        memcpy(&hdg, &payload[12], sizeof(float));
        navigation.updateGpsData(lat, lon, spd, hdg);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_SEND_HEADING:
      if (length >= 4) {
        memcpy(&piHeading, &payload[0], sizeof(float));
        navigation.setHeading(piHeading);
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_RETURN_TO_START:
      if (navigationActive || gps.isValid()) {
        navigation.returnToStart();
        navigationActive = true;
        manualOverride = false;
        controlMode = MODE_AUTO;
        prepareAck();
      } else {
        prepareError(0xFD);
      }
      break;

    case CMD_MANUAL_OVERRIDE:
      if (length >= 3) {
        int8_t leftMotor  = (int8_t)payload[0];
        int8_t rightMotor = (int8_t)payload[1];
        bool active       = payload[2] != 0;

        if (active && navigationActive) {
          navigationActive = false;
          navigation.pause();
        }

        if (active && frontObstacle() && leftMotor > 0 && rightMotor > 0) {
          motors.stop();
          leftMotor  = 0;
          rightMotor = 0;
        } else {
          motors.setMotors((int)leftMotor, (int)rightMotor);
        }
        manualOverride = true;
        controlMode = MODE_MANUAL;
        lastManualCommand = millis();
        lastManualLeft  = leftMotor;
        lastManualRight = rightMotor;
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_EMERGENCY_STOP:
      motors.stop();
      navigationActive = false;
      manualOverride = false;
      navigation.stop();
      beepPatternNB(5, 50, 50);
      DEBUG_SERIAL.println(F("# EMERGENCY STOP"));
      prepareAck();
      break;

    case CMD_WIRELESS_BROADCAST:
      sendWirelessGps();
      prepareAck();
      break;

    // CMD_FOLLOW_LINE removed — no line follower hardware

    case CMD_REQUEST_OBSTACLE: {
      int dist = obstacleAvoid.getDistance();
      bool obs = (dist > 0 && dist < OBSTACLE_THRESHOLD);
      responseBuffer[0] = RESP_OBSTACLE;
      responseBuffer[1] = obs ? 1 : 0;
      if (dist < 0) dist = 0;
      responseBuffer[2] = (dist >> 8) & 0xFF;
      responseBuffer[3] = dist & 0xFF;
      responseLength = 4;
      break;
    }

    case CMD_SOUND_BUZZER:
      if (length >= 1) {
        uint8_t secs = payload[0];
        if (secs > 0 && secs <= 10) {
          beepPatternNB(secs, 900, 100);
        }
        prepareAck();
      } else {
        prepareError(ERR_PACKET_SIZE);
      }
      break;

    case CMD_ENGAGE_WIRELESS:
      // Pi asking to engage/disengage wireless backup
      if (length >= 1) {
        if (payload[0]) {
          transitionToState(STATE_WIRELESS);
        } else {
          transitionToState(STATE_I2C);
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

// ======================== RESPONSE BUILDERS ========================
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
  float lat = gps.getLatitude();
  float lon = gps.getLongitude();
  float spd = gps.getSpeed();
  float hdg = piHeading;
  uint8_t sat = gps.getSatellites();
  memcpy(&responseBuffer[2],  &lat, 4);
  memcpy(&responseBuffer[6],  &lon, 4);
  memcpy(&responseBuffer[10], &spd, 4);
  memcpy(&responseBuffer[14], &hdg, 4);
  responseBuffer[18] = sat;
  responseLength = 19;
}

void prepareStatusResponse() {
  responseBuffer[0] = RESP_STATUS;
  responseBuffer[1] = (controlMode == MODE_AUTO) ? 0 : 1;
  responseBuffer[2] = navigationActive ? 1 : 0;
  responseBuffer[3] = manualOverride   ? 1 : 0;
  responseBuffer[4] = navigation.getWaypointCount();
  responseBuffer[5] = readBatteryPercent();
  responseBuffer[6] = readSignalQuality();
  responseBuffer[7] = navigation.getCurrentWaypointIndex();
  responseBuffer[8] = navigation.isWaypointJustCompleted() ? 1 : 0;
  if (navigation.isWaypointJustCompleted()) {
    navigation.clearWaypointCompletionFlag();
  }
  responseLength = 9;
}

// ======================== WAYPOINTS ========================
void resetPendingWaypoints() { pendingWaypointCount = 0; }

void storePendingWaypoint(const WaypointPacket& pkt) {
  if (pendingWaypointCount >= MAX_WAYPOINTS) return;
  PendingWaypoint slot;
  slot.latitude  = pkt.latitude;
  slot.longitude = pkt.longitude;
  slot.id  = pkt.id;
  slot.seq = pkt.seq;
  pendingWaypoints[pendingWaypointCount++] = slot;
}

void commitPendingWaypoints() {
  navigation.clearWaypoints();
  for (uint8_t i = 0; i < pendingWaypointCount; ++i) {
    navigation.addWaypoint(pendingWaypoints[i].latitude,
                           pendingWaypoints[i].longitude,
                           pendingWaypoints[i].id);
  }
  DEBUG_SERIAL.print(F("# Committed "));
  DEBUG_SERIAL.print(navigation.getWaypointCount());
  DEBUG_SERIAL.println(F(" waypoints"));
  resetPendingWaypoints();
}

// ======================== WIRELESS TX HELPERS ========================
// NOTE: ALL wireless.send() calls are DISABLED.
// The ESP8266 remote is TX-only (blind send) — it never receives.
// Calling wireless.send() takes the CC1101 out of RX mode (IDLE → TX → RX),
// which causes us to miss incoming packets and lose the wireless connection.
// These functions are kept as stubs for future bi-directional hardware.

void sendWirelessGps() {
  // DISABLED: ESP8266 is TX-only, cannot receive GPS data
  // wireless.send() would disrupt RX and drop incoming packets
}

void sendWirelessStatus() {
  // DISABLED: ESP8266 is TX-only, cannot receive status
  // wireless.send() would disrupt RX and drop incoming packets
}

void sendWirelessReady() {
  // ESP8266 remote is TX-only (blind send) — it never receives replies.
  // Do NOT call wireless.send() here as it takes CC1101 out of RX mode
  // and causes us to miss the next incoming packets from the remote.
  markWirelessHandshake();
}

void sendWirelessObstacleAlert(int distance) {
  // DISABLED: ESP8266 is TX-only, cannot receive obstacle alerts
  // wireless.send() would disrupt RX and drop incoming packets
  (void)distance;
}

// ======================== LEGACY STRING HANDLER ========================
void processWirelessMessage(const String& message) {
  if (message.length() == 0) return;
  lastManualCommand = millis();

  // Stack-based comparison — avoids heap allocation / fragmentation
  if (message.equalsIgnoreCase(F("PING"))) {
    sendWirelessReady();
    sendWirelessStatus();
  } else if (message.equalsIgnoreCase(F("STATUS?"))) {
    sendWirelessStatus();
  } else if (message.equalsIgnoreCase(F("RETURN!")) || message.equalsIgnoreCase(F("RETURN"))) {
    if (gps.isValid()) {
      navigation.returnToStart();
      transitionToState(STATE_I2C);
      navigationActive = true;
    }
  }
}

// ======================== NON-BLOCKING BUZZER ========================
void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  buzzerPulsesLeft = pulses;
  buzzerOnMs  = onMs;
  buzzerOffMs = offMs;
  buzzerToneOn = true;
  buzzerPhaseStart = millis();
  tone(BUZZER_PIN, BUZZER_FREQ);
}

void updateBuzzer() {
  if (buzzerPulsesLeft == 0) return;
  unsigned long elapsed = millis() - buzzerPhaseStart;

  if (buzzerToneOn) {
    if (elapsed >= buzzerOnMs) {
      noTone(BUZZER_PIN);
      buzzerToneOn = false;
      buzzerPhaseStart = millis();
      buzzerPulsesLeft--;
      if (buzzerPulsesLeft == 0) return; // done
    }
  } else {
    if (elapsed >= buzzerOffMs) {
      tone(BUZZER_PIN, BUZZER_FREQ);
      buzzerToneOn = true;
      buzzerPhaseStart = millis();
    }
  }
}

// ======================== UTILITY ========================
uint8_t readBatteryPercent() { return 85; }

uint8_t readSignalQuality() {
  int8_t rssi = wireless.getRSSI();
  if (rssi == 0) return 0;
  if (rssi > -50) return 100;
  if (rssi < -90) return 10;
  return (rssi + 100) * 2;
}

// Legacy blocking beep (only used if someone still calls it)
void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  beepPatternNB(pulses, onMs, offMs);
}
