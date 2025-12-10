/*
 * Arduino UNO Multi-Protocol Remote Controller (ZigBee / BLE / UART LoRa)
 * - Reads a 2-axis joystick (A0: X, A1: Y) and optional button (D4)
 * - Sends control frames over a selected wireless link (SoftwareSerial)
 * - Receives location frames from the robot and prints them to USB Serial
 * - Periodic HELLO heartbeat announces controller ID + protocol
 *
 * Frame formats (matches Mega wireless parser):
 *   Outgoing control:   "MCTL,<DIR>[,<SPEED>]" where DIR in {FORWARD,BACKWARD,LEFT,RIGHT,STOP}
 *   Heartbeat:          "HELLO,<deviceId>,<protocol>"
 *   Incoming GPS:       "GPS,<lat>,<lon>,<speed>,<heading>,<sats>,<iso8601>\n"
 *
 * Wiring:
 *   - ZigBee/XBee (3.3V logic!) via a level shifter or compatible shield
 *       ZigBee TX -> UNO pin 2  (SoftwareSerial RX)
 *       ZigBee RX -> UNO pin 3  (SoftwareSerial TX) [level shift recommended]
 *   - Joystick
 *       X axis -> A0
 *       Y axis -> A1
 *       SW (button) -> D4 (active LOW) with INPUT_PULLUP
 *   - Status LED -> built-in LED (pin 13)
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// ---------------- Protocol Selection ----------------
// Select exactly one link below.
#define USE_ZIGBEE 1
// #define USE_BLE 1
// #define USE_LORA 1

#if (defined(USE_ZIGBEE) + defined(USE_BLE) + defined(USE_LORA)) != 1
#error "Select exactly one protocol: USE_ZIGBEE, USE_BLE, or USE_LORA"
#endif

// ---------------- Configuration ----------------
static const uint8_t PIN_LINK_RX = 2;   // UNO receives on 2 (connect to module TX)
static const uint8_t PIN_LINK_TX = 3;   // UNO transmits on 3 (connect via level shift if needed)

#if defined(USE_ZIGBEE)
static const long LINK_BAUD = 57600;    // common for XBee in transparent mode
static const char* LINK_NAME = "zigbee";
#elif defined(USE_BLE)
static const long LINK_BAUD = 38400;    // HC-05 default in AT mode; adjust if your module uses 9600
static const char* LINK_NAME = "ble";
#elif defined(USE_LORA)
static const long LINK_BAUD = 9600;     // typical UART LoRa (e.g., E32) transparent serial
static const char* LINK_NAME = "lora";
#endif

static const uint8_t PIN_JOY_X = A0;
static const uint8_t PIN_JOY_Y = A1;
static const uint8_t PIN_JOY_SW = 4;      // optional pushbutton (active LOW)

static const char*   DEVICE_ID = "uno_controller_1";

// Joystick tuning
static const int JOY_CENTER = 512;         // ideal center (10-bit ADC)
static const int JOY_DEADBAND = 80;        // no movement within +/- deadband
static const unsigned long SEND_INTERVAL_MS = 120;   // throttle transmissions
static const unsigned long HEARTBEAT_MS    = 5000;   // HELLO interval

// ------------------------------------------------
SoftwareSerial linkSerial(PIN_LINK_RX, PIN_LINK_TX); // RX, TX

struct JoyState {
  int xRaw;
  int yRaw;
  int dx;    // centered delta
  int dy;    // centered delta
  const char* direction; // forward, reverse, left, right, stop
  int speed;             // 0..255
};

JoyState lastSent = {0,0,0,0,"stop", 0};
unsigned long lastSendAt = 0;
unsigned long lastHelloAt = 0;

// Utility: safe string send to the selected link
void sendLinkLine(const String& line) {
  linkSerial.print(line);
  linkSerial.print('\n');
}

// Map joystick deflection to direction/speed
JoyState computeJoy() {
  JoyState s;
  s.xRaw = analogRead(PIN_JOY_X);
  s.yRaw = analogRead(PIN_JOY_Y);
  s.dx = s.xRaw - JOY_CENTER;
  s.dy = s.yRaw - JOY_CENTER;

  // Apply deadband
  int ax = abs(s.dx);
  int ay = abs(s.dy);

  // Default
  s.direction = "stop";
  s.speed = 0;

  // If button pressed (active LOW) treat as stop
  bool buttonPressed = (digitalRead(PIN_JOY_SW) == LOW);
  if (buttonPressed) {
    return s; // stop
  }

  if (ax < JOY_DEADBAND && ay < JOY_DEADBAND) {
    return s; // stop
  }

  // Choose dominant axis
  if (ay >= ax) {
    // forward/reverse by Y axis (note: joystick forward often gives lower ADC value)
    if (s.dy < -JOY_DEADBAND) {
      s.direction = "forward";
      int magnitude = constrain(abs(s.dy) - JOY_DEADBAND, 0, 512 - JOY_DEADBAND);
      s.speed = map(magnitude, 0, 512 - JOY_DEADBAND, 0, 255);
    } else if (s.dy > JOY_DEADBAND) {
      s.direction = "reverse";
      int magnitude = constrain(abs(s.dy) - JOY_DEADBAND, 0, 512 - JOY_DEADBAND);
      s.speed = map(magnitude, 0, 512 - JOY_DEADBAND, 0, 255);
    }
  } else {
    // left/right by X axis
    if (s.dx < -JOY_DEADBAND) {
      s.direction = "left";
      int magnitude = constrain(abs(s.dx) - JOY_DEADBAND, 0, 512 - JOY_DEADBAND);
      s.speed = map(magnitude, 0, 512 - JOY_DEADBAND, 0, 255);
    } else if (s.dx > JOY_DEADBAND) {
      s.direction = "right";
      int magnitude = constrain(abs(s.dx) - JOY_DEADBAND, 0, 512 - JOY_DEADBAND);
      s.speed = map(magnitude, 0, 512 - JOY_DEADBAND, 0, 255);
    }
  }

  // Clamp
  s.speed = constrain(s.speed, 0, 255);
  return s;
}

bool joyChanged(const JoyState& a, const JoyState& b) {
  if (a.direction != b.direction) return true;
  // Consider speed changed if difference > 6 to avoid noise
  if (abs(a.speed - b.speed) > 6) return true;
  return false;
}

void maybeSendHeartbeat() {
  unsigned long now = millis();
  if (now - lastHelloAt >= HEARTBEAT_MS) {
    String line = String("HELLO,") + DEVICE_ID + "," + LINK_NAME;
    sendLinkLine(line);
    lastHelloAt = now;
  }
}

void sendJoystick(const JoyState& s) {
  // Match Mega wireless parser: expects "MCTL,<DIR>[,<SPEED>]"
  String line;
  if (String(s.direction) == "stop") {
    line = F("MCTL,STOP");
  } else {
    // Direction keywords: FORWARD, BACKWARD, LEFT, RIGHT
    String dir = String(s.direction);
    dir.toUpperCase();
    line = String("MCTL,") + dir + "," + String(s.speed);
  }
  sendLinkLine(line);
  lastSent = s;
  lastSendAt = millis();
}

// Read a line from the selected link (non-blocking), returns true if a full line is available
bool readLinkLine(String& out) {
  static String buf;
  while (linkSerial.available()) {
    char c = (char)linkSerial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        out = buf;
        buf = "";
        return true;
      }
    } else if (isPrintable(c) || c == ',' || c == '-' || c == '.' || c == ':' || c == 'T' || c == 'Z') {
      buf += c;
    }
  }
  return false;
}

void handleInboundFrame(const String& line) {
  // Minimal parsing: just echo GPS frames to USB Serial for operator awareness
  if (line.startsWith("GPS,")) {
    Serial.print(F("[GPS] "));
    Serial.println(line);
    // Optional: blink LED to indicate update
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
  } else if (line.startsWith("MODE,") || line.startsWith("HELLO")) {
    Serial.print(F("[ZB] "));
    Serial.println(line);
  } else {
    // Unknown/other
    Serial.print(F("[RX] "));
    Serial.println(line);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);

  Serial.begin(115200);           // USB serial for debug
  linkSerial.begin(LINK_BAUD);    // Selected link serial

  delay(500);
  Serial.print(F("UNO remote starting on "));
  Serial.println(LINK_NAME);

  // Initial HELLO
  sendLinkLine(String("HELLO,") + DEVICE_ID + "," + LINK_NAME);
  lastHelloAt = millis();
}

void loop() {
  // Heartbeat periodically
  maybeSendHeartbeat();

  // Read joystick and send if changed or interval elapsed
  JoyState current = computeJoy();
  unsigned long now = millis();
  bool changed = joyChanged(current, lastSent);
  bool timeElapsed = (now - lastSendAt) >= SEND_INTERVAL_MS;

  if (changed && timeElapsed) {
    sendJoystick(current);
    // Small activity blink
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2);
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Receive frames from robot via selected link (GPS, etc.)
  String frame;
  if (readLinkLine(frame)) {
    handleInboundFrame(frame);
  }
}
