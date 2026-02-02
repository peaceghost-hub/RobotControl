// Arduino UNO LoRa Remote (UART Transparent Mode)
//
// Sends manual control commands from a joystick over LoRa UART module.
//
// Hardware connections (UNO <-> LoRa module):
//   UNO D6 (TX)  -> Module RXD (pin 5)
//   UNO D5 (RX)  -> Module TXD (pin 6)
//   Module M0    -> GND (normal mode)
//   Module M1    -> GND (normal mode)
//   GND common
//   Module VCC -> 3.3-5V (per module spec)
//
// Joystick:
//   VRx -> A0
//   VRy -> A1
//   GND -> GND
//   VCC -> 5V

#include <SoftwareSerial.h>

// LoRa UART pins
static const uint8_t LORA_RX_PIN = 5;
static const uint8_t LORA_TX_PIN = 6;
static const long LORA_BAUD = 9600;

SoftwareSerial lora(LORA_RX_PIN, LORA_TX_PIN); // RX, TX

// Connection tracking
unsigned long lastRxMillis = 0;
bool connected = false;
bool readySeen = false;

// Status LED
static const uint8_t LED_PIN = 13;

// Joystick pins
static const uint8_t JOY_X_PIN = A0;
static const uint8_t JOY_Y_PIN = A1;

// Tuning
static const int JOY_CENTER = 512;
static const int JOY_DEADZONE = 90;
static const unsigned long COMMAND_INTERVAL_MS = 120;
static const unsigned long PING_INTERVAL_MS = 1000;

static unsigned long lastCommandMs = 0;
static unsigned long lastPingMs = 0;

enum MoveDir { DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT };
MoveDir lastDir = DIR_STOP;
int lastSpeed = 0;
bool readySeen = false;
bool connected = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("[UNO] LoRa Remote Booting..."));

  lora.begin(LORA_BAUD);
  delay(100);
  flushLoRa();

  Serial.print(F("[UNO] LoRa UART init on D"));
  Serial.print(LORA_RX_PIN);
  Serial.print(F("(RX)/D"));
  Serial.print(LORA_TX_PIN);
  Serial.print(F("(TX) @ "));
  Serial.println(LORA_BAUD);

  // Send hello beacon to robot
  lora.println(F("HELLO,UNO_REMOTE"));
  lora.println(F("PING"));
}

void loop() {
  // Periodic PING until READY seen
  if (!readySeen && (millis() - lastPingMs >= PING_INTERVAL_MS)) {
    Serial.println(F("[UNO] TX PING"));
    lora.println(F("PING"));
    lastPingMs = millis();
  }

  // Echo any incoming LoRa frames to Serial and update connection status
  static String line;
  while (lora.available()) {
    char c = lora.read();

    if (c == '\n') {
      line.trim();
      if (line.length() > 0) {
        // Only consider the link alive when we receive a well-formed ASCII line.
        lastRxMillis = millis();
        Serial.print(F("[UNO] RX "));
        Serial.println(line);
        if (line.equalsIgnoreCase("READY")) {
          readySeen = true;
          Serial.println(F("[UNO] READY received"));
        }
      }
      line = "";
    } else if (c != '\r') {
      // Keep only printable ASCII to avoid garbage.
      if (c >= 32 && c <= 126) {
        line += c;
      }
      if (line.length() > 96) {
        line.remove(0, line.length() - 64);
      }
    }
  }

  // Forward any Serial input to LoRa (manual test typing)
  while (Serial.available()) {
    lora.write(Serial.read());
  }

  // Connection logic: consider connected if data seen in last 10 seconds
  bool nowConnected = (millis() - lastRxMillis) < 10000UL;
  if (nowConnected != connected) {
    connected = nowConnected;
    Serial.println(connected ? F("[UNO] LoRa CONNECTED") : F("[UNO] LoRa DISCONNECTED"));
    digitalWrite(LED_PIN, connected ? HIGH : LOW);
  }

  // Periodic hello beacon to robot (every 5s)
  static unsigned long lastHello = 0;
  if (millis() - lastHello >= 5000UL) {
    lora.println(F("HELLO,UNO_REMOTE"));
    lastHello = millis();
  }

  // Joystick control loop
  if (millis() - lastCommandMs >= COMMAND_INTERVAL_MS) {
    int x = analogRead(JOY_X_PIN);
    int y = analogRead(JOY_Y_PIN);

    int dx = x - JOY_CENTER;
    int dy = y - JOY_CENTER;

    MoveDir dir = DIR_STOP;
    int mag = 0;
    if (abs(dy) >= abs(dx)) {
      // Forward/back dominates
      if (dy > JOY_DEADZONE) {
        dir = DIR_FWD;
        mag = dy;
      } else if (dy < -JOY_DEADZONE) {
        dir = DIR_BACK;
        mag = -dy;
      }
    } else {
      // Left/right dominates
      if (dx > JOY_DEADZONE) {
        dir = DIR_RIGHT;
        mag = dx;
      } else if (dx < -JOY_DEADZONE) {
        dir = DIR_LEFT;
        mag = -dx;
      }
    }

    // Map magnitude to speed (0-255)
    int speed = 0;
    if (dir != DIR_STOP) {
      mag = constrain(mag, JOY_DEADZONE, 512);
      speed = map(mag, JOY_DEADZONE, 512, 120, 255);
    }

    // Only send when changes occur
    if (dir != lastDir || abs(speed - lastSpeed) >= 10) {
      Serial.print(F("[UNO] TX "));
      switch (dir) {
        case DIR_FWD:
          Serial.print(F("MCTL,FORWARD,"));
          Serial.println(speed);
          lora.print(F("MCTL,FORWARD,"));
          lora.println(speed);
          break;
        case DIR_BACK:
          Serial.print(F("MCTL,BACKWARD,"));
          Serial.println(speed);
          lora.print(F("MCTL,BACKWARD,"));
          lora.println(speed);
          break;
        case DIR_LEFT:
          Serial.print(F("MCTL,LEFT,"));
          Serial.println(speed);
          lora.print(F("MCTL,LEFT,"));
          lora.println(speed);
          break;
        case DIR_RIGHT:
          Serial.print(F("MCTL,RIGHT,"));
          Serial.println(speed);
          lora.print(F("MCTL,RIGHT,"));
          lora.println(speed);
          break;
        case DIR_STOP:
        default:
          Serial.println(F("MCTL,STOP"));
          lora.println(F("MCTL,STOP"));
          break;
      }

      lastDir = dir;
      lastSpeed = speed;
    }
    lastCommandMs = millis();
  }
}

void flushLoRa() {
  while (lora.available()) { lora.read(); }
}
