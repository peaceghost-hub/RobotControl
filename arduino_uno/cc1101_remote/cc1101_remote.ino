// Arduino UNO CC1101 Remote (SPI)
//
// Sends manual control commands from a joystick over CC1101 SPI module.
//
// Hardware connections (UNO <-> CC1101 module):
//   UNO D10 (CS)  -> Module CSN
//   UNO D11 (MOSI)-> Module SI (MOSI)
//   UNO D12 (MISO)-> Module SO (MISO)
//   UNO D13 (SCK) -> Module SCK
//   UNO D2 (GDO0) -> Module GDO0 (interrupt)
//   UNO D3 (GDO2) -> Module GDO2 (optional)
//   GND common
//   Module VCC -> 3.3V (IMPORTANT: NOT 5V!)
//
// Joystick:
//   VRx -> A0
//   VRy -> A1
//   GND -> GND
//   VCC -> 5V

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <SPI.h>

// CC1101 SPI pins (Arduino UNO)
static const uint8_t CC1101_CS = 10;
static const uint8_t CC1101_GDO0 = 2;
static const uint8_t CC1101_GDO2 = 3;

// CC1101 Configuration (must match receiver!)
static constexpr float FREQUENCY = 433.0;      // MHz
static constexpr float DATA_RATE = 9.6;        // kBaud
static const uint8_t MODULATION = 0;           // 2-FSK
static const uint8_t RX_BW = 325;              // 325 kHz
static const float DEVIATION = 47.60;          // kHz
static const uint8_t PA_POWER = 10;            // +10 dBm
static const uint8_t SYNC_MODE = 2;            // 16-bit sync word
static const uint16_t SYNC_WORD = 0xD191;      // 211, 145 (must match!)
static const uint8_t CRC_MODE = 1;             // CRC enabled
static const uint8_t PKT_FORMAT = 0;           // Normal mode
static const uint8_t LENGTH_CONFIG = 1;        // Variable packet length

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

// Message structure (matches receiver)
struct WirelessMessage {
  uint8_t type;
  uint8_t length;
  uint8_t data[64];

  WirelessMessage() : type(0), length(0) {}
};

// Message types
enum MessageType : uint8_t {
  MSG_TYPE_COMMAND = 0x01,
  MSG_TYPE_STATUS = 0x02,
  MSG_TYPE_HANDSHAKE = 0x05,
  MSG_TYPE_ACK = 0x06,
  MSG_TYPE_HEARTBEAT = 0x08
};

// Command subtypes
enum CommandType : uint8_t {
  WIRELESS_CMD_MOTOR_FORWARD = 0x10,
  WIRELESS_CMD_MOTOR_BACKWARD = 0x11,
  WIRELESS_CMD_MOTOR_LEFT = 0x12,
  WIRELESS_CMD_MOTOR_RIGHT = 0x13,
  WIRELESS_CMD_MOTOR_STOP = 0x14
};

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("[UNO] CC1101 Remote Booting..."));

  // Initialize CC1101
  if (!initCC1101()) {
    Serial.println(F("[UNO] CC1101 initialization failed!"));
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }

  Serial.println(F("[UNO] CC1101 SPI init complete"));

  // Send hello beacon to robot
  sendHandshake();
}

void loop() {
  // Periodic PING until READY seen
  if (!readySeen && (millis() - lastPingMs >= PING_INTERVAL_MS)) {
    Serial.println(F("[UNO] TX PING"));
    sendHeartbeat();
    lastPingMs = millis();
  }

  // Check for incoming messages
  checkForMessages();

  // Connection logic: consider connected if data seen in last 10 seconds
  bool nowConnected = (millis() - lastRxMillis) < 10000UL;
  if (nowConnected != connected) {
    connected = nowConnected;
    Serial.println(connected ? F("[UNO] CC1101 CONNECTED") : F("[UNO] CC1101 DISCONNECTED"));
    digitalWrite(LED_PIN, connected ? HIGH : LOW);
  }

  // Periodic hello beacon to robot (every 5s)
  static unsigned long lastHello = 0;
  if (millis() - lastHello >= 5000UL) {
    sendHandshake();
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
          Serial.print(F("MOTOR_FORWARD,"));
          Serial.println(speed);
          sendMotorCommand(WIRELESS_CMD_MOTOR_FORWARD, speed);
          break;
        case DIR_BACK:
          Serial.print(F("MOTOR_BACKWARD,"));
          Serial.println(speed);
          sendMotorCommand(WIRELESS_CMD_MOTOR_BACKWARD, speed);
          break;
        case DIR_LEFT:
          Serial.print(F("MOTOR_LEFT,"));
          Serial.println(speed);
          sendMotorCommand(WIRELESS_CMD_MOTOR_LEFT, speed);
          break;
        case DIR_RIGHT:
          Serial.print(F("MOTOR_RIGHT,"));
          Serial.println(speed);
          sendMotorCommand(WIRELESS_CMD_MOTOR_RIGHT, speed);
          break;
        case DIR_STOP:
        default:
          Serial.println(F("MOTOR_STOP"));
          sendMotorCommand(WIRELESS_CMD_MOTOR_STOP, 0);
          break;
      }

      lastDir = dir;
      lastSpeed = speed;
    }
    lastCommandMs = millis();
  }
}

bool initCC1101() {
  // setSpiPin(sck, miso, mosi, ss)
  ELECHOUSE_cc1101.setSpiPin(13, 12, 11, CC1101_CS);
  ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setCCMode(1);       // High sensitivity mode
  ELECHOUSE_cc1101.setModulation(MODULATION);
  ELECHOUSE_cc1101.setMHZ(FREQUENCY);
  ELECHOUSE_cc1101.setRxBW(RX_BW);
  ELECHOUSE_cc1101.setDeviation(DEVIATION);
  ELECHOUSE_cc1101.setPA(PA_POWER);
  ELECHOUSE_cc1101.setSyncMode(SYNC_MODE);
  ELECHOUSE_cc1101.setSyncWord(SYNC_WORD, false);
  ELECHOUSE_cc1101.setCrc(CRC_MODE);
  ELECHOUSE_cc1101.setPktFormat(PKT_FORMAT);
  ELECHOUSE_cc1101.setLengthConfig(LENGTH_CONFIG);
  ELECHOUSE_cc1101.setDRate(DATA_RATE);

  // Test communication
  if (!ELECHOUSE_cc1101.getCC1101()) {
    return false;
  }

  // Set receive mode
  ELECHOUSE_cc1101.SetRx();
  return true;
}

void sendHandshake() {
  WirelessMessage msg;
  msg.type = MSG_TYPE_HANDSHAKE;
  msg.length = 12;
  memcpy(msg.data, "UNO_REMOTE", 10);
  msg.data[10] = 0x00;
  msg.data[11] = 0x01; // Version

  ELECHOUSE_cc1101.SendData((byte*)&msg, msg.length + 2);
}

void sendHeartbeat() {
  WirelessMessage msg;
  msg.type = MSG_TYPE_HEARTBEAT;
  msg.length = 0;

  ELECHOUSE_cc1101.SendData((byte*)&msg, 2);
}

void sendMotorCommand(uint8_t command, uint8_t speed) {
  WirelessMessage msg;
  msg.type = MSG_TYPE_COMMAND;
  msg.length = 2;
  msg.data[0] = command;
  msg.data[1] = speed;

  ELECHOUSE_cc1101.SendData((byte*)&msg, msg.length + 2);
}

void checkForMessages() {
  if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
    byte rxBuffer[64];
    byte size = ELECHOUSE_cc1101.ReceiveData(rxBuffer);

    if (size >= 2) {
      WirelessMessage* msg = (WirelessMessage*)rxBuffer;

      lastRxMillis = millis();
      Serial.print(F("[UNO] RX type="));
      Serial.print(msg->type, HEX);
      Serial.print(F(" len="));
      Serial.println(msg->length);

      // Process message
      switch (msg->type) {
        case MSG_TYPE_STATUS:
          if (msg->length >= 5 && memcmp(msg->data, "READY", 5) == 0) {
            readySeen = true;
            Serial.println(F("[UNO] READY received"));
          }
          break;

        case MSG_TYPE_ACK:
          Serial.println(F("[UNO] ACK received"));
          break;

        default:
          Serial.println(F("[UNO] Unknown message type"));
          break;
      }
    }
  }
}