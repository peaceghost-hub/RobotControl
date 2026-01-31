#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Forward declarations to avoid circular includes
class GPSHandler;
class CompassHandler;
class Navigation;
class MotorControl;
class ObstacleAvoidance;

struct WaypointPacket;
struct PendingWaypoint;

// Hardware definitions
#define DEBUG_SERIAL   Serial
#define GPS_SERIAL     Serial1     // RX1 (19), TX1 (18)
#define BUZZER_PIN     34
// Audible tone frequency for passive buzzer (Hz)
#define BUZZER_FREQ    2000

// ==================== WIRELESS PROTOCOL SELECTION ====================
// Uncomment ONE of these to select wireless protocol:
#define WIRELESS_PROTOCOL_ZIGBEE     // Uses Serial2 (RX2=17, TX2=16), transparent UART
// #define WIRELESS_PROTOCOL_LORA    // Uses SPI + pins 9(CS) & 8(RST), long range
// #define WIRELESS_PROTOCOL_BLE     // Uses Serial3 (RX3=15, TX3=14), 38400 baud HC-05 / 9600 HM-10

// ======================================================================

// Constants
const uint32_t DEBUG_BAUD = 115200;
const uint32_t GPS_BAUD = 9600;
const uint8_t I2C_ADDRESS = 0x08;

// Compass I2C (software bus to avoid conflicts with Pi I2C)
// Wire the compass to these pins instead of SDA/SCL (20/21)
#define COMPASS_USE_SOFT_I2C 1
#define COMPASS_SDA_PIN 44
#define COMPASS_SCL_PIN 45

// Wireless protocol serial port baud rates
#ifdef WIRELESS_PROTOCOL_ZIGBEE
  #define WIRELESS_SERIAL Serial2
  const uint32_t WIRELESS_BAUD = 9600;
#elif defined(WIRELESS_PROTOCOL_BLE)
  #define WIRELESS_SERIAL Serial3
  const uint32_t WIRELESS_BAUD = 38400;  // HC-05: change to 9600 or set via AT mode
  // For HM-10, use 9600
#elif defined(WIRELESS_PROTOCOL_LORA)
  // LoRa uses SPI, no serial port needed
  const uint8_t LORA_CS_PIN = 9;
  const uint8_t LORA_RST_PIN = 8;
#endif

// Legacy compatibility (maps to selected protocol)
#ifdef WIRELESS_PROTOCOL_ZIGBEE
  #define ZIGBEE_SERIAL Serial2
  const uint32_t ZIGBEE_BAUD = 9600;
#endif

// I2C Protocol
const uint8_t CMD_PING            = 'P';
const uint8_t CMD_NAV_START       = 'S';
const uint8_t CMD_NAV_STOP        = 'T';
const uint8_t CMD_NAV_PAUSE       = 'A';
const uint8_t CMD_NAV_RESUME      = 'R';
const uint8_t CMD_WAYPOINT_CLEAR  = 'C';
const uint8_t CMD_WAYPOINT_PACKET = 'W';
const uint8_t CMD_WAYPOINT_COMMIT = 'M';
const uint8_t CMD_REQUEST_GPS     = 'G';
const uint8_t CMD_REQUEST_STATUS  = 'U';
const uint8_t CMD_REQUEST_OBSTACLE= 'O';  // New: request obstacle flag/distance
const uint8_t CMD_HEARTBEAT       = 'H';
// Enhanced feature commands
const uint8_t CMD_SEND_GPS        = 'F';  // Pi -> Mega GPS forwarding
const uint8_t CMD_SEND_HEADING    = 'D';  // Pi -> Mega heading forwarding
const uint8_t CMD_WAYPOINT_COMPLETED = 'Y';  // Mega -> Pi: waypoint reached
const uint8_t CMD_RETURN_TO_START = 'B';
const uint8_t CMD_MANUAL_OVERRIDE = 'V';
const uint8_t CMD_EMERGENCY_STOP  = 'E';
const uint8_t CMD_WIRELESS_BROADCAST = 'Z';
const uint8_t CMD_FOLLOW_LINE     = 'L';  // Enable/disable line follower mode

const uint8_t RESP_ACK    = 0x80;
const uint8_t RESP_GPS    = 0x81;
const uint8_t RESP_STATUS = 0x82;
const uint8_t RESP_OBSTACLE=0x83;  // Obstacle status response
const uint8_t RESP_ERROR  = 0xFF;

// Error codes
const uint8_t ERR_NO_WAYPOINTS = 0x01;
const uint8_t ERR_BUFFER_FULL  = 0x02;
const uint8_t ERR_PACKET_SIZE  = 0x03;

// Waypoint structures
struct WaypointPacket {
  uint16_t id;
  uint8_t seq;
  float latitude;
  float longitude;
};

struct PendingWaypoint {
  float latitude;
  float longitude;
  uint16_t id;
  uint8_t seq;
};

// Global instances
extern GPSHandler gps;
extern CompassHandler compass;
extern Navigation navigation;
extern MotorControl motors;
extern ObstacleAvoidance obstacleAvoid;

// Global variables
extern uint8_t responseBuffer[32];
extern uint8_t responseLength;
extern PendingWaypoint pendingWaypoints[];
extern uint8_t pendingWaypointCount;
extern bool navigationActive;
extern bool manualOverride;
extern bool i2cHandshakeComplete;
extern bool zigbeeHandshakeComplete;
extern unsigned long lastStatusUpdate;
extern unsigned long lastZigbeeGps;
extern unsigned long lastManualCommand;
extern unsigned long lastHeartbeat;
extern int manualSpeed;
extern String zigbeeBuffer;

// Constants for timing
const unsigned long STATUS_INTERVAL = 2000;     // ms
const unsigned long ZIGBEE_GPS_INTERVAL = 1200; // ms
const unsigned long MANUAL_TIMEOUT = 1500;      // ms
const unsigned long HEARTBEAT_TIMEOUT = 5000;   // ms

// Maximum number of waypoints
const uint8_t MAX_WAYPOINTS = 20;

enum ControlMode : uint8_t { MODE_AUTO = 0, MODE_MANUAL = 1 };
extern ControlMode controlMode;

// Function declarations
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
void handleZigbee();
void processZigbeeMessage(const String& message);
void sendZigbeeGps();
void sendZigbeeStatus();
void sendZigbeeReady();
void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void markI2CHandshake();
void markZigbeeHandshake();
void checkReadyTone();
uint8_t readBatteryPercent();
uint8_t readSignalQuality();

// Line follower sensor pins (4-pin module: GND, VCC, OUT, ENA)
#define LINE_FOLLOWER_OUT 32
#define LINE_FOLLOWER_ENA 33

// Constants for HMC5883L
#ifndef HMC5883_MAGGAIN_1_3
  #define HMC5883_MAGGAIN_1_3 0x20  // 1.3 gain
#endif

// Define PI if not already defined
#ifndef PI
  #define PI 3.14159265358979323846
#endif

#endif
