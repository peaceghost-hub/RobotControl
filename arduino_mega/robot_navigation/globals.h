#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Wire.h>

// Forward declarations to avoid circular includes
class GPSHandler;
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
// #define WIRELESS_PROTOCOL_ZIGBEE     // Uses Serial2 (RX2=17, TX2=16), transparent UART
// #define WIRELESS_PROTOCOL_LORA    // Uses SPI + pins 9(CS) & 8(RST), long range
#define WIRELESS_PROTOCOL_CC1101   // Uses SPI + pins 53(CS) & 2(GDO0), CC1101 module
// #define WIRELESS_PROTOCOL_BLE     // Uses Serial3 (RX3=15, TX3=14), 38400 baud HC-05 / 9600 HM-10

// ======================================================================

// Constants
const uint32_t DEBUG_BAUD = 115200;
const uint32_t GPS_BAUD = 9600;
const uint8_t I2C_ADDRESS = 0x08;

// Wireless protocol serial port baud rates
#ifdef WIRELESS_PROTOCOL_ZIGBEE
  #define WIRELESS_SERIAL Serial2
  const uint32_t WIRELESS_BAUD = 9600;
#elif defined(WIRELESS_PROTOCOL_BLE)
  #define WIRELESS_SERIAL Serial3
  const uint32_t WIRELESS_BAUD = 38400;  // HC-05: change to 9600 or set via AT mode
  // For HM-10, use 9600
#elif defined(WIRELESS_PROTOCOL_CC1101)
  // CC1101 SPI module pins (Arduino Mega)
  #define CC1101_CS_PIN 53      // Chip Select (SS)
  #define CC1101_GDO0_PIN 2     // Interrupt pin
  #define CC1101_GDO2_PIN 3     // Optional interrupt/control pin
  constexpr float CC1101_FREQUENCY = 433.00f;  // MHz
  constexpr float CC1101_DATA_RATE = 9.6f;     // kBaud
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
const uint8_t CMD_SOUND_BUZZER = 'Q';  // New: sound buzzer for duration
const uint8_t CMD_HEARTBEAT       = 'H';
const uint8_t CMD_SET_AUTO_SPEED  = 'N';  // New: set autonomous navigation base speed (PWM)
const uint8_t CMD_ENGAGE_WIRELESS = 'X';  // New: engage/disengage CC1101 backup control
// Enhanced feature commands
const uint8_t CMD_SEND_GPS        = 'F';  // Pi -> Mega GPS forwarding
const uint8_t CMD_SEND_HEADING    = 'D';  // Pi -> Mega heading forwarding
const uint8_t CMD_WAYPOINT_COMPLETED = 'Y';  // Mega -> Pi: waypoint reached
const uint8_t CMD_RETURN_TO_START = 'B';
const uint8_t CMD_MANUAL_OVERRIDE = 'V';
const uint8_t CMD_EMERGENCY_STOP  = 'E';
const uint8_t CMD_WIRELESS_BROADCAST = 'Z';
// CMD_FOLLOW_LINE removed — no line follower hardware

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
  double latitude;
  double longitude;
};

struct PendingWaypoint {
  double latitude;
  double longitude;
  uint16_t id;
  uint8_t seq;
};

// Global instances
extern GPSHandler gps;
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
extern bool wirelessHandshakeComplete;
extern unsigned long lastStatusUpdate;
extern unsigned long lastWirelessGps;
extern unsigned long lastManualCommand;
extern unsigned long lastHeartbeat;
extern int manualSpeed;

// Constants for timing
const unsigned long STATUS_INTERVAL = 2000;     // ms
const unsigned long ZIGBEE_GPS_INTERVAL = 1200; // ms
const unsigned long MANUAL_TIMEOUT = 7500;      // ms (x5 from 1500)
const unsigned long HEARTBEAT_TIMEOUT = 25000;  // ms (x5 from 5000)

// Maximum number of waypoints
const uint8_t MAX_WAYPOINTS = 20;

enum ControlMode : uint8_t { MODE_AUTO = 0, MODE_MANUAL = 1 };
extern ControlMode controlMode;

// ===================== STATE MACHINE (blueprint) ==========================
// Two mutually-exclusive states — only one "owner" drives the motors.
enum RobotState : uint8_t {
  STATE_I2C      = 0,   // Pi owns the motors (autonomous / Pi-joystick)
  STATE_WIRELESS = 1,   // ESP8266 remote owns the motors
  STATE_FAILSAFE = 2    // Both comms lost — motors stopped, idle
};
extern RobotState robotState;

// Timing budget constants (µs / ms)
const unsigned long CC1101_POLL_INTERVAL   = 50;    // ms between SPI polls (match ESP8266 TX rate)
const unsigned long WIRELESS_LINK_TIMEOUT  = 15000; // ms — no packets → back to I2C (x5 from 3000)
const unsigned long I2C_LINK_TIMEOUT       = 15000; // ms — no I2C activity → consider lost (x5 from 3000)
const unsigned long SENSOR_UPDATE_INTERVAL = 100;   // ms between ultrasonic reads
const unsigned long BUZZER_TICK_INTERVAL   = 10;    // ms for non-blocking buzzer
const unsigned long FAILSAFE_BEEP_INTERVAL = 4000;  // ms — slow beep in failsafe mode

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
void transitionToState(RobotState newState);
void pollCC1101();
void processRawMotorCommand(int16_t throttle, int16_t steer, uint8_t flags);
void processWirelessCommand(uint8_t cmd, uint8_t speed);
void processWirelessMessage(const String& message);
void sendWirelessGps();
void sendWirelessStatus();
void sendWirelessReady();
void sendWirelessObstacleAlert(int distance);
void beepPattern(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void beepPatternNB(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void updateBuzzer();
void markI2CHandshake();
void markWirelessHandshake();
uint8_t readBatteryPercent();
uint8_t readSignalQuality();

// Line follower and KY-032 IR sensor removed — only HC-SR04 ultrasonic for obstacle detection

// Define PI if not already defined
#ifndef PI
  #define PI 3.14159265358979323846
#endif

#endif
