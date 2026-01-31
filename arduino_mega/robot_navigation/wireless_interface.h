#ifndef WIRELESS_INTERFACE_H
#define WIRELESS_INTERFACE_H

#include <Arduino.h>

// Wireless protocol selection is defined in globals.h
// (WIRELESS_PROTOCOL_ZIGBEE / WIRELESS_PROTOCOL_LORA / WIRELESS_PROTOCOL_BLE)

// Message structure for all protocols
struct WirelessMessage {
  uint8_t type;      // MESSAGE_TYPE_COMMAND, MESSAGE_TYPE_STATUS, etc.
  uint8_t length;
  uint8_t data[64];
  
  WirelessMessage() : type(0), length(0) {}
};

// Message types (compatible across all protocols)
enum MessageType : uint8_t {
  MSG_TYPE_COMMAND = 0x01,      // Remote control command
  MSG_TYPE_STATUS = 0x02,        // Robot status update
  MSG_TYPE_GPS = 0x03,           // GPS position data
  MSG_TYPE_OBSTACLE = 0x04,      // Obstacle alert
  MSG_TYPE_HANDSHAKE = 0x05,     // Connection handshake
  MSG_TYPE_ACK = 0x06,           // Acknowledgment
  MSG_TYPE_ERROR = 0x07,         // Error message
  MSG_TYPE_HEARTBEAT = 0x08      // Keep-alive
};

// Command subtypes
enum CommandType : uint8_t {
  WIRELESS_CMD_MOTOR_FORWARD = 0x10,
  WIRELESS_CMD_MOTOR_BACKWARD = 0x11,
  WIRELESS_CMD_MOTOR_LEFT = 0x12,
  WIRELESS_CMD_MOTOR_RIGHT = 0x13,
  WIRELESS_CMD_MOTOR_STOP = 0x14,
  WIRELESS_CMD_MODE_AUTO = 0x20,
  WIRELESS_CMD_MODE_MANUAL = 0x21,
  WIRELESS_CMD_NAV_START = 0x30,
  WIRELESS_CMD_NAV_STOP = 0x31,
  WIRELESS_CMD_NAV_PAUSE = 0x32,
  WIRELESS_CMD_NAV_RESUME = 0x33
};

/**
 * Abstract base class for wireless communication
 * All protocols inherit from this and implement the same interface
 */
class WirelessInterface {
public:
  virtual ~WirelessInterface() = default;

  // Initialize wireless module
  virtual bool begin() = 0;

  // Send message (blocking until sent or timeout)
  virtual bool send(const WirelessMessage& msg) = 0;

  // Receive message (non-blocking, returns true if message available)
  virtual bool receive(WirelessMessage& msg) = 0;

  // Check if module is connected
  virtual bool isConnected() const = 0;

  // Get RSSI (Received Signal Strength Indicator) if available
  virtual int8_t getRSSI() const = 0;

  // Get protocol name for debugging
  virtual const char* getProtocolName() const = 0;

  // Update internal state (for modules needing periodic updates)
  virtual void update() {}

  // Convenience method: send text string
  bool sendString(const char* str) {
    WirelessMessage msg;
    msg.type = MSG_TYPE_COMMAND;
    msg.length = strlen(str);
    if (msg.length > 64) msg.length = 64;
    memcpy(msg.data, str, msg.length);
    return send(msg);
  }

  // Convenience method: receive text string
  bool receiveString(char* str, uint8_t max_len) {
    WirelessMessage msg;
    if (!receive(msg)) return false;
    
    uint8_t copy_len = (msg.length < max_len) ? msg.length : (max_len - 1);
    memcpy(str, msg.data, copy_len);
    str[copy_len] = '\0';
    return true;
  }
};

#endif // WIRELESS_INTERFACE_H
