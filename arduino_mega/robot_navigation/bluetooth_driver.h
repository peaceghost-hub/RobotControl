#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H

#include <Arduino.h>
#include "wireless_interface.h"

/**
 * Bluetooth Driver for HC-05 (Classic BT) and HM-10 (BLE)
 * For mobile device or nearby control
 * 
 * Hardware connections (Arduino Mega):
 *   RX3 (pin 15) ← TXD (Module TX)
 *   TX3 (pin 14) → RXD (Module RX)
 *   GND ← GND
 *   5V ← VCC (check module specs, may need level shifter)
 * 
 * HC-05: Classic Bluetooth (better range and compatibility)
 *   Baud: 38400 or 9600 (configurable, default 9600)
 *   Range: 10-100m
 *   Power: ~40mA operating
 *   Best for: Android phones, older iOS
 * 
 * HM-10: Bluetooth Low Energy (BLE, lower power)
 *   Baud: 9600 (fixed)
 *   Range: 20-30m
 *   Power: ~10mA operating
 *   Best for: iPhone (iOS 5+), modern Android
 * 
 * To use HM-10, change BAUD_RATE to 9600 and uncomment HM10_MODE
 */
class BluetoothDriver : public WirelessInterface {
private:
  HardwareSerial& serial;
  bool connected;
  uint32_t lastPacketTime;
  uint32_t lastConnectAttempt;
  String rxBuffer;
  
  // Configuration: uncomment for HM-10, leave commented for HC-05
  // #define HM10_MODE
  
  #ifdef HM10_MODE
    static const uint32_t BAUD_RATE = 9600;      // HM-10 fixed baud
    static const char* MODULE_TYPE = "HM-10 (BLE)";
  #else
    static const uint32_t BAUD_RATE = 38400;     // HC-05 default
    static const char* MODULE_TYPE = "HC-05 (Classic BT)";
  #endif
  
  static const uint32_t CONNECT_TIMEOUT = 5000;
  static const uint32_t PACKET_TIMEOUT = 100;
  
public:
  BluetoothDriver(HardwareSerial& serialPort = Serial3) 
    : serial(serialPort), connected(false), lastPacketTime(0), 
      lastConnectAttempt(0), rxBuffer("") {}

  bool begin() override {
    serial.begin(BAUD_RATE);
    delay(100);
    
    // Clear any pending data
    while (serial.available()) {
      serial.read();
    }
    
    // BT module should respond with ready signal
    // In transparent mode, just check if we can communicate
    unsigned long start = millis();
    bool readyFound = false;
    
    while ((millis() - start) < CONNECT_TIMEOUT) {
      if (serial.available()) {
        char c = serial.read();
        if (c == '+' || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
          readyFound = true;
          connected = true;
          lastPacketTime = millis();
          lastConnectAttempt = millis();
          break;
        }
      }
      delay(10);
    }
    
    if (!readyFound) {
      // Module might already be silent (transparent mode)
      // Try to detect presence by waiting for connection
      connected = true;  // Assume connected
      lastPacketTime = millis();
      lastConnectAttempt = millis();
    }
    
    return true;
  }

  bool send(const WirelessMessage& msg) override {
    if (!connected) return false;
    
    // Format: [Type]:[Length]:[Data...]
    // Example: 01:05:HELLO
    char txBuffer[128];
    int pos = 0;
    
    // Type and length as hex
    pos += snprintf(txBuffer + pos, sizeof(txBuffer) - pos, 
                   "%02X:%02X:", msg.type, msg.length);
    
    // Data as ASCII if printable, otherwise as hex
    for (int i = 0; i < msg.length && pos < 120; i++) {
      if (msg.data[i] >= 32 && msg.data[i] < 127) {
        txBuffer[pos++] = msg.data[i];
      } else {
        // Non-printable, escape as \xHH
        pos += snprintf(txBuffer + pos, sizeof(txBuffer) - pos, 
                       "\\x%02X", msg.data[i]);
      }
    }
    
    txBuffer[pos++] = '\r';
    txBuffer[pos++] = '\n';
    txBuffer[pos] = '\0';
    
    serial.write((uint8_t*)txBuffer, pos);
    serial.flush();
    
    lastPacketTime = millis();
    return true;
  }

  bool receive(WirelessMessage& msg) override {
    if (!connected) return false;
    
    // Read all available data
    while (serial.available()) {
      char c = serial.read();
      
      if (c == '\r' || c == '\n') {
        if (rxBuffer.length() > 0) {
          // Parse buffer: "TYPE:LENGTH:DATA"
          if (parseMessage(rxBuffer, msg)) {
            rxBuffer = "";
            lastPacketTime = millis();
            return true;
          }
          rxBuffer = "";
        }
      } else if (c >= 32 && c < 127) {
        rxBuffer += c;
        if (rxBuffer.length() > 100) {
          rxBuffer.remove(0, 50);  // Prevent overflow
        }
      }
    }
    
    return false;
  }

  bool isConnected() const override {
    // Consider connected if received packet in last 10 seconds
    // or very recently connected (module might be silent in transparent mode)
    return connected && (millis() - lastPacketTime < 10000 || 
                        (millis() - lastConnectAttempt < 5000));
  }

  int8_t getRSSI() const override {
    // Bluetooth RSSI not easily available without AT commands
    return -80;  // Typical range
  }

  const char* getProtocolName() const override {
    #ifdef HM10_MODE
      return "Bluetooth LE (HM-10)";
    #else
      return "Classic Bluetooth (HC-05)";
    #endif
  }

  void update() override {
    // Periodically check connection
    if ((millis() - lastPacketTime) > 15000) {
      // No packets for 15 seconds - might be disconnected
      // Try sending a keepalive
      WirelessMessage keepalive;
      keepalive.type = MSG_TYPE_HEARTBEAT;
      keepalive.length = 0;
      send(keepalive);
    }
  }

  // Set HC-05 baud rate (AT mode)
  bool configureBaudRate(uint32_t newBaud) {
    #ifdef HM10_MODE
      return false;  // HM-10 has fixed baud
    #else
      // Only works in AT mode (hold KEY pin during boot)
      // This is a placeholder - actual implementation would:
      // 1. Switch to AT mode
      // 2. Send AT+UART=<baud>,0,0
      // 3. Restart module
      return true;
    #endif
  }

  // Connect to specific device by address (HC-05 only)
  bool connectToAddress(const char* btAddress) {
    #ifdef HM10_MODE
      return false;  // Not applicable for BLE
    #else
      // Example: "AT+LINK=AABBCCDDEEFF"
      // Address format: AA:BB:CC:DD:EE:FF
      char cmd[32];
      snprintf(cmd, sizeof(cmd), "AT+LINK=%s", btAddress);
      // Would need AT mode to work properly
      return false;
    #endif
  }

private:
  bool parseMessage(const String& str, WirelessMessage& msg) {
    // Parse format: "TYPE:LENGTH:DATA"
    // Example: "02:05:HELLO"
    
    int colon1 = str.indexOf(':');
    int colon2 = str.indexOf(':', colon1 + 1);
    
    if (colon1 < 0 || colon2 < 0) {
      return false;
    }
    
    // Parse type
    String typeStr = str.substring(0, colon1);
    msg.type = strtol(typeStr.c_str(), NULL, 16);
    
    // Parse length
    String lenStr = str.substring(colon1 + 1, colon2);
    msg.length = strtol(lenStr.c_str(), NULL, 16);
    
    if (msg.length > 64) {
      return false;
    }
    
    // Parse data
    String dataStr = str.substring(colon2 + 1);
    
    for (int i = 0; i < msg.length && i < dataStr.length(); i++) {
      msg.data[i] = dataStr[i];
    }
    
    return true;
  }
};

#endif // BLUETOOTH_DRIVER_H
