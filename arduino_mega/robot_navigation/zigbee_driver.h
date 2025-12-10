#ifndef ZIGBEE_DRIVER_H
#define ZIGBEE_DRIVER_H

#include <Arduino.h>
#include "wireless_interface.h"

/**
 * ZigBee Driver for XBee modules
 * Supports Series 1 and Series 2 modules
 * Default baud: 57600
 * 
 * Hardware connections (Arduino Mega):
 *   RX2 (pin 17) ← DOUT (Module TX)
 *   TX2 (pin 16) → DIN (Module RX)
 *   GND ← GND
 *   3.3V ← VCC (with LDO regulator recommended)
 */
class ZigBeeDriver : public WirelessInterface {
private:
  HardwareSerial& serial;
  bool connected;
  uint32_t lastPacketTime;
  uint8_t packetBuffer[96];  // XBee API mode buffer
  uint8_t bufferPos;
  uint8_t expectedLength;
  
  static const uint32_t BAUD_RATE = 57600;
  static const uint32_t CONNECT_TIMEOUT = 5000;
  static const uint32_t PACKET_TIMEOUT = 100;
  
  // XBee API frame types
  static const uint8_t API_START_DELIMITER = 0x7E;
  static const uint8_t API_TRANSMIT_REQUEST = 0x10;
  static const uint8_t API_TRANSMIT_STATUS = 0x89;
  static const uint8_t API_RECEIVE_PACKET = 0x90;
  static const uint8_t API_REMOTE_COMMAND = 0x97;
  
public:
  ZigBeeDriver(HardwareSerial& serialPort = Serial2) 
    : serial(serialPort), connected(false), lastPacketTime(0), 
      bufferPos(0), expectedLength(0) {}

  bool begin() override {
    serial.begin(BAUD_RATE);
    delay(100);
    
    // Clear any pending data
    while (serial.available()) {
      serial.read();
    }
    
    // Send AT command to check module (non-API mode compatibility)
    // In API mode, module responds with frame
    unsigned long start = millis();
    
    while (!connected && (millis() - start) < CONNECT_TIMEOUT) {
      // Check for any response indicating module is alive
      if (serial.available()) {
        uint8_t b = serial.read();
        if (b == API_START_DELIMITER || (b >= 32 && b <= 126)) {
          connected = true;
          lastPacketTime = millis();
          break;
        }
      }
      delay(10);
    }
    
    return connected;
  }

  bool send(const WirelessMessage& msg) override {
    if (!connected) return false;

    // Build XBee API frame: [Start][Length][Type][Data][Checksum]
    uint8_t frame[96];
    frame[0] = API_START_DELIMITER;
    
    // Frame length: type(1) + frame_id(1) + dest_addr(8) + options(1) + data
    uint8_t payload_len = msg.length + 11;
    frame[1] = (payload_len >> 8) & 0xFF;  // Length MSB
    frame[2] = payload_len & 0xFF;         // Length LSB
    
    frame[3] = API_TRANSMIT_REQUEST;  // Frame type
    frame[4] = 0x01;                  // Frame ID
    frame[5] = 0x00; frame[6] = 0x00; // Dest 64-bit addr (broadcast)
    frame[7] = 0x00; frame[8] = 0x00;
    frame[9] = 0xFF; frame[10] = 0xFF;
    frame[11] = 0xFF; frame[12] = 0xFE;
    frame[13] = 0x00;                 // Options (no ACK)
    
    // Copy message data
    memcpy(&frame[14], msg.data, msg.length);
    
    // Calculate checksum: sum of all bytes after length
    uint8_t checksum = 0;
    for (int i = 3; i < (14 + msg.length); i++) {
      checksum += frame[i];
    }
    checksum = 0xFF - checksum;
    frame[14 + msg.length] = checksum;
    
    // Send entire frame
    serial.write(frame, 15 + msg.length);
    serial.flush();
    
    lastPacketTime = millis();
    return true;
  }

  bool receive(WirelessMessage& msg) override {
    if (!connected) return false;

    while (serial.available()) {
      uint8_t b = serial.read();
      
      // Look for API start delimiter
      if (b == API_START_DELIMITER) {
        bufferPos = 0;
        packetBuffer[bufferPos++] = b;
        expectedLength = 0;
        continue;
      }
      
      if (bufferPos == 0) {
        // Waiting for start delimiter
        continue;
      }
      
      packetBuffer[bufferPos++] = b;
      
      // Check for complete frame length header
      if (bufferPos == 3) {
        expectedLength = ((packetBuffer[1] << 8) | packetBuffer[2]) + 4;
      }
      
      // Check if complete frame received
      if (bufferPos >= expectedLength && expectedLength > 0) {
        // Parse frame
        if (packetBuffer[3] == API_RECEIVE_PACKET) {
          // Extract data from receive packet frame
          // Frame: [Start][Len_MSB][Len_LSB][Type][Src_Addr64(8)][Src_Addr16(2)][Options][RSSI][Data...][Checksum]
          uint8_t data_start = 14;  // Skip header and source info
          uint8_t data_len = expectedLength - data_start - 1;  // -1 for checksum
          
          if (data_len > 0 && data_len <= 64) {
            msg.type = packetBuffer[3];
            msg.length = data_len;
            memcpy(msg.data, &packetBuffer[data_start], data_len);
            
            lastPacketTime = millis();
            bufferPos = 0;
            expectedLength = 0;
            return true;
          }
        }
        
        bufferPos = 0;
        expectedLength = 0;
      }
      
      // Prevent buffer overflow
      if (bufferPos >= 96) {
        bufferPos = 0;
        expectedLength = 0;
      }
    }
    
    return false;
  }

  bool isConnected() const override {
    // Consider connected if received packet in last 10 seconds
    return connected && (millis() - lastPacketTime < 10000);
  }

  int8_t getRSSI() const override {
    // Not easily available in transparent mode
    return 0;
  }

  const char* getProtocolName() const override {
    return "ZigBee (XBee)";
  }

  void update() override {
    // Check for disconnection timeout
    if ((millis() - lastPacketTime) > 15000) {
      connected = false;
    }
  }
};

#endif // ZIGBEE_DRIVER_H
