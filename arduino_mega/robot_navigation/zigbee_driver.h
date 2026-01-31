#ifndef ZIGBEE_DRIVER_H
#define ZIGBEE_DRIVER_H

#include <Arduino.h>
#include "globals.h"
#include "wireless_interface.h"

/**
 * ZigBee Driver (Transparent UART)
 *
 * This driver is for CC2530/CC2591 "transparent serial" ZigBee modules and
 * similar UART bridges. It is NOT XBee API mode.
 *
 * Message format: newline-delimited ASCII frames ("...\n").
 * Incoming lines are delivered to the application as MSG_TYPE_COMMAND with
 * msg.data containing the ASCII text (no trailing CR/LF).
 *
 * Hardware connections (Arduino Mega):
 *   RX2 (pin 17) ← Module TX
 *   TX2 (pin 16) → Module RX
 *   GND ← GND
 *   3.3V ← VCC (3.3V recommended; many modules are NOT 5V tolerant)
 */
class ZigBeeDriver : public WirelessInterface {
private:
  HardwareSerial& serial;
  bool connected;
  uint32_t lastRxTime;
  String rxLine;

  static const uint32_t CONNECT_TIMEOUT = 5000;
  
public:
  ZigBeeDriver(HardwareSerial& serialPort = Serial2) 
    : serial(serialPort), connected(false), lastRxTime(0), rxLine("") {}

  bool begin() override {
    serial.begin(ZIGBEE_BAUD);
    delay(100);
    
    // Clear any pending data
    while (serial.available()) {
      serial.read();
    }

    // Transparent UART modules are often silent until the peer transmits.
    // We'll consider it "connected" after receiving any byte.
    unsigned long start = millis();
    while (!connected && (millis() - start) < CONNECT_TIMEOUT) {
      if (serial.available()) {
        connected = true;
        lastRxTime = millis();
        break;
      }
      delay(10);
    }

    // If silent, still allow TX; the remote will usually transmit first (PING).
    return true;
  }

  bool send(const WirelessMessage& msg) override {
    // For transparent UART, treat payload as raw bytes and terminate with newline.
    // Application code already sends ASCII frames (e.g. MCTL,...).
    if (msg.length > 0) {
      serial.write(msg.data, msg.length);
    }
    serial.write('\n');
    serial.flush();
    return true;
  }

  bool receive(WirelessMessage& msg) override {
    while (serial.available()) {
      char c = (char)serial.read();

      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        if (rxLine.length() == 0) {
          continue;
        }

        msg.type = MSG_TYPE_COMMAND;
        msg.length = (rxLine.length() > 64) ? 64 : rxLine.length();
        for (uint8_t i = 0; i < msg.length; i++) {
          msg.data[i] = (uint8_t)rxLine[i];
        }

        rxLine = "";
        lastRxTime = millis();
        connected = true;
        return true;
      }

      if (c >= 32 && c <= 126) {
        rxLine += c;
        if (rxLine.length() > 80) {
          // Prevent unbounded growth; keep tail.
          rxLine.remove(0, rxLine.length() - 60);
        }
      }
    }

    return false;
  }

  bool isConnected() const override {
    // Consider connected only if we saw RX recently.
    return connected && (millis() - lastRxTime < 10000);
  }

  int8_t getRSSI() const override {
    // Not easily available in transparent mode
    return 0;
  }

  const char* getProtocolName() const override {
    return "ZigBee (UART Transparent)";
  }

  void update() override {
    // Check for disconnection timeout
    if ((millis() - lastRxTime) > 15000) {
      connected = false;
    }
  }
};

#endif // ZIGBEE_DRIVER_H
