#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include <Arduino.h>
#include "globals.h"
#include "wireless_interface.h"

/**
 * LoRa Driver (UART Transparent Mode)
 *
 * This driver is for UART-based LoRa modules like E32/E220 series.
 * M0 and M1 pins should be grounded for normal transparent transmission mode.
 *
 * Message format: newline-delimited ASCII frames ("...\n").
 * Incoming lines are delivered to the application as MSG_TYPE_COMMAND with
 * msg.data containing the ASCII text (no trailing CR/LF).
 *
 * Hardware connections (Arduino Mega):
 *   RX2 (pin 17) ← Module TXD
 *   TX2 (pin 16) → Module RXD
 *   M0 → GND (normal mode)
 *   M1 → GND (normal mode)
 *   GND ← GND
 *   VCC ← 3.3-5V (per module spec)
 */
class LoRaDriver : public WirelessInterface {
private:
  HardwareSerial& serial;
  bool connected;
  uint32_t lastRxTime;
  String rxLine;

  static const uint32_t CONNECT_TIMEOUT = 5000;
  static const uint8_t REG_OP_MODE = 0x01;
  static const uint8_t REG_FRF_MSB = 0x06;
  static const uint8_t REG_FRF_MID = 0x07;
  static const uint8_t REG_FRF_LSB = 0x08;
  static const uint8_t REG_PA_CONFIG = 0x09;
  static const uint8_t REG_LNA = 0x0C;
  static const uint8_t REG_FIFO_ADDR_PTR = 0x0D;
  static const uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
  static const uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
  static const uint8_t REG_IRQ_FLAGS = 0x12;
  static const uint8_t REG_PAYLOAD_LENGTH = 0x22;
  static const uint8_t REG_MODEM_CONFIG_1 = 0x1D;
  static const uint8_t REG_MODEM_CONFIG_2 = 0x1E;
  static const uint8_t REG_DIO_MAPPING_1 = 0x40;
  
  // LoRa operating modes
  static const uint8_t MODE_SLEEP = 0x00;
  static const uint8_t MODE_STANDBY = 0x01;
  static const uint8_t MODE_TX = 0x03;
  static const uint8_t MODE_RX_CONTINUOUS = 0x05;
  static const uint8_t MODE_RX_SINGLE = 0x06;
  
  // IRQ flags
  static const uint8_t IRQ_TX_DONE = 0x08;
  static const uint8_t IRQ_RX_DONE = 0x40;
  
  // Frequency: 915 MHz (USA) - adjust for your region
  static const uint32_t FREQ_CENTER = 915000000;
  
public:
  LoRaDriver(HardwareSerial& serialPort = Serial2) 
    : serial(serialPort), connected(false), lastRxTime(0), rxLine("") {}

  bool begin() override {
    serial.begin(LORA_BAUD);
    delay(100);
    
    // Clear any pending data
    while (serial.available()) {
      serial.read();
    }

    // UART LoRa modules are often silent until the peer transmits.
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
    return "LoRa (UART Transparent)";
  }

  void update() override {
    // Check for disconnection timeout
    if ((millis() - lastRxTime) > 15000) {
      connected = false;
    }
  }
};

#endif // LORA_DRIVER_H
