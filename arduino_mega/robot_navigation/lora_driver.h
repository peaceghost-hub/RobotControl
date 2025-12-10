#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include <Arduino.h>
#include "wireless_interface.h"

/**
 * LoRa Driver for SX1276/RFM95W modules
 * Low power, long range (up to 15km line-of-sight)
 * 
 * Hardware connections (Arduino Mega):
 *   Digital 9 (pin 9) → NSS/CS (Module chip select)
 *   Digital 8 (pin 8) → RST (Module reset)
 *   SPI MOSI (pin 51) → MOSI
 *   SPI MISO (pin 50) → MISO
 *   SPI SCK (pin 52) → SCK
 *   GND ← GND
 *   3.3V ← VCC (with LDO regulator, 100-150mA peak)
 * 
 * Recommended module: HopeRF RFM95W or Semtech SX1276
 * Library: arduino-lmic or RadioHead
 */
class LoRaDriver : public WirelessInterface {
private:
  uint8_t csPin, rstPin;
  bool connected;
  uint32_t lastPacketTime;
  uint8_t rxBuffer[256];
  uint8_t rxBufferLen;
  
  // LoRa register definitions (simplified)
  static const uint8_t REG_FIFO = 0x00;
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
  LoRaDriver(uint8_t chipSelectPin = 9, uint8_t resetPin = 8) 
    : csPin(chipSelectPin), rstPin(resetPin), connected(false), 
      lastPacketTime(0), rxBufferLen(0) {}

  bool begin() override {
    pinMode(csPin, OUTPUT);
    pinMode(rstPin, OUTPUT);
    
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);  // 4MHz SPI clock
    
    // Reset module
    digitalWrite(rstPin, LOW);
    delay(100);
    digitalWrite(rstPin, HIGH);
    delay(100);
    
    // Check if module is responding
    uint8_t version = readReg(0x42);  // Read version register
    if (version != 0x12) {
      // Version might be different, try initializing anyway
      Serial.print(F("# LoRa version: 0x"));
      Serial.println(version, HEX);
    }
    
    // Set to LoRa mode
    writeReg(REG_OP_MODE, 0x80);  // LoRa mode
    delay(10);
    
    // Set frequency
    setFrequency(FREQ_CENTER);
    
    // Configure modem (SF7, BW125k, CRC enabled)
    writeReg(REG_MODEM_CONFIG_1, 0x72);  // Bw=125kHz, CODING_RATE_4_5, IMPLICIT_HEADER_MODE_OFF
    writeReg(REG_MODEM_CONFIG_2, 0x74);  // SF=7, TX_CONTINUOUS_MODE_OFF, RX_PAYLOAD_CRC_ON
    
    // Set power output (max: 20dBm)
    writeReg(REG_PA_CONFIG, 0xFF);  // 20dBm
    
    // Configure DIO mapping for IRQ
    writeReg(REG_DIO_MAPPING_1, 0x00);  // DIO0: RX/TX Done
    
    // Set to RX mode
    writeReg(REG_OP_MODE, 0x85);  // RX continuous mode
    
    connected = true;
    lastPacketTime = millis();
    
    return true;
  }

  bool send(const WirelessMessage& msg) override {
    if (!connected) return false;
    
    // Set to standby mode
    writeReg(REG_OP_MODE, 0x81);
    delay(10);
    
    // Set TX mode
    writeReg(REG_OP_MODE, 0x83);  // TX mode
    
    // Set FIFO address and data
    writeReg(REG_FIFO_ADDR_PTR, 0x00);
    
    // Write packet: [Type][Length][Data...]
    writeReg(REG_FIFO, msg.type);
    writeReg(REG_FIFO, msg.length);
    
    for (int i = 0; i < msg.length; i++) {
      writeReg(REG_FIFO, msg.data[i]);
    }
    
    // Set payload length (type + length + data)
    writeReg(REG_PAYLOAD_LENGTH, msg.length + 2);
    
    // Wait for TX done (max 100ms)
    uint32_t start = millis();
    while ((readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0) {
      if (millis() - start > 100) break;
      delay(1);
    }
    
    // Clear TX done flag
    writeReg(REG_IRQ_FLAGS, 0x08);
    
    // Return to RX mode
    writeReg(REG_OP_MODE, 0x85);  // RX continuous
    
    lastPacketTime = millis();
    return true;
  }

  bool receive(WirelessMessage& msg) override {
    if (!connected) return false;
    
    // Check for RX done
    uint8_t flags = readReg(REG_IRQ_FLAGS);
    if ((flags & IRQ_RX_DONE) == 0) {
      return false;  // No packet received
    }
    
    // Clear RX done flag
    writeReg(REG_IRQ_FLAGS, 0x40);
    
    // Get payload length
    uint8_t payloadLen = readReg(REG_PAYLOAD_LENGTH);
    
    if (payloadLen < 2 || payloadLen > 254) {
      return false;  // Invalid length
    }
    
    // Read from FIFO
    writeReg(REG_FIFO_ADDR_PTR, 0x00);
    
    uint8_t msgType = readReg(REG_FIFO);
    uint8_t dataLen = readReg(REG_FIFO);
    
    if (dataLen > 64) {
      return false;  // Data too long
    }
    
    msg.type = msgType;
    msg.length = dataLen;
    
    for (int i = 0; i < dataLen; i++) {
      msg.data[i] = readReg(REG_FIFO);
    }
    
    lastPacketTime = millis();
    
    // Return to RX mode
    writeReg(REG_OP_MODE, 0x85);
    
    return true;
  }

  bool isConnected() const override {
    return connected;
  }

  int8_t getRSSI() const override {
    // RSSI = -137 + readReg(0x1B)
    // Requires reading register, simplified here
    return -100;  // Placeholder
  }

  const char* getProtocolName() const override {
    return "LoRa (SX1276/RFM95W)";
  }

  void update() override {
    // Check if module is still responding
    if ((millis() - lastPacketTime) > 30000) {
      // Consider disconnected after 30 seconds of silence
      // Try to re-sync
      uint8_t version = readReg(0x42);
      if (version == 0x12 || version != 0xFF) {
        lastPacketTime = millis();
      }
    }
  }

private:
  void setFrequency(uint32_t freq) {
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeReg(REG_FRF_LSB, (uint8_t)frf);
  }

  void writeReg(uint8_t reg, uint8_t value) {
    digitalWrite(csPin, LOW);
    SPI.transfer(reg | 0x80);  // Set write bit
    SPI.transfer(value);
    digitalWrite(csPin, HIGH);
  }

  uint8_t readReg(uint8_t reg) {
    digitalWrite(csPin, LOW);
    SPI.transfer(reg & 0x7F);  // Clear write bit
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    return value;
  }
};

#endif // LORA_DRIVER_H
