#ifndef CC1101_DRIVER_H
#define CC1101_DRIVER_H

#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "globals.h"
#include "wireless_interface.h"

/**
 * CC1101 SPI Driver
 *
 * This driver is for CC1101 transceiver modules using SPI interface.
 * Supports 315/433/868/915MHz ISM bands with configurable parameters.
 *
 * Hardware connections (Arduino Mega):
 *   CSN (Pin 53) → Module CSN
 *   SCK (Pin 52) → Module SCK
 *   MOSI (Pin 51) → Module MOSI
 *   MISO (Pin 50) → Module MISO
 *   GDO0 (Pin 2) → Module GDO0 (interrupt)
 *   GDO2 (Pin 3) → Module GDO2 (optional)
 *   GND → GND
 *   VCC → 3.3V (IMPORTANT: NOT 5V!)
 */
class CC1101Driver : public WirelessInterface {
private:
  bool initialized;
  byte rxBuffer[64];
  unsigned long lastRxTime;
  static const uint32_t CONNECT_TIMEOUT = 5000;

  // CC1101 Configuration (must match between transmitter and receiver)
  static constexpr float FREQUENCY = CC1101_FREQUENCY;      // MHz
  static constexpr float DATA_RATE = CC1101_DATA_RATE;      // kBaud
  static const uint8_t MODULATION = 0;        // 2-FSK
  static const uint8_t RX_BW = 325;           // 325 kHz
  static const float DEVIATION = 47.60;       // kHz
  static const uint8_t PA_POWER = 10;         // +10 dBm
  static const uint8_t SYNC_MODE = 2;         // 16-bit sync word
  static const uint16_t SYNC_WORD = 0xD191;   // 211, 145 (must match!)
  static const uint8_t CRC_MODE = 1;          // CRC enabled
  static const uint8_t PKT_FORMAT = 0;        // Normal mode
  static const uint8_t LENGTH_CONFIG = 1;     // Variable packet length

public:
  CC1101Driver();
  virtual ~CC1101Driver();

  // WirelessInterface implementation
  bool begin() override;
  bool send(const WirelessMessage& msg) override;
  bool receive(WirelessMessage& msg) override;
  bool isConnected() const override;
  int8_t getRSSI() const override;
  const char* getProtocolName() const override { return "CC1101"; }
  void update() override;

private:
  void configureModule();
  bool initModule();
  void handleReceivedData(byte* data, byte length);
};

#endif</content>
<parameter name="filePath">/home/thewizard/RobotControl/arduino_mega/robot_navigation/cc1101_driver.h