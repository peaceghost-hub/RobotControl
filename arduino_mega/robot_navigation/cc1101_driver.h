#ifndef CC1101_DRIVER_H
#define CC1101_DRIVER_H

#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "globals.h"
#include "wireless_interface.h"

/**
 * CC1101 SPI Driver — NON-BLOCKING, ISR-SAFE
 *
 * Design rules (per state-machine blueprint):
 *   1. Every SPI transaction is wrapped in noInterrupts()/interrupts()
 *      so the I2C ISR can never fire mid-transfer.
 *   2. pollReceive() reads AT MOST one packet per call.
 *   3. Every receive path ends with SIDLE → SFRX → SetRx to keep the
 *      FIFO clean (same pattern as the proven standalone receiver).
 *   4. No while-loops, no delay(), no blocking calls.
 *
 * Hardware connections (Arduino Mega):
 *   CSN  (Pin 53) → Module CSN
 *   SCK  (Pin 52) → Module SCK
 *   MOSI (Pin 51) → Module MOSI
 *   MISO (Pin 50) → Module MISO
 *   GDO0 (Pin 2)  → Module GDO0
 *   GDO2 (Pin 3)  → Module GDO2 (optional)
 *   GND → GND,  VCC → 3.3 V (NOT 5 V!)
 */
class CC1101Driver : public WirelessInterface {
private:
  bool initialized;
  byte rxBuffer[64];
  unsigned long lastRxTime;

  // Radio configuration (must match ESP8266 transmitter)
  static constexpr float FREQUENCY  = CC1101_FREQUENCY;  // 433.00 MHz
  static constexpr float DATA_RATE  = CC1101_DATA_RATE;   // 9.6 kBaud
  static const uint8_t  MODULATION  = 0;       // 2-FSK
  static const uint16_t RX_BW       = 325;     // kHz
  static const uint8_t  PA_POWER    = 10;      // +10 dBm
  static const uint8_t  SYNC_MODE   = 2;       // 16-bit sync word
  static const uint16_t SYNC_WORD   = 0xD391;  // 211, 145
  static const uint8_t  CRC_MODE    = 1;       // CRC enabled
  static const uint8_t  LENGTH_CFG  = 1;       // Variable packet length
  static constexpr float DEVIATION  = 47.60f;  // kHz

  static const uint32_t CONNECT_TIMEOUT = 3000; // ms — link-lost threshold

public:
  CC1101Driver();
  virtual ~CC1101Driver();

  // ---- WirelessInterface overrides ----
  bool  begin() override;
  bool  send(const WirelessMessage& msg) override;
  bool  receive(WirelessMessage& msg) override;  // single packet, non-blocking
  bool  isConnected() const override;
  int8_t getRSSI() const override;
  const char* getProtocolName() const override { return "CC1101"; }
  void  update() override;                       // intentionally empty

  // ---- Timing helpers (used by state machine) ----
  unsigned long getLastRxTime() const { return lastRxTime; }

private:
  void configureModule();   // sets all radio params (called once)
  void resetRxPipeline();   // SIDLE → SFRX → SetRx (interrupt-safe)
};

#endif