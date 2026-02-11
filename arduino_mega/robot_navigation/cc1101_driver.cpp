#include "cc1101_driver.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "globals.h"

// =====================================================================
//  CC1101 Driver — NON-BLOCKING, ISR-SAFE implementation
//
//  Every SPI bus access is wrapped in noInterrupts()/interrupts() so
//  the I2C slave ISR cannot fire mid-transfer and corrupt the CC1101.
//  At most ONE packet is read per call to receive().
// =====================================================================

CC1101Driver::CC1101Driver() : initialized(false), lastRxTime(0) {
}

CC1101Driver::~CC1101Driver() {
  if (initialized) {
    noInterrupts();
    ELECHOUSE_cc1101.goSleep();
    interrupts();
  }
}

// ---- begin() — one-time radio initialisation ----
bool CC1101Driver::begin() {
  if (initialized) return true;

  Serial.println(F("# CC1101: Initializing SPI driver..."));

  // Set SPI pin mapping (Mega: SCK=52, MISO=50, MOSI=51, SS=53)
  ELECHOUSE_cc1101.setSpiPin(52, 50, 51, CC1101_CS_PIN);
  ELECHOUSE_cc1101.setGDO(CC1101_GDO0_PIN, CC1101_GDO2_PIN);

  // Full radio configuration (SPI-heavy — guard it)
  noInterrupts();
  configureModule();
  ELECHOUSE_cc1101.SetRx();
  interrupts();

  initialized = true;
  Serial.println(F("# CC1101: Ready (RX mode)"));
  return true;
}

// ---- update() — intentionally empty ----
// All work happens in receive() called by the main state machine.
void CC1101Driver::update() {
  // Nothing — by design
}

// ---- send() — transmit a WirelessMessage (non-blocking, rate-limited) ----
bool CC1101Driver::send(const WirelessMessage& msg) {
  if (!initialized) return false;

  // Rate-limit to avoid hammering SPI while Pi or sensors also need the bus
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 50) return false;

  noInterrupts();
  ELECHOUSE_cc1101.SendData((byte*)&msg, msg.length + 2);
  interrupts();

  lastSendTime = millis();
  return true;
}

// ---- receive() — read ONE packet, non-blocking ----
//
// Returns true if a valid RawMotorPacket (or WirelessMessage) was decoded.
// Always resets the RX pipeline before returning so the FIFO stays clean.
bool CC1101Driver::receive(WirelessMessage& msg) {
  if (!initialized) return false;

  // --- 1. Check if anything is in the RX FIFO (fast SPI read) ---
  noInterrupts();
  bool hasData = ELECHOUSE_cc1101.CheckRxFifo(0);
  interrupts();

  if (!hasData) return false;

  // --- 2. Read the packet out of the FIFO ---
  byte size;
  noInterrupts();
  size = ELECHOUSE_cc1101.ReceiveData(rxBuffer);
  interrupts();

  if (size == 0 || size > sizeof(rxBuffer)) {
    // Empty or oversized → flush and restart
    resetRxPipeline();
    return false;
  }

  // --- 3. Try to decode as RawMotorPacket (6 bytes from ESP8266) ---
  RawMotorPacket* pktPtr = nullptr;

  if (size == sizeof(RawMotorPacket)) {
    pktPtr = (RawMotorPacket*)rxBuffer;
  } else if (size == sizeof(RawMotorPacket) + 1) {
    // Library sometimes prepends the length byte
    pktPtr = (RawMotorPacket*)&rxBuffer[1];
  }

  if (pktPtr != nullptr) {
    // Validate CRC: sum of first 5 bytes == crc byte
    uint8_t sum = 0;
    const uint8_t* b = (const uint8_t*)pktPtr;
    for (uint8_t i = 0; i < sizeof(RawMotorPacket) - 1; i++) sum += b[i];

    if (pktPtr->crc == sum) {
      msg.type   = MSG_TYPE_RAW_MOTOR;
      msg.length = sizeof(RawMotorPacket);
      memcpy(msg.data, (const uint8_t*)pktPtr, sizeof(RawMotorPacket));
      lastRxTime = millis();
      resetRxPipeline();
      return true;
    }
    // CRC mismatch — fall through to flush
  }

  // --- 4. Try as standard WirelessMessage [type][length][data…] ---
  if (size >= 2) {
    msg.type   = rxBuffer[0];
    msg.length = rxBuffer[1];
    if (msg.length <= 62 && size >= (msg.length + 2)) {
      memcpy(msg.data, &rxBuffer[2], msg.length);
      lastRxTime = millis();
      resetRxPipeline();
      return true;
    }
  }

  // --- 5. Unrecognised / garbage — flush ---
  resetRxPipeline();
  return false;
}

// ---- isConnected() ----
bool CC1101Driver::isConnected() const {
  return initialized && (millis() - lastRxTime < CONNECT_TIMEOUT);
}

// ---- getRSSI() ----
int8_t CC1101Driver::getRSSI() const {
  if (!initialized) return -127;
  noInterrupts();
  int8_t rssi = ELECHOUSE_cc1101.getRssi();
  interrupts();
  return rssi;
}

// ---- configureModule() — set all radio parameters (call with ints disabled) ----
void CC1101Driver::configureModule() {
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setModulation(MODULATION);
  ELECHOUSE_cc1101.setMHZ(FREQUENCY);
  ELECHOUSE_cc1101.setRxBW(RX_BW);
  ELECHOUSE_cc1101.setDeviation(DEVIATION);
  ELECHOUSE_cc1101.setPA(PA_POWER);
  ELECHOUSE_cc1101.setSyncMode(SYNC_MODE);
  ELECHOUSE_cc1101.setSyncWord(SYNC_WORD >> 8, SYNC_WORD & 0xFF); // 211, 145
  ELECHOUSE_cc1101.setCrc(CRC_MODE);
  ELECHOUSE_cc1101.setDcFilterOff(0);
  ELECHOUSE_cc1101.setManchester(0);
  ELECHOUSE_cc1101.setPktFormat(0);
  ELECHOUSE_cc1101.setWhiteData(0);
  ELECHOUSE_cc1101.setLengthConfig(LENGTH_CFG);
  ELECHOUSE_cc1101.setDRate(DATA_RATE);
}

// ---- resetRxPipeline() — SIDLE → SFRX → SetRx (call any time) ----
//  Guards SPI with noInterrupts().  Cheap: ~20 µs total.
void CC1101Driver::resetRxPipeline() {
  noInterrupts();
  ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
  // No delayMicroseconds needed — SPI strobe takes ~4 µs each
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX
  ELECHOUSE_cc1101.SetRx();
  interrupts();
}