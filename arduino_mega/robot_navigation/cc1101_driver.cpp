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

CC1101Driver::CC1101Driver() : initialized(false), lastRxTime(0), rxGoodCount(0), rxBadCount(0),
                               rxPending(false), rxPendingSince(0) {
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
  // NOTE: Do NOT call setGDO() — working standalone code doesn't use it
  // on the Mega side. The library defaults are fine.

  // Full radio configuration — do NOT wrap in noInterrupts() because
  // Init() → Reset() calls delay() which needs Timer 0 ISR.
  // This runs once at deferred boot (5 s), no I2C contention yet.
  configureModule();

  // Flush any stale RX data and enter clean RX mode
  noInterrupts();
  ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX (flush RX FIFO)
  ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX (flush TX FIFO)
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

  // --- Safe TX: write FIFO with ints off (fast), then strobe TX and
  //     wait with ints ON so TWI / Timer ISRs can still fire. ---
  uint8_t pktLen = msg.length + 2;

  noInterrupts();
  ELECHOUSE_cc1101.SpiStrobe(0x36);                   // SIDLE
  ELECHOUSE_cc1101.SpiStrobe(0x3B);                   // SFTX  (flush TX FIFO)
  ELECHOUSE_cc1101.SpiWriteReg(0x3F, pktLen);         // length byte
  ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, (byte*)&msg, pktLen); // payload
  ELECHOUSE_cc1101.SpiStrobe(0x35);                   // STX   (start transmit)
  interrupts();                                        // ← ints back ON

  // Wait for TX to finish (GDO0 goes HIGH then LOW) — with interrupts ENABLED.
  // At 9.6 kBaud a max-size packet takes ~70 ms; timeout at 100 ms.
  unsigned long txStart = millis();
  while (digitalRead(CC1101_GDO0_PIN) == LOW) {
    if (millis() - txStart > 100) { resetRxPipeline(); return false; }
  }
  while (digitalRead(CC1101_GDO0_PIN) == HIGH) {
    if (millis() - txStart > 100) { resetRxPipeline(); return false; }
  }

  // Back to RX mode
  resetRxPipeline();

  lastSendTime = millis();
  return true;
}

// ---- receive() — read ONE packet, non-blocking ----
//
// Matches the proven standalone receiver as closely as possible:
//
//   Standalone uses:  CheckRxFifo(50)  →  ReceiveData(buf)  →  SIDLE,delay(2),SFRX,delay(2),SetRx
//   CheckRxFifo(50) blocks for 50 ms to let the packet fully arrive.
//
// We replicate that with a NON-BLOCKING two-phase approach:
//   Phase 1: first time we see data in FIFO → record timestamp, return false
//   Phase 2: on next call, if 50 ms have passed → read the completed packet
//
// We do NOT use ReceiveData() because it has no bounds check on the
// length byte — a garbage length (e.g. 200) overflows our buffer and
// corrupts rxGoodCount/rxBadCount/lastRxTime in RAM.
// Instead we read the length byte first, validate it, then read only
// that many bytes — safe, bounded, no overflow possible.
bool CC1101Driver::receive(WirelessMessage& msg) {
  if (!initialized) return false;

  // --- 1. Check RXBYTES ---
  noInterrupts();
  uint8_t rxRaw = ELECHOUSE_cc1101.SpiReadStatus(0x3B); // CC1101_RXBYTES
  interrupts();

  // Overflow — flush and restart
  if (rxRaw & 0x80) {
    rxPending = false;
    resetRxPipeline();
    return false;
  }

  uint8_t rxBytes = rxRaw & 0x7F;

  if (rxBytes == 0) {
    rxPending = false;
    return false;
  }

  // --- 2. Non-blocking wait (replaces CheckRxFifo(50)'s delay(50)) ---
  // First time seeing data → start the timer, don't read yet.
  if (!rxPending) {
    rxPending = true;
    rxPendingSince = millis();
    return false;
  }
  // Still waiting? Come back later.
  if (millis() - rxPendingSince < 50) return false;

  // 50 ms have passed — packet should be complete. Clear the flag.
  rxPending = false;

  // --- 3. Re-check RXBYTES after the wait ---
  noInterrupts();
  rxRaw = ELECHOUSE_cc1101.SpiReadStatus(0x3B);
  interrupts();

  if (rxRaw & 0x80) {
    resetRxPipeline();
    return false;
  }

  rxBytes = rxRaw & 0x7F;
  if (rxBytes == 0) return false;

  // --- 4. Manual FIFO read (SAFE — no ReceiveData buffer overflow) ---
  // Read the length byte first, validate, then read only that many bytes.
  byte pktLen;
  byte status[2];

  noInterrupts();
  pktLen = ELECHOUSE_cc1101.SpiReadReg(CC1101_RXFIFO);  // first FIFO byte = length
  interrupts();

  // Validate length: must be 1..62 and we must have enough bytes
  // FIFO should contain: 1(len, already read) + pktLen(data) + 2(status)
  if (pktLen == 0 || pktLen > 62 || rxBytes < (pktLen + 3)) {
    resetRxPipeline();
    return false;
  }

  // Read the payload + 2 status bytes
  noInterrupts();
  ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxBuffer, pktLen);
  ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, status, 2);
  interrupts();

  // --- 5. Reset radio to RX (matches standalone: SIDLE→delay→SFRX→delay→SetRx) ---
  resetRxPipeline();

  // NOTE: Do NOT check hardware CRC_OK bit (status[1] & 0x80).
  // The working standalone code does not check it — it relies solely
  // on the software checksum (computeCrc).  Hardware CRC_OK may fail
  // due to register config differences between ccmode=0 and ccmode=1.

  // --- 6. Decode as RawMotorPacket (6 bytes, matches standalone exactly) ---
  if (pktLen == sizeof(RawMotorPacket)) {
    RawMotorPacket* pkt = (RawMotorPacket*)rxBuffer;
    uint8_t sum = 0;
    const uint8_t* b = (const uint8_t*)pkt;
    for (uint8_t i = 0; i < sizeof(RawMotorPacket) - 1; i++) sum += b[i];

    if (pkt->crc == sum) {
      msg.type   = MSG_TYPE_RAW_MOTOR;
      msg.length = sizeof(RawMotorPacket);
      memcpy(msg.data, (const uint8_t*)pkt, sizeof(RawMotorPacket));
      lastRxTime = millis();
      rxGoodCount++;
      return true;
    }
    rxBadCount++;
    return false;
  }

  // --- 7. Try as standard WirelessMessage [type][length][data…] ---
  if (pktLen >= 2) {
    msg.type   = rxBuffer[0];
    msg.length = rxBuffer[1];
    if (msg.length <= 60 && pktLen >= (msg.length + 2)) {
      memcpy(msg.data, &rxBuffer[2], msg.length);
      lastRxTime = millis();
      rxGoodCount++;
      return true;
    }
  }

  // --- 8. Unrecognized ---
  rxBadCount++;
  return false;
}

// ---- ensureRxMode() — no-op ----
// Removed: reading MARCSTATE inside noInterrupts() adds freeze risk,
// and the radio is often in a non-0x0D state while receiving a packet
// (e.g. 0x11 = RX_OVERFLOW, or transitioning).  The overflow check in
// receive() handles the real failure case safely.
void CC1101Driver::ensureRxMode() {
  // intentionally empty — overflow handled in receive()
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

// ---- configureModule() — set all radio parameters ----
// Called once during deferred init with interrupts ENABLED
// (Init() internally calls delay() which needs Timer 0 ISR).
void CC1101Driver::configureModule() {
  ELECHOUSE_cc1101.Init();
  // NOTE: Do NOT call setCCMode(1) — it changes MDMCFG3/MDMCFG4/PKTCTRL0
  // which alters the effective data rate and packet format.
  // The proven working standalone code does NOT call setCCMode.
  // Init() → RegConfigSettings() → setCCMode(0) sets the correct defaults.
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
  ELECHOUSE_cc1101.setAdrChk(0);         // no address filtering (match ESP8266)
  ELECHOUSE_cc1101.setAddr(0);           // address 0 (match ESP8266)
  ELECHOUSE_cc1101.setLengthConfig(LENGTH_CFG);
  ELECHOUSE_cc1101.setDRate(DATA_RATE);
}

// ---- resetRxPipeline() — SIDLE → SFRX → SetRx (call any time) ----
//  Guards SPI with noInterrupts().  Cheap: ~20 µs total.
void CC1101Driver::resetRxPipeline() {
  noInterrupts();
  ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
  delayMicroseconds(100);            // let CC1101 reach IDLE (matches standalone's delay(2))
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX
  delayMicroseconds(100);            // let flush complete
  ELECHOUSE_cc1101.SetRx();
  interrupts();
}