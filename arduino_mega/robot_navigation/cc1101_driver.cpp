#include "cc1101_driver.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "globals.h"

CC1101Driver::CC1101Driver() : initialized(false), lastRxTime(0) {
}

CC1101Driver::~CC1101Driver() {
  if (initialized) {
    ELECHOUSE_cc1101.goSleep();
  }
}

bool CC1101Driver::begin() {
  if (initialized) return true;

  Serial.println(F("Initializing CC1101 SPI driver..."));

  // Initialize SPI pins (Arduino Mega defaults)
  // setSpiPin(sck, miso, mosi, ss)
  ELECHOUSE_cc1101.setSpiPin(52, 50, 51, CC1101_CS_PIN);
  ELECHOUSE_cc1101.setGDO(CC1101_GDO0_PIN, CC1101_GDO2_PIN);

  // Initialize module
  if (!initModule()) {
    Serial.println(F("CC1101 initialization failed"));
    return false;
  }

  initialized = true;
  Serial.println(F("CC1101 driver initialized successfully"));
  return true;
}

void CC1101Driver::update() {
  if (!initialized) return;

  // Check for received data
  if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
    byte size = ELECHOUSE_cc1101.ReceiveData(rxBuffer);
    if (size > 0) {
      handleReceivedData(rxBuffer, size);
      lastRxTime = millis();
    }
  }

  // Reset connection if no data received for too long
  if (millis() - lastRxTime > CONNECT_TIMEOUT) {
    // Note: We don't reset here as it might interrupt ongoing operations
    // Connection status is checked via isConnected()
  }
}

bool CC1101Driver::send(const WirelessMessage& msg) {
  if (!initialized) return false;

  // Send data (length + data)
  ELECHOUSE_cc1101.SendData((byte*)&msg, msg.length + 2); // +2 for type and length bytes

  return true;
}

bool CC1101Driver::receive(WirelessMessage& msg) {
  if (!initialized) return false;

  // Check if data is available
  if (!ELECHOUSE_cc1101.CheckReceiveFlag()) {
    return false;
  }

  // Receive data
  byte size = ELECHOUSE_cc1101.ReceiveData(rxBuffer);
  if (size < 2) return false; // Need at least type and length

  // Parse message
  msg.type = rxBuffer[0];
  msg.length = rxBuffer[1];

  if (msg.length > 64) msg.length = 64; // Safety check
  if (size - 2 < msg.length) msg.length = size - 2; // Adjust if received less

  memcpy(msg.data, &rxBuffer[2], msg.length);

  lastRxTime = millis();
  return true;
}

bool CC1101Driver::isConnected() const {
  return initialized && (millis() - lastRxTime < CONNECT_TIMEOUT);
}

int8_t CC1101Driver::getRSSI() const {
  if (!initialized) return -127; // Invalid RSSI value

  // Get RSSI from CC1101 (this is approximate)
  return ELECHOUSE_cc1101.getRssi();
}

void CC1101Driver::configureModule() {
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setCCMode(1);       // High sensitivity mode
  ELECHOUSE_cc1101.setModulation(MODULATION);     // 2-FSK
  ELECHOUSE_cc1101.setMHZ(FREQUENCY);             // Frequency
  ELECHOUSE_cc1101.setRxBW(RX_BW);                // Receive bandwidth
  ELECHOUSE_cc1101.setDeviation(DEVIATION);       // Frequency deviation
  ELECHOUSE_cc1101.setPA(PA_POWER);               // Transmission power
  ELECHOUSE_cc1101.setSyncMode(SYNC_MODE);        // Sync word mode
  ELECHOUSE_cc1101.setSyncWord(SYNC_WORD, false); // Sync word
  ELECHOUSE_cc1101.setCrc(CRC_MODE);              // CRC enabled
  ELECHOUSE_cc1101.setPktFormat(PKT_FORMAT);      // Packet format
  ELECHOUSE_cc1101.setLengthConfig(LENGTH_CONFIG); // Variable length
  ELECHOUSE_cc1101.setDRate(DATA_RATE);           // Data rate
}

bool CC1101Driver::initModule() {
  configureModule();

  // Test communication
  if (!ELECHOUSE_cc1101.getCC1101()) {
    Serial.println(F("CC1101 not found"));
    return false;
  }

  // Set receive mode
  ELECHOUSE_cc1101.SetRx();

  return true;
}

void CC1101Driver::handleReceivedData(byte* data, byte length) {
  // This method is kept for backward compatibility but not used
  // since receive() handles data retrieval
}