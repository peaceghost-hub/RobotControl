/*
 * ESP8266 (NodeMCU) + ADS1115 + CC1101 — Transmitter
 * FIXES: 
 * 1. Auto-calibration at startup (Fixes drifting/moving at idle)
 * 2. Blind Send + Manual Length Byte (Fixes Soft WDT Reset crash)
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

Adafruit_ADS1115 ads; 

const uint8_t PIN_SCK  = D5; 
const uint8_t PIN_MISO = D6; 
const uint8_t PIN_MOSI = D7; 
const uint8_t PIN_CSN  = D2; 
const uint8_t PIN_GDO0 = D1; 
const uint8_t PIN_JOY_BTN = D0; 

float frequency = 433.00;
bool reverseToggle = false;

struct Packet {
  int16_t throttle; 
  int16_t steer;    
  uint8_t flags;    
  uint8_t crc;
} pkt;

// DEFAULT Centers (Will be overwritten by setup calibration)
int16_t xCenter = 16384; 
int16_t yCenter = 16384;
const int32_t deadband = 600; // Joystick deadzone

uint8_t computeCrc(const Packet& p) {
  const uint8_t* b = (const uint8_t*)&p;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(Packet) - 1; i++) sum += b[i];
  return sum;
}

int32_t readAveragedAds(uint8_t channel, uint8_t samples = 8) {
  int32_t acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += ads.readADC_SingleEnded(channel); 
  }
  return acc / samples;
}

int16_t mapAdsToSigned255(int32_t raw, int32_t center) {
  int32_t delta = raw - center;

  // If inside deadband, return 0 (STOP)
  if (abs(delta) < deadband) return 0;

  if (delta > 0) {
    int32_t span = 32767 - center - deadband;
    if (span <= 0) span = 1; // prevent divide by zero
    int32_t val = (delta - deadband) * 255L / span;
    return (int16_t)constrain(val, 0L, 255L);
  } else {
    int32_t span = center - deadband;
    if (span <= 0) span = 1;
    int32_t val = (abs(delta) - deadband) * 255L / span;
    return (int16_t)constrain(-val, -255L, 0L);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nESP8266 Joystick TX - Auto Calibrating...");
  
  pinMode(PIN_JOY_BTN, INPUT_PULLUP);

  Wire.begin(D3, D4); 
  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 not found!");
    while (1) delay(1000);
  }
  ads.setGain(GAIN_ONE); 
  ads.setDataRate(RATE_ADS1115_128SPS);

  // --- AUTO CALIBRATION ---
  // Ensure user is NOT touching the joystick now
  Serial.println("DO NOT TOUCH JOYSTICK");
  delay(500);
  xCenter = readAveragedAds(0, 30); // Take 30 samples
  yCenter = readAveragedAds(1, 30);
  
  Serial.print("Calibrated Centers -> X: ");
  Serial.print(xCenter);
  Serial.print(" Y: ");
  Serial.println(yCenter);
  Serial.println("--------------------------------");

  // CC1101 Setup
  ELECHOUSE_cc1101.setGDO(PIN_GDO0, 0); 
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  SPI.begin(); 
  ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

  if (!ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101 Error");
    while (1) delay(1000);
  }

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(frequency);
  ELECHOUSE_cc1101.setModulation(0);      
  ELECHOUSE_cc1101.setDRate(9.6);
  ELECHOUSE_cc1101.setRxBW(325);
  ELECHOUSE_cc1101.setDeviation(47.60);
  ELECHOUSE_cc1101.setPA(10);
  ELECHOUSE_cc1101.setSyncMode(2);
  ELECHOUSE_cc1101.setSyncWord(211, 145);
  ELECHOUSE_cc1101.setCrc(1);
  ELECHOUSE_cc1101.setDcFilterOff(0);
  ELECHOUSE_cc1101.setManchester(0);
  ELECHOUSE_cc1101.setPktFormat(0);
  ELECHOUSE_cc1101.setWhiteData(0);
  ELECHOUSE_cc1101.setAdrChk(0);
  ELECHOUSE_cc1101.setAddr(0);
  ELECHOUSE_cc1101.setLengthConfig(1);
}

void loop() {
  int32_t xRaw = readAveragedAds(0, 4);
  int32_t yRaw = readAveragedAds(1, 4);

  // Button logic
  static bool lastBtn = HIGH;
  bool btn = (digitalRead(PIN_JOY_BTN) == LOW);
  if (btn != lastBtn) {
    lastBtn = btn;
    if (btn) reverseToggle = !reverseToggle;
  }

  // Calculate Speed
  int16_t steer    = mapAdsToSigned255(xRaw, xCenter);
  int16_t throttle = mapAdsToSigned255(yRaw, yCenter);

  // Inverse Steer if needed (depends on your mounting)
  // steer = -steer; 

  if (reverseToggle) throttle = -throttle;

  pkt.throttle = throttle;
  pkt.steer    = steer;
  pkt.flags    = (reverseToggle ? 0x01 : 0x00);
  pkt.crc      = computeCrc(pkt);

  // --- MANUAL BLIND SEND (NO CRASH) ---
  byte len = sizeof(pkt);
  ELECHOUSE_cc1101.SpiStrobe(0x36); // IDLE
  ELECHOUSE_cc1101.SpiWriteReg(0x3F, len); // LENGTH BYTE
  ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, (uint8_t*)&pkt, len); // DATA
  ELECHOUSE_cc1101.SpiStrobe(0x35); // TRANSMIT
  delay(30); // WAIT
  ELECHOUSE_cc1101.SpiStrobe(0x36); // IDLE
  ELECHOUSE_cc1101.SpiStrobe(0x3B); // FLUSH TX
  // ------------------------------------

  // Debug (View this to confirm 0, 0 when idle)
  Serial.print("Thr: "); Serial.print(throttle);
  Serial.print(" Str: "); Serial.print(steer);
  Serial.print(" RawX: "); Serial.print(xRaw);
  Serial.print(" RawY: "); Serial.println(yRaw);

  delay(20);
}
