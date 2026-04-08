/*
 * ESP8266 (NodeMCU) + ADS1115 + CC1101 — Dual Joystick (Tank Drive) TX
 *
 * This transmitter dedicates one joystick to each wheel:
 *   - Joystick 1 drives the RIGHT wheel
 *   - Joystick 2 drives the LEFT wheel
 *
 * The wireless packet remains 6 bytes wide so the existing CC1101 / I2C
 * transport stays compatible, but the two int16 fields now represent direct
 * wheel speeds instead of throttle/steer.
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
const uint8_t PIN_JOY1_BTN = D0;
const uint8_t PIN_JOY2_BTN = D8;

const uint8_t CALIBRATION_SAMPLES = 20;
const uint8_t LOOP_SAMPLES = 3;
const unsigned long BUTTON_DEBOUNCE_MS = 80;
const int16_t TOGGLE_NEUTRAL_THRESHOLD = 25;
const unsigned long TOGGLE_NEUTRAL_HOLD_MS = 120;
const int32_t CENTER_TRACK_WINDOW = 1400;
const uint8_t OPPOSITE_SIGN_CONFIRM_FRAMES = 3;
const int16_t STRONG_DRIVE_THRESHOLD = 90;

// Per-axis gain trim to compensate for asymmetric joystick travel around
// the calibrated center. Negative travel on both tank-drive axes already
// reaches full scale on this hardware, while positive travel tops out early.
static const float JOY1_POS_GAIN = 1.76f;
static const float JOY1_NEG_GAIN = 1.00f;
static const float JOY2_POS_GAIN = 1.28f;
static const float JOY2_NEG_GAIN = 1.00f;

float frequency = 433.00;
bool reverseLeft = false;
bool reverseRight = false;
bool btn1StableState = false;
bool btn2StableState = false;
bool btn1PressArmed = false;
bool btn2PressArmed = false;
unsigned long neutralSinceMs = 0;
int8_t leftPendingSign = 0;
int8_t rightPendingSign = 0;
uint8_t leftPendingCount = 0;
uint8_t rightPendingCount = 0;
int16_t lastStableLeft = 0;
int16_t lastStableRight = 0;

struct Packet {
  int16_t leftSpeed;
  int16_t rightSpeed;
  uint8_t flags;    
  uint8_t crc;
} pkt;

// Calibration centers
int16_t x1Center = 16384;
int16_t y1Center = 16384;
int16_t x2Center = 16384;
int16_t y2Center = 16384;
const int32_t deadband = 600;

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

int16_t applyDirectionalGain(int16_t value, float negativeGain, float positiveGain) {
  if (value > 0) {
    return (int16_t)constrain((long)roundf(value * positiveGain), 0L, 255L);
  }
  if (value < 0) {
    return (int16_t)constrain((long)roundf(value * negativeGain), -255L, 0L);
  }
  return 0;
}

int16_t stabilizeAxisDirection(int16_t candidate, int16_t& lastStable, int8_t& pendingSign, uint8_t& pendingCount) {
  const int16_t candidateAbs = abs(candidate);
  const int16_t lastAbs = abs(lastStable);

  if (candidateAbs <= TOGGLE_NEUTRAL_THRESHOLD) {
    pendingSign = 0;
    pendingCount = 0;
    lastStable = 0;
    return 0;
  }

  if (lastStable == 0 || candidateAbs < STRONG_DRIVE_THRESHOLD || lastAbs < STRONG_DRIVE_THRESHOLD) {
    pendingSign = 0;
    pendingCount = 0;
    lastStable = candidate;
    return candidate;
  }

  const bool sameSign = (candidate > 0 && lastStable > 0) || (candidate < 0 && lastStable < 0);
  if (sameSign) {
    pendingSign = 0;
    pendingCount = 0;
    lastStable = candidate;
    return candidate;
  }

  const int8_t desiredSign = (candidate > 0) ? 1 : -1;
  if (pendingSign != desiredSign) {
    pendingSign = desiredSign;
    pendingCount = 1;
    return lastStable;
  }

  if (pendingCount < 255) pendingCount++;
  if (pendingCount < OPPOSITE_SIGN_CONFIRM_FRAMES) {
    return lastStable;
  }

  pendingSign = 0;
  pendingCount = 0;
  lastStable = candidate;
  return candidate;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nESP8266 Tank Drive TX - Auto Calibrating...");
  
  pinMode(PIN_JOY1_BTN, INPUT_PULLUP);
  pinMode(PIN_JOY2_BTN, INPUT);

  Wire.begin(D3, D4); 
  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 not found!");
    while (1) delay(1000);
  }
  ads.setGain(GAIN_ONE); 
  ads.setDataRate(RATE_ADS1115_250SPS);

  Serial.println("DO NOT TOUCH JOYSTICKS");
  delay(500);
  x1Center = readAveragedAds(0, CALIBRATION_SAMPLES);
  y1Center = readAveragedAds(1, CALIBRATION_SAMPLES);
  x2Center = readAveragedAds(2, CALIBRATION_SAMPLES);
  y2Center = readAveragedAds(3, CALIBRATION_SAMPLES);
  
  Serial.println("Calibration Complete.");
  Serial.println("--------------------------------");

  btn1StableState = (digitalRead(PIN_JOY1_BTN) == LOW);
  btn2StableState = (digitalRead(PIN_JOY2_BTN) == HIGH);
  neutralSinceMs = millis();

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
  // NOTE: Do NOT call setCCMode — it changes data rate registers.
  // The proven working code does NOT call setCCMode.
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

  Serial.println("CC1101 Ready - TX Mode");
}

void loop() {
  int32_t y1Raw = readAveragedAds(1, LOOP_SAMPLES + 1);
  int32_t y2Raw = readAveragedAds(3, LOOP_SAMPLES + 1);

  if (abs(y1Raw - y1Center) <= CENTER_TRACK_WINDOW) {
    y1Center = (int16_t)(((int32_t)y1Center * 31 + y1Raw) / 32);
  }
  if (abs(y2Raw - y2Center) <= CENTER_TRACK_WINDOW) {
    y2Center = (int16_t)(((int32_t)y2Center * 31 + y2Raw) / 32);
  }

  int16_t joy1Throttle = mapAdsToSigned255(y1Raw, y1Center);
  int16_t joy2Throttle = mapAdsToSigned255(y2Raw, y2Center);

  joy1Throttle = applyDirectionalGain(joy1Throttle, JOY1_NEG_GAIN, JOY1_POS_GAIN);
  joy2Throttle = applyDirectionalGain(joy2Throttle, JOY2_NEG_GAIN, JOY2_POS_GAIN);

  const bool joysticksNeutral =
      abs(joy1Throttle) <= TOGGLE_NEUTRAL_THRESHOLD &&
      abs(joy2Throttle) <= TOGGLE_NEUTRAL_THRESHOLD;

  const unsigned long now = millis();
  if (joysticksNeutral) {
    if (neutralSinceMs == 0) neutralSinceMs = now;
  } else {
    neutralSinceMs = 0;
    btn1PressArmed = false;
    btn2PressArmed = false;
  }
  const bool neutralLongEnough =
      neutralSinceMs > 0 && (now - neutralSinceMs) >= TOGGLE_NEUTRAL_HOLD_MS;

  static bool lastBtn1Raw = false;
  static unsigned long lastBtn1ChangeMs = 0;
  const bool btn1Raw = (digitalRead(PIN_JOY1_BTN) == LOW);
  if (btn1Raw != lastBtn1Raw) {
    lastBtn1Raw = btn1Raw;
    lastBtn1ChangeMs = now;
  } else if (btn1Raw != btn1StableState && (now - lastBtn1ChangeMs) >= BUTTON_DEBOUNCE_MS) {
    btn1StableState = btn1Raw;
    if (btn1StableState) {
      btn1PressArmed = neutralLongEnough;
    } else if (btn1PressArmed && neutralLongEnough) {
      reverseLeft = !reverseLeft;
      btn1PressArmed = false;
      Serial.print("ReverseLeft: ");
      Serial.println(reverseLeft ? "ON" : "OFF");
    } else {
      btn1PressArmed = false;
    }
  }

  static bool lastBtn2Raw = false;
  static unsigned long lastBtn2ChangeMs = 0;
  const bool btn2Raw = (digitalRead(PIN_JOY2_BTN) == HIGH);
  if (btn2Raw != lastBtn2Raw) {
    lastBtn2Raw = btn2Raw;
    lastBtn2ChangeMs = now;
  } else if (btn2Raw != btn2StableState && (now - lastBtn2ChangeMs) >= BUTTON_DEBOUNCE_MS) {
    btn2StableState = btn2Raw;
    if (btn2StableState) {
      btn2PressArmed = neutralLongEnough;
    } else if (btn2PressArmed && neutralLongEnough) {
      reverseRight = !reverseRight;
      btn2PressArmed = false;
      Serial.print("ReverseRight: ");
      Serial.println(reverseRight ? "ON" : "OFF");
    } else {
      btn2PressArmed = false;
    }
  }

  int16_t leftThrottle = joy1Throttle;
  int16_t rightThrottle = joy2Throttle;

  // Swap joysticks: joystick 1 drives the right wheel, joystick 2 drives the left wheel.
  int16_t temp = leftThrottle;
  leftThrottle = rightThrottle;
  rightThrottle = temp;

  // Preserve the tested wheel orientation mapping.
  leftThrottle = -leftThrottle;

  if (reverseLeft) leftThrottle = -leftThrottle;
  if (reverseRight) rightThrottle = -rightThrottle;

  leftThrottle = stabilizeAxisDirection(leftThrottle, lastStableLeft, leftPendingSign, leftPendingCount);
  rightThrottle = stabilizeAxisDirection(rightThrottle, lastStableRight, rightPendingSign, rightPendingCount);

  pkt.leftSpeed = leftThrottle;
  pkt.rightSpeed = rightThrottle;
  pkt.flags = 0;
  if (reverseLeft) pkt.flags |= 0x01;
  if (reverseRight) pkt.flags |= 0x02;
  pkt.crc      = computeCrc(pkt);

  // --- MANUAL BLIND SEND (NO CRASH) ---
  byte len = sizeof(pkt);
  ELECHOUSE_cc1101.SpiStrobe(0x36); // IDLE
  ELECHOUSE_cc1101.SpiStrobe(0x3B); // FLUSH TX (clear any leftover data)
  ELECHOUSE_cc1101.SpiWriteReg(0x3F, len); // LENGTH BYTE
  ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, (uint8_t*)&pkt, len); // DATA
  ELECHOUSE_cc1101.SpiStrobe(0x35); // TRANSMIT
  delay(30); // WAIT for TX to complete (~12ms at 9.6kBaud for 6-byte packet)
  ELECHOUSE_cc1101.SpiStrobe(0x36); // IDLE
  ELECHOUSE_cc1101.SpiStrobe(0x3B); // FLUSH TX
  // ------------------------------------

  Serial.print("Left: "); Serial.print(pkt.leftSpeed);
  Serial.print(" | Right: "); Serial.println(pkt.rightSpeed);

  delay(20);
}
