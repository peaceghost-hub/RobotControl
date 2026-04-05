/*
 * ESP8266 (NodeMCU) + ADS1115 + CC1101 — Dual-Joystick Transmitter
 *
 * Stick layout:
 *   - Primary stick (direction): ADS1115 A0 = VRX, A2 = VRY
 *   - Secondary stick (speed):   ADS1115 A1 = VRX, A3 = VRY
 *   - Secondary stick switch:    D8 (reverse toggle)
 *
 * The radio packet format stays the same:
 *   throttle + steer + flags + crc
 *
 * This keeps the Mega raw-motor path and the USB serial bridge compatible.
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
const uint8_t PIN_SPEED_BTN = D8;

const uint8_t CH_DIR_X   = 0;  // Primary joystick VRX
const uint8_t CH_SPEED_X = 1;  // Secondary joystick VRX
const uint8_t CH_DIR_Y   = 2;  // Primary joystick VRY
const uint8_t CH_SPEED_Y = 3;  // Secondary joystick VRY

const uint8_t CALIBRATION_SAMPLES = 20;
const uint8_t LOOP_SAMPLES = 3;

float frequency = 433.00;
bool reverseToggle = false;

struct Packet {
  int16_t throttle; 
  int16_t steer;    
  uint8_t flags;    
  uint8_t crc;
} pkt;

// DEFAULT Centers (Will be overwritten by setup calibration)
int16_t dirXCenter = 16384;
int16_t dirYCenter = 16384;
int16_t speedXCenter = 16384;
int16_t speedYCenter = 16384;

const int32_t deadband = 600; // Shared joystick deadzone

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

int16_t computeSpeedMagnitude(int16_t speedX, int16_t speedY) {
  return max(abs(speedX), abs(speedY));
}

int16_t scaleAxisByMagnitude(int16_t axis, int16_t magnitude) {
  long scaled = ((long)axis * (long)magnitude) / 255L;
  return (int16_t)constrain(scaled, -255L, 255L);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nESP8266 Dual-Joystick TX - Auto Calibrating...");
  
  pinMode(PIN_SPEED_BTN, INPUT_PULLUP);

  Wire.begin(D3, D4); 
  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 not found!");
    while (1) delay(1000);
  }
  ads.setGain(GAIN_ONE); 
  ads.setDataRate(RATE_ADS1115_250SPS);

  // --- AUTO CALIBRATION ---
  // Ensure user is NOT touching either joystick now
  Serial.println("DO NOT TOUCH EITHER JOYSTICK");
  delay(500);
  dirXCenter   = readAveragedAds(CH_DIR_X, CALIBRATION_SAMPLES);
  speedXCenter = readAveragedAds(CH_SPEED_X, CALIBRATION_SAMPLES);
  dirYCenter   = readAveragedAds(CH_DIR_Y, CALIBRATION_SAMPLES);
  speedYCenter = readAveragedAds(CH_SPEED_Y, CALIBRATION_SAMPLES);
  
  Serial.print("Calibrated Centers -> DirX: ");
  Serial.print(dirXCenter);
  Serial.print(" DirY: ");
  Serial.print(dirYCenter);
  Serial.print(" SpdX: ");
  Serial.print(speedXCenter);
  Serial.print(" SpdY: ");
  Serial.println(speedYCenter);
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
  int32_t dirXRaw   = readAveragedAds(CH_DIR_X, LOOP_SAMPLES);
  int32_t speedXRaw = readAveragedAds(CH_SPEED_X, LOOP_SAMPLES);
  int32_t dirYRaw   = readAveragedAds(CH_DIR_Y, LOOP_SAMPLES);
  int32_t speedYRaw = readAveragedAds(CH_SPEED_Y, LOOP_SAMPLES);

  // Button logic
  static bool lastBtn = HIGH;
  bool btn = (digitalRead(PIN_SPEED_BTN) == LOW);
  if (btn != lastBtn) {
    lastBtn = btn;
    if (btn) reverseToggle = !reverseToggle;
  }

  int16_t dirX = mapAdsToSigned255(dirXRaw, dirXCenter);
  int16_t dirY = mapAdsToSigned255(dirYRaw, dirYCenter);
  int16_t speedX = mapAdsToSigned255(speedXRaw, speedXCenter);
  int16_t speedY = mapAdsToSigned255(speedYRaw, speedYCenter);

  // If your direction stick is mounted differently, invert here:
  // dirX = -dirX;
  // dirY = -dirY;

  // Secondary stick controls speed magnitude only. Any deflection of that
  // stick increases speed, while the primary stick supplies the direction
  // vector (forward/reverse + steer).
  int16_t speedMagnitude = computeSpeedMagnitude(speedX, speedY);
  int16_t directionMagnitude = max(abs(dirX), abs(dirY));

  int16_t throttle = 0;
  int16_t steer = 0;
  if (speedMagnitude > 0 && directionMagnitude > 0) {
    throttle = scaleAxisByMagnitude(dirY, speedMagnitude);
    steer = scaleAxisByMagnitude(dirX, speedMagnitude);
  }

  // Preserve the original reverse-toggle behavior on the joystick switch.
  // This is optional now that the direction stick can command reverse
  // directly, but it remains useful for quick inversion if desired.
  if (reverseToggle) {
    throttle = -throttle;
  }

  if (abs(throttle) < 10) throttle = 0;
  if (abs(steer) < 10) steer = 0;

  pkt.throttle = throttle;
  pkt.steer    = steer;
  pkt.flags    = (reverseToggle ? 0x01 : 0x00) | (btn ? 0x02 : 0x00);
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

  // Debug (USB serial bridge still keys off the Thr/Str prefix)
  Serial.print("Thr: "); Serial.print(throttle);
  Serial.print(" Str: "); Serial.print(steer);
  Serial.print(" DirX: "); Serial.print(dirXRaw);
  Serial.print(" DirY: "); Serial.print(dirYRaw);
  Serial.print(" SpdX: "); Serial.print(speedXRaw);
  Serial.print(" SpdY: "); Serial.print(speedYRaw);
  Serial.print(" Mag: "); Serial.print(speedMagnitude);
  Serial.print(" Rev: "); Serial.print(reverseToggle ? 1 : 0);
  Serial.print(" Btn: "); Serial.println(btn ? 1 : 0);

  delay(20);
}
