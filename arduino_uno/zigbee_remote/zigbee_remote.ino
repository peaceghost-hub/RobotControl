/*
 * Arduino Mega + CC1101 Receiver -> L298N
 * FIXES: Buffer safety, robust startup.
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define PIN_GDO0 2
#define PIN_CSN  53

// L298N
#define IN1 22
#define IN2 23
#define ENA 5   // PWM (Left Side)
#define IN3 24
#define IN4 25
#define ENB 6   // PWM (Right Side)

float frequency = 433.00;
unsigned long lastPacketMs = 0;

struct Packet {
  int16_t throttle; 
  int16_t steer;    
  uint8_t flags;    
  uint8_t crc;
} pkt;

uint8_t computeCrc(const Packet& p) {
  const uint8_t* b = (const uint8_t*)&p;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(Packet) - 1; i++) sum += b[i];
  return sum;
}

void setMotor(int speed, uint8_t inA, uint8_t inB, uint8_t enPWM) {
  int s = constrain(speed, -255, 255);
  
  // Deadzone at receiver end (optional, helps eliminate buzzing)
  if (abs(s) < 15) s = 0;

  if (s > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enPWM, s);
  } else if (s < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(enPWM, -s); // PWM must be positive
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(enPWM, 0);
  }
}

void stopMotors() {
  setMotor(0, IN1, IN2, ENA);
  setMotor(0, IN3, IN4, ENB);
}

void setup() {
  Serial.begin(9600);
  delay(1200);
  Serial.println("Mega CC1101 -> L298N RX");

  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  stopMotors(); // Ensure stop at boot

  if (!ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101 not detected.");
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
  ELECHOUSE_cc1101.SetRx();

  lastPacketMs = millis();
}

void loop() {
  if (ELECHOUSE_cc1101.CheckRxFifo(50)) {
    byte buf[64] = {0}; // Increased buffer size for safety
    byte len = ELECHOUSE_cc1101.ReceiveData(buf);

    if (len == sizeof(Packet)) {
      memcpy(&pkt, buf, sizeof(Packet));
      
      if (pkt.crc == computeCrc(pkt)) {
        lastPacketMs = millis();

        // MIXING LOGIC:
        // Arcade Drive: Throttle = Forward/Back, Steer = Turn
        int left  = pkt.throttle + pkt.steer;
        int right = pkt.throttle - pkt.steer;

        // Clip to max PWM
        left  = constrain(left,  -255, 255);
        right = constrain(right, -255, 255);

        setMotor(left,  IN1, IN2, ENA);
        setMotor(right, IN3, IN4, ENB);

        Serial.print("L:"); Serial.print(left);
        Serial.print(" R:"); Serial.println(right);
      } else {
        Serial.println("CRC Fail");
      }
    }

    // Reset Radio to RX
    ELECHOUSE_cc1101.SpiStrobe(0x36); // SIDLE
    delay(2);
    ELECHOUSE_cc1101.SpiStrobe(0x3A); // SFRX
    delay(2);
    ELECHOUSE_cc1101.SetRx();
  }

  // Failsafe: Stop if signal lost for 500ms
  if (millis() - lastPacketMs > 500) {
    stopMotors();
  }
}