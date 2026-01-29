/*
 * Simple I2C Slave Test for Arduino Mega
 * Upload this to Mega to verify I2C communication works
 * 
 * Expected behavior:
 * - Mega listens on I2C address 0x08
 * - Responds to any read with incrementing counter
 * - Blinks LED on every I2C request
 * - Prints debug info to Serial
 */

#include <Wire.h>

#define I2C_ADDRESS 0x08
#define LED_PIN 13

volatile uint8_t counter = 0;
volatile bool requestReceived = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println("========================================");
  Serial.println("I2C Slave Test - Arduino Mega");
  Serial.println("========================================");
  Serial.print("I2C Address: 0x");
  Serial.println(I2C_ADDRESS, HEX);
  Serial.println("Waiting for I2C requests from Pi...");
  Serial.println();
  
  // Initialize I2C as slave
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  Serial.println("I2C initialized. Status:");
  Serial.println("- SDA: Pin 20");
  Serial.println("- SCL: Pin 21");
  Serial.println("- Pull-ups: Required on SDA and SCL");
  Serial.println();
}

void loop() {
  if (requestReceived) {
    requestReceived = false;
    
    // Blink LED to show activity
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
  
  // Print heartbeat every 5 seconds
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    Serial.print("Heartbeat - I2C requests received: ");
    Serial.println(counter);
  }
}

// Called when Pi requests data
void requestEvent() {
  counter++;
  Wire.write(counter);  // Send counter value
  requestReceived = true;
  
  Serial.print("REQUEST #");
  Serial.print(counter);
  Serial.print(" - Sent: 0x");
  Serial.println(counter, HEX);
}

// Called when Pi sends data
void receiveEvent(int numBytes) {
  Serial.print("RECEIVE - ");
  Serial.print(numBytes);
  Serial.print(" bytes: ");
  
  while (Wire.available()) {
    uint8_t data = Wire.read();
    Serial.print("0x");
    Serial.print(data, HEX);
    Serial.print(" ");
  }
  Serial.println();
  requestReceived = true;
}
