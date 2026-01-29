// Arduino UNO ZigBee Remote - Connection Status + Echo
// Pins: D2 (SoftwareSerial RX from XBee DOUT), D3 (SoftwareSerial TX to XBee DIN)
// Joystick pins optional (A0/A1), not required for connection status demo

#include <SoftwareSerial.h>

// ZigBee serial on pins 2 (RX) and 3 (TX)
static const uint8_t ZIGBEE_RX_PIN = 2;
static const uint8_t ZIGBEE_TX_PIN = 3;
static const long ZIGBEE_BAUD = 57600;

SoftwareSerial zigbee(ZIGBEE_RX_PIN, ZIGBEE_TX_PIN); // RX, TX

// Connection tracking
unsigned long lastRxMillis = 0;
bool connected = false;

// Status LED
static const uint8_t LED_PIN = 13;

void setup() {
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	Serial.begin(115200);
	while (!Serial) { ; }
	Serial.println(F("[UNO] ZigBee Remote Booting..."));

	zigbee.begin(ZIGBEE_BAUD);
	delay(100);
	flushZigBee();

	Serial.print(F("[UNO] ZigBee init on D"));
	Serial.print(ZIGBEE_RX_PIN);
	Serial.print(F("(RX)/D"));
	Serial.print(ZIGBEE_TX_PIN);
	Serial.print(F("(TX) @ "));
	Serial.println(ZIGBEE_BAUD);

	// Send hello beacon to robot
	zigbee.println(F("HELLO,UNO_REMOTE"));
}

void loop() {
	// Echo any incoming ZigBee frames to Serial and update connection status
	while (zigbee.available()) {
		char c = zigbee.read();
		Serial.write(c);
		lastRxMillis = millis();
	}

	// Forward any Serial input to ZigBee (manual test typing)
	while (Serial.available()) {
		zigbee.write(Serial.read());
	}

	// Connection logic: consider connected if data seen in last 10 seconds
	bool nowConnected = (millis() - lastRxMillis) < 10000UL;
	if (nowConnected != connected) {
		connected = nowConnected;
		Serial.println(connected ? F("[UNO] ZigBee CONNECTED") : F("[UNO] ZigBee DISCONNECTED"));
		digitalWrite(LED_PIN, connected ? HIGH : LOW);
	}

	// Periodic hello beacon to robot (every 5s)
	static unsigned long lastHello = 0;
	if (millis() - lastHello >= 5000UL) {
		zigbee.println(F("HELLO,UNO_REMOTE"));
		lastHello = millis();
	}
}

void flushZigBee() {
	while (zigbee.available()) { zigbee.read(); }
}