// Arduino UNO ZigBee Remote (CC2530/CC2591 transparent UART)
//
// Sends MCTL commands from a joystick over ZigBee UART.
//
// Wiring (UNO <-> ZigBee module):
//   UNO D2 (RX)  <- Module TX
//   UNO D3 (TX)  -> Module RX  (IMPORTANT: module is 3.3V logic; use a divider)
//   GND common
//   Module VCC per your module (often 3.3V recommended)
//
// Joystick:
//   VRx -> A0
//   VRy -> A1
//   GND -> GND
//   VCC -> 5V (most joystick modules are fine at 5V)

#include <SoftwareSerial.h>

// ZigBee serial on pins 2 (RX) and 3 (TX)
static const uint8_t ZIGBEE_RX_PIN = 2;
static const uint8_t ZIGBEE_TX_PIN = 3;
static const long ZIGBEE_BAUD = 9600; // Must match the Mega's ZIGBEE_BAUD

SoftwareSerial zigbee(ZIGBEE_RX_PIN, ZIGBEE_TX_PIN); // RX, TX

// Connection tracking
unsigned long lastRxMillis = 0;
bool connected = false;
bool readySeen = false;

// Status LED
static const uint8_t LED_PIN = 13;

// Joystick pins
static const uint8_t JOY_X_PIN = A0;
static const uint8_t JOY_Y_PIN = A1;

// Tuning
static const int JOY_CENTER = 512;
static const int JOY_DEADZONE = 90;
static const unsigned long COMMAND_INTERVAL_MS = 120;
static const unsigned long PING_INTERVAL_MS = 1000;

static unsigned long lastCommandMs = 0;
static unsigned long lastPingMs = 0;

enum MoveDir { DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT };
MoveDir lastDir = DIR_STOP;
int lastSpeed = 0;

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
	zigbee.println(F("PING"));
}

void loop() {
	// Periodic PING until READY seen
	if (!readySeen && (millis() - lastPingMs >= PING_INTERVAL_MS)) {
		Serial.println(F("[UNO] TX PING"));
		zigbee.println(F("PING"));
		lastPingMs = millis();
	}

	// Echo any incoming ZigBee frames to Serial and update connection status
	static String line;
	while (zigbee.available()) {
		char c = zigbee.read();

		if (c == '\n') {
			line.trim();
			if (line.length() > 0) {
				// Only consider the link alive when we receive a well-formed ASCII line.
				lastRxMillis = millis();
				Serial.print(F("[UNO] RX "));
				Serial.println(line);
				if (line.equalsIgnoreCase("READY")) {
					readySeen = true;
					Serial.println(F("[UNO] READY received"));
				}
			}
			line = "";
		} else if (c != '\r') {
			// Keep only printable ASCII to avoid garbage due to baud mismatch/noise.
			if (c >= 32 && c <= 126) {
				line += c;
			}
			if (line.length() > 96) {
				line.remove(0, line.length() - 64);
			}
		}
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

	// Joystick control loop
	if (millis() - lastCommandMs >= COMMAND_INTERVAL_MS) {
		int x = analogRead(JOY_X_PIN);
		int y = analogRead(JOY_Y_PIN);

		int dx = x - JOY_CENTER;
		int dy = y - JOY_CENTER;

		MoveDir dir = DIR_STOP;
		int mag = 0;
		if (abs(dy) >= abs(dx)) {
			// Forward/back dominates
			if (dy > JOY_DEADZONE) {
				dir = DIR_FWD;
				mag = dy;
			} else if (dy < -JOY_DEADZONE) {
				dir = DIR_BACK;
				mag = -dy;
			}
		} else {
			// Left/right dominates
			if (dx > JOY_DEADZONE) {
				dir = DIR_RIGHT;
				mag = dx;
			} else if (dx < -JOY_DEADZONE) {
				dir = DIR_LEFT;
				mag = -dx;
			}
		}

		// Map magnitude to speed (0-255). Keep a minimum so it actually moves.
		int speed = 0;
		if (dir != DIR_STOP) {
			mag = constrain(mag, JOY_DEADZONE, 512);
			speed = map(mag, JOY_DEADZONE, 512, 120, 255);
		}

		// Only send when changes occur to reduce spam.
		if (dir != lastDir || abs(speed - lastSpeed) >= 10) {
			Serial.print(F("[UNO] TX "));
			switch (dir) {
				case DIR_FWD:
					Serial.print(F("MCTL,FORWARD,"));
					Serial.println(speed);
					zigbee.print(F("MCTL,FORWARD,"));
					zigbee.println(speed);
					break;
				case DIR_BACK:
					Serial.print(F("MCTL,BACKWARD,"));
					Serial.println(speed);
					zigbee.print(F("MCTL,BACKWARD,"));
					zigbee.println(speed);
					break;
				case DIR_LEFT:
					Serial.print(F("MCTL,LEFT,"));
					Serial.println(speed);
					zigbee.print(F("MCTL,LEFT,"));
					zigbee.println(speed);
					break;
				case DIR_RIGHT:
					Serial.print(F("MCTL,RIGHT,"));
					Serial.println(speed);
					zigbee.print(F("MCTL,RIGHT,"));
					zigbee.println(speed);
					break;
				case DIR_STOP:
				default:
					Serial.println(F("MCTL,STOP"));
					zigbee.println(F("MCTL,STOP"));
					break;
			}

			lastDir = dir;
			lastSpeed = speed;
			lastCommandMs = millis();
		} else {
			lastCommandMs = millis();
		}
	}
}

void flushZigBee() {
	while (zigbee.available()) { zigbee.read(); }
}