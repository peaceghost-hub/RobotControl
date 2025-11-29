/**
 * ESP32 Robot Controller (Arduino)
 * Connects to the Environmental Monitoring Dashboard
 *
 * This sketch runs on ESP32 and:
 * - Reads sensor data
 * - Sends data to dashboard via HTTP
 * - Receives and executes commands from dashboard
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

// WiFi Configuration
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Dashboard Configuration
const char* DASHBOARD_URL = "http://192.168.1.100:5000";  // Change to your dashboard IP
const char* DEVICE_ID = "esp32_01";

// Timing intervals (in milliseconds)
const unsigned long SENSOR_INTERVAL = 10000;   // 10 seconds
const unsigned long GPS_INTERVAL = 15000;      // 15 seconds
const unsigned long STATUS_INTERVAL = 30000;   // 30 seconds
const unsigned long COMMAND_INTERVAL = 5000;   // 5 seconds

// Sensor pins
const int TEMP_PIN = 34;
const int LIGHT_PIN = 35;
const int LED_PIN = 2;

// Global variables
unsigned long lastSensorSend = 0;
unsigned long lastGpsSend = 0;
unsigned long lastStatusSend = 0;
unsigned long lastCommandCheck = 0;

WiFiClient wifiClient;

void setup() {
  Serial.begin(115200);

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEMP_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);

  // Connect to WiFi
  connectToWiFi();

  Serial.println("ESP32 Robot Controller started");
}

void loop() {
  unsigned long currentTime = millis();

  // Send sensor data
  if (currentTime - lastSensorSend >= SENSOR_INTERVAL) {
    sendSensorData();
    lastSensorSend = currentTime;
  }

  // Send GPS data
  if (currentTime - lastGpsSend >= GPS_INTERVAL) {
    sendGpsData();
    lastGpsSend = currentTime;
  }

  // Send status
  if (currentTime - lastStatusSend >= STATUS_INTERVAL) {
    sendStatusData();
    lastStatusSend = currentTime;
  }

  // Check for commands
  if (currentTime - lastCommandCheck >= COMMAND_INTERVAL) {
    checkPendingCommands();
    lastCommandCheck = currentTime;
  }

  delay(100);  // Small delay to prevent overwhelming the CPU
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void sendSensorData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping sensor data send");
    return;
  }

  HTTPClient http;
  http.begin(wifiClient, String(DASHBOARD_URL) + "/api/sensor_data");
  http.addHeader("Content-Type", "application/json");

  // Read sensors
  int tempRaw = analogRead(TEMP_PIN);
  int lightRaw = analogRead(LIGHT_PIN);

  // Convert to meaningful values
  float temperature = (tempRaw / 4095.0) * 50.0;  // 0-50°C approximation
  float lightLevel = (lightRaw / 4095.0) * 100.0; // 0-100% approximation

  // Create JSON payload
  DynamicJsonDocument doc(1024);
  doc["timestamp"] = getTimestamp();
  doc["device_id"] = DEVICE_ID;
  doc["temperature"] = round(temperature * 10) / 10;
  doc["humidity"] = 60.0;  // Mock humidity
  doc["mq2"] = 150;        // Mock gas sensors
  doc["mq135"] = 200;
  doc["mq7"] = 100;
  doc["light_level"] = round(lightLevel * 10) / 10;

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.println("Sending sensor data...");
  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    Serial.printf("Sensor data sent successfully, response: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error sending sensor data: %d\n", httpResponseCode);
  }

  http.end();
}

void sendGpsData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping GPS data send");
    return;
  }

  HTTPClient http;
  http.begin(wifiClient, String(DASHBOARD_URL) + "/api/gps_data");
  http.addHeader("Content-Type", "application/json");

  // Mock GPS data (replace with actual GPS readings)
  DynamicJsonDocument doc(512);
  doc["latitude"] = 40.7128;
  doc["longitude"] = -74.0060;
  doc["altitude"] = 10.5;
  doc["speed"] = 0.0;
  doc["heading"] = 0.0;
  doc["satellites"] = 0;
  doc["timestamp"] = getTimestamp();
  doc["device_id"] = DEVICE_ID;

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.println("Sending GPS data...");
  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    Serial.printf("GPS data sent successfully, response: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error sending GPS data: %d\n", httpResponseCode);
  }

  http.end();
}

void sendStatusData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping status data send");
    return;
  }

  HTTPClient http;
  http.begin(wifiClient, String(DASHBOARD_URL) + "/api/status");
  http.addHeader("Content-Type", "application/json");

  // Get system info
  DynamicJsonDocument doc(512);
  doc["online"] = true;
  doc["battery"] = 85.5;
  doc["signal_strength"] = WiFi.RSSI();
  doc["device_id"] = DEVICE_ID;

  JsonObject systemInfo = doc.createNestedObject("system_info");
  systemInfo["free_memory"] = ESP.getFreeHeap();
  systemInfo["uptime"] = millis() / 1000;

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.println("Sending status data...");
  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    Serial.printf("Status data sent successfully, response: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error sending status data: %d\n", httpResponseCode);
  }

  http.end();
}

void checkPendingCommands() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping command check");
    return;
  }

  HTTPClient http;
  String url = String(DASHBOARD_URL) + "/api/commands/pending?device_id=" + DEVICE_ID;
  http.begin(wifiClient, url);

  Serial.println("Checking for pending commands...");
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    String payload = http.getString();

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      JsonArray commands = doc["commands"];

      for (JsonObject command : commands) {
        executeCommand(command);
      }
    } else {
      Serial.println("Failed to parse command response");
    }
  } else {
    Serial.printf("Error getting commands: %d\n", httpResponseCode);
  }

  http.end();
}

void executeCommand(JsonObject command) {
  int commandId = command["id"];
  const char* commandType = command["command_type"];
  JsonObject payload = command["payload"];

  Serial.printf("Executing command: %s (ID: %d)\n", commandType, commandId);

  bool success = true;
  String errorMessage = "";

  try {
    if (strcmp(commandType, "move") == 0) {
      const char* direction = payload["direction"];
      float speed = payload["speed"] | 0.0;

      Serial.printf("Moving %s at speed %.1f\n", direction, speed);
      // Your motor control code here

    } else if (strcmp(commandType, "stop") == 0) {
      Serial.println("Stopping ESP32 robot");
      // Stop motors

    } else if (strcmp(commandType, "led_on") == 0) {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED turned on");

    } else if (strcmp(commandType, "led_off") == 0) {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED turned off");

    } else {
      Serial.printf("Unknown command type: %s\n", commandType);
      success = false;
      errorMessage = "Unknown command type";
    }

  } catch (const std::exception& e) {
    Serial.printf("Error executing command: %s\n", e.what());
    success = false;
    errorMessage = e.what();
  }

  // Acknowledge command
  acknowledgeCommand(commandId, success ? "completed" : "failed", errorMessage);
}

void acknowledgeCommand(int commandId, const char* status, String errorMessage) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  HTTPClient http;
  String url = String(DASHBOARD_URL) + "/api/commands/" + String(commandId) + "/ack";
  http.begin(wifiClient, url);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument doc(256);
  doc["status"] = status;
  if (errorMessage.length() > 0) {
    doc["error_message"] = errorMessage;
  }

  String jsonString;
  serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0) {
    Serial.printf("Command %d acknowledged: %s\n", commandId, status);
  } else {
    Serial.printf("Error acknowledging command %d: %d\n", commandId, httpResponseCode);
  }

  http.end();
}

String getTimestamp() {
  // Simplified timestamp (in real implementation, sync with NTP)
  char timestamp[25];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d",
          2025, 11, 9, 12, 0, 0);  // Replace with actual time
  return String(timestamp);
}