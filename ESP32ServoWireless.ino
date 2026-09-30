/*
  ESP32-WROOM Wireless Servo Controller

  One initial USB upload sets up all three wireless capabilities:
    - Bluetooth Classic serial commands: ESP32-Servo, pairing PIN 1234
    - Wi-Fi HTTP commands: /health and /servo?angle=90
    - Password-protected Wi-Fi OTA sketch uploads: hostname esp32-servo.local

  IMPORTANT: Replace the three placeholder values below ON YOUR COMPUTER.
  Do not put your real Wi-Fi or OTA password in a public repository.

  Leave SERVO_PIN as -1 until the servo signal wire has a chosen GPIO. Bluetooth
  and Wi-Fi health checks still work, while angle commands safely refuse to run.

  Required Arduino libraries:
    - ESP32 board package (includes WiFi, WebServer, BluetoothSerial, ArduinoOTA)
    - ESP32Servo library from Arduino Library Manager
*/

#include <ArduinoOTA.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <WiFi.h>

// ---- Set these locally before the first USB upload. ----
const char *WIFI_SSID = "REPLACE_WITH_YOUR_WIFI_NAME";
const char *WIFI_PASSWORD = "REPLACE_WITH_YOUR_WIFI_PASSWORD";
const char *OTA_PASSWORD = "REPLACE_WITH_A_LONG_UNIQUE_PASSWORD";

const char *BLUETOOTH_NAME = "ESP32-Servo";
const char *BLUETOOTH_PIN = "1234";
const char *OTA_HOSTNAME = "esp32-servo";

// Keep -1 until the servo signal wire is connected to a GPIO, such as GPIO 18.
const int SERVO_PIN = -1;
const int DEFAULT_ANGLE = 90;
const size_t COMMAND_BUFFER_LENGTH = 32;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;

BluetoothSerial SerialBT;
WebServer server(80);
Servo servo;
bool servoAttached = false;
bool webServerStarted = false;
bool otaStarted = false;
int currentAngle = DEFAULT_ANGLE;
char commandBuffer[COMMAND_BUFFER_LENGTH];
size_t commandLength = 0;
unsigned long lastWifiRetry = 0;

bool configured(const char *value) {
  return String(value).indexOf("REPLACE_WITH_") < 0;
}

void logAndBluetoothReply(const String &message) {
  Serial.println(message);
  if (SerialBT.hasClient()) {
    SerialBT.println(message);
  }
}

bool setServoAngle(int angle, String &error) {
  if (!servoAttached) {
    error = "servo pin is not configured";
    return false;
  }
  if (angle < 0 || angle > 180) {
    error = "angle must be an integer from 0 through 180";
    return false;
  }
  servo.write(angle);
  currentAngle = angle;
  return true;
}

void handleHealth() {
  const String body = String("{\"ok\":true,\"ip\":\"") + WiFi.localIP().toString() +
                      "\",\"servo_configured\":" + (servoAttached ? "true" : "false") +
                      ",\"ota_ready\":" + (otaStarted ? "true" : "false") + "}";
  server.send(200, "application/json", body);
}

void handleServo() {
  if (!server.hasArg("angle")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing angle query parameter\"}");
    return;
  }
  const String angleText = server.arg("angle");
  for (size_t i = 0; i < angleText.length(); ++i) {
    if (!isDigit(angleText[i])) {
      server.send(400, "application/json", "{\"ok\":false,\"error\":\"angle must be an integer from 0 through 180\"}");
      return;
    }
  }
  String error;
  if (!setServoAngle(angleText.toInt(), error)) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"" + error + "\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true,\"angle\":" + String(currentAngle) + "}");
}

void processBluetoothCommand(String command) {
  command.trim();
  command.toLowerCase();
  if (command == "health") {
    logAndBluetoothReply(String("OK bluetooth_connected=true servo_configured=") +
                         (servoAttached ? "true" : "false") +
                         " wifi_connected=" + (WiFi.status() == WL_CONNECTED ? "true" : "false"));
    return;
  }
  if (command == "help") {
    logAndBluetoothReply("Commands: health | angle <0-180> | help");
    return;
  }
  if (command.startsWith("angle ")) {
    String angleText = command.substring(6);
    angleText.trim();
    for (size_t i = 0; i < angleText.length(); ++i) {
      if (!isDigit(angleText[i])) {
        logAndBluetoothReply("ERROR angle must be an integer from 0 through 180.");
        return;
      }
    }
    String error;
    if (!setServoAngle(angleText.toInt(), error)) {
      logAndBluetoothReply("ERROR " + error + ".");
      return;
    }
    logAndBluetoothReply("OK angle=" + String(currentAngle));
    return;
  }
  logAndBluetoothReply("ERROR unknown command. Type help.");
}

void readBluetoothCommands() {
  while (SerialBT.available()) {
    const char received = static_cast<char>(SerialBT.read());
    if (received == '\r') continue;
    if (received == '\n') {
      commandBuffer[commandLength] = '\0';
      processBluetoothCommand(String(commandBuffer));
      commandLength = 0;
    } else if (commandLength < COMMAND_BUFFER_LENGTH - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      commandLength = 0;
      logAndBluetoothReply("ERROR command too long.");
    }
  }
}

void startWifiServices() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!webServerStarted) {
    server.on("/health", HTTP_GET, handleHealth);
    server.on("/servo", HTTP_GET, handleServo);
    server.begin();
    webServerStarted = true;
  }

  if (!otaStarted) {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { Serial.println("OTA update starting."); });
    ArduinoOTA.onEnd([]() { Serial.println("OTA update complete; restarting."); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA error: %u\n", error); });
    ArduinoOTA.begin();
    otaStarted = true;
  }

  Serial.print("Wi-Fi ready. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("OTA hostname: ");
  Serial.print(OTA_HOSTNAME);
  Serial.println(".local");
}

void setup() {
  Serial.begin(115200);
  delay(250);

  if (SERVO_PIN >= 0) {
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2400);
    servo.write(DEFAULT_ANGLE);
    servoAttached = true;
  }

  SerialBT.setPin(BLUETOOTH_PIN, 4);
  SerialBT.begin(BLUETOOTH_NAME);
  Serial.println("Bluetooth ready: ESP32-Servo (PIN 1234)");

  if (!configured(WIFI_SSID) || !configured(WIFI_PASSWORD) || !configured(OTA_PASSWORD)) {
    Serial.println("Wi-Fi/OTA not configured. Replace all three placeholder credentials and upload again.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
}

void loop() {
  readBluetoothCommands();

  if (WiFi.status() == WL_CONNECTED) {
    startWifiServices();
    if (webServerStarted) server.handleClient();
    if (otaStarted) ArduinoOTA.handle();
    return;
  }

  if (configured(WIFI_SSID) && configured(WIFI_PASSWORD) &&
      millis() - lastWifiRetry >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetry = millis();
    Serial.println("Wi-Fi disconnected; retrying.");
    WiFi.reconnect();
  }
}
