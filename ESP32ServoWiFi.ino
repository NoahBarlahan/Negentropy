/*
  ESP32 Wi-Fi Servo Command Server

  Upload this sketch to an ESP32. It joins your Wi-Fi network and accepts:
    GET /health                 connection test
    GET /servo?angle=90         set a servo angle (0 through 180)

  Before uploading:
    1. Set WIFI_SSID and WIFI_PASSWORD below.
    2. Leave SERVO_PIN as -1 until the servo signal wire has a GPIO assigned.
       /health will still work, while /servo safely returns an error.
    3. When ready, use a PWM-capable GPIO such as 18 (not GPIO 34-39).

  Arduino IDE requirements:
    - Install an ESP32 board package from Boards Manager.
    - Install the "ESP32Servo" library from Library Manager.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char *WIFI_SSID = "REPLACE_WITH_WIFI_NAME";
const char *WIFI_PASSWORD = "REPLACE_WITH_WIFI_PASSWORD";

// Keep -1 until you have chosen and wired a servo signal pin.
const int SERVO_PIN = -1;
const int DEFAULT_ANGLE = 90;

WebServer server(80);
Servo servo;
bool servoAttached = false;
int currentAngle = DEFAULT_ANGLE;

void sendJson(int statusCode, const String &body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(statusCode, "application/json", body);
}

void handleHealth() {
  String body = "{\"ok\":true,\"ip\":\"" + WiFi.localIP().toString() +
                "\",\"servo_configured\":" + (servoAttached ? "true" : "false") + "}";
  sendJson(200, body);
}

void handleServo() {
  if (!servoAttached) {
    sendJson(409, "{\"ok\":false,\"error\":\"Servo pin is not configured. Set SERVO_PIN, wire the servo, and re-upload.\"}");
    return;
  }

  if (!server.hasArg("angle")) {
    sendJson(400, "{\"ok\":false,\"error\":\"Missing angle query parameter (0-180).\"}");
    return;
  }

  const int angle = server.arg("angle").toInt();
  const String angleText = server.arg("angle");
  if (angleText.length() == 0 || angle < 0 || angle > 180) {
    sendJson(400, "{\"ok\":false,\"error\":\"Angle must be an integer from 0 to 180.\"}");
    return;
  }

  servo.write(angle);
  currentAngle = angle;
  sendJson(200, "{\"ok\":true,\"angle\":" + String(currentAngle) + "}");
}

void setup() {
  Serial.begin(115200);
  delay(250);

  if (SERVO_PIN >= 0) {
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2400);
    servo.write(DEFAULT_ANGLE);
    servoAttached = true;
    Serial.printf("Servo enabled on GPIO %d at %d degrees.\n", SERVO_PIN, DEFAULT_ANGLE);
  } else {
    Serial.println("Servo disabled: set SERVO_PIN when you have selected a signal pin.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to Wi-Fi network: %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected. ESP32 address: http://");
  Serial.println(WiFi.localIP());
  Serial.println("Try: /health  or  /servo?angle=90");

  server.on("/health", HTTP_GET, handleHealth);
  server.on("/servo", HTTP_GET, handleServo);
  server.onNotFound([]() {
    sendJson(404, "{\"ok\":false,\"error\":\"Use /health or /servo?angle=0..180\"}");
  });
  server.begin();
}

void loop() {
  server.handleClient();
}
