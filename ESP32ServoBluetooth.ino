/*
  ESP32-WROOM Bluetooth Servo Command Server

  This is a Bluetooth Classic (SPP / serial) sketch for an original ESP32-WROOM.
  It is separate from ESP32ServoWiFi.ino: upload ONE sketch at a time.

  Bluetooth name: ESP32-Servo
  Pairing PIN:     1234

  Bluetooth commands (end each command with Enter):
    health       test that Bluetooth serial is connected
    angle 90     move to an angle from 0 through 180
    help         show available commands

  Before wiring a servo, leave SERVO_PIN as -1. The ESP32 will pair and answer
  "health", but safely refuses angle commands. When ready, set it to a suitable
  output GPIO such as 18 and re-upload.

  Requires the ESP32 board package and the ESP32Servo library. BluetoothSerial
  is included in the original ESP32 board package; it works on ESP32-WROOM.
*/

#include <BluetoothSerial.h>
#include <ESP32Servo.h>

const char *BLUETOOTH_NAME = "ESP32-Servo";
const char *PAIRING_PIN = "1234";

// Keep -1 until you choose and wire a servo signal GPIO.
const int SERVO_PIN = -1;
const int DEFAULT_ANGLE = 90;
const size_t COMMAND_BUFFER_LENGTH = 32;

BluetoothSerial SerialBT;
Servo servo;
bool servoAttached = false;
int currentAngle = DEFAULT_ANGLE;
char commandBuffer[COMMAND_BUFFER_LENGTH];
size_t commandLength = 0;

void reply(const String &message) {
  SerialBT.println(message);
  Serial.println(message);
}

void printHelp() {
  reply("Commands: health | angle <0-180> | help");
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command == "health") {
    reply(String("OK bluetooth_connected=true servo_configured=") +
          (servoAttached ? "true" : "false"));
    return;
  }

  if (command == "help") {
    printHelp();
    return;
  }

  if (command.startsWith("angle ")) {
    if (!servoAttached) {
      reply("ERROR servo pin is not configured. Set SERVO_PIN, wire the servo, and re-upload.");
      return;
    }

    String angleText = command.substring(6);
    angleText.trim();
    for (size_t i = 0; i < angleText.length(); ++i) {
      if (!isDigit(angleText[i])) {
        reply("ERROR angle must be an integer from 0 through 180.");
        return;
      }
    }

    const int angle = angleText.toInt();
    if (angleText.length() == 0 || angle < 0 || angle > 180) {
      reply("ERROR angle must be an integer from 0 through 180.");
      return;
    }

    servo.write(angle);
    currentAngle = angle;
    reply("OK angle=" + String(currentAngle));
    return;
  }

  reply("ERROR unknown command. Type help.");
}

void readBluetoothCommands() {
  while (SerialBT.available()) {
    const char received = static_cast<char>(SerialBT.read());
    if (received == '\r') {
      continue;
    }
    if (received == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(String(commandBuffer));
      commandLength = 0;
      continue;
    }
    if (commandLength < COMMAND_BUFFER_LENGTH - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      commandLength = 0;
      reply("ERROR command too long.");
    }
  }
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

  SerialBT.setPin(PAIRING_PIN, 4);
  if (!SerialBT.begin(BLUETOOTH_NAME)) {
    Serial.println("Bluetooth failed to start.");
    return;
  }

  Serial.println("Bluetooth ready. Pair with: ESP32-Servo (PIN 1234)");
  Serial.println("Then use esp32_bluetooth_command.py with its outgoing Bluetooth COM port.");
}

void loop() {
  readBluetoothCommands();
}
