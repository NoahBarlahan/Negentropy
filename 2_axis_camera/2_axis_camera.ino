#include <AccelStepper.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ===========================================================================
// TWO-AXIS CAMERA: SAFE, SLOW SERIAL JOG CONTROLLER
// ===========================================================================
// Axis assignment:
//   Z axis / Motor 1: DIR 32, STEP 34
//   Y axis / Motor 2: DIR 38, STEP 36
//
// Serial commands (9600 baud, Newline line ending):
//   z10     Move to absolute Z position +10 degrees.
//   z-10    Move to absolute Z position -10 degrees.
//   y5      Move to absolute Y position +5 degrees.
//   y-50    Move to absolute Y position -50 degrees.
//   s       Smoothly stop the moving axis.
//   p       Print both software positions.
//   zero    Declare the current positions to be Z=0 and Y=0. No movement.
//   h       Print help.
//
// Only one axis may move at a time. Commands outside the configured software
// limits are rejected. The software limits are useful only if the mechanism is
// physically placed at its safe center before the Mega starts or is reset.


// ---------------------------------------------------------------------------
// A4988 pin assignments
// ---------------------------------------------------------------------------
const byte Z_DIR_PIN = 32;
const byte Z_STEP_PIN = 34;

const byte Y_DIR_PIN = 38;
const byte Y_STEP_PIN = 36;

// AccelStepper::DRIVER uses STEP and DIR. Constructor order is STEP, then DIR.
AccelStepper zMotor(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper yMotor(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);


// ---------------------------------------------------------------------------
// Resolution and intentionally slow test motion
// ---------------------------------------------------------------------------
// Typical 1.8-degree NEMA 17 motor: 200 full steps per revolution.
const long FULL_STEPS_PER_REVOLUTION = 200;

// Both A4988 drivers are wired for 1/8 microstepping:
//   MS1 = HIGH, MS2 = HIGH, MS3 = LOW
const long MICROSTEPS_PER_FULL_STEP = 8;
const long STEPS_PER_REVOLUTION =
  FULL_STEPS_PER_REVOLUTION * MICROSTEPS_PER_FULL_STEP;

// 80 microsteps/s at 1600 microsteps/rev is 18 degrees per second.
// These deliberately slow values are intended for direction commissioning.
const float MAX_SPEED_MICROSTEPS_PER_SECOND = 80.0;
const float ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED = 120.0;
const unsigned int MINIMUM_STEP_PULSE_MICROSECONDS = 2;

// If a positive command rotates an axis opposite to your desired positive
// direction, change only that axis from false to true and upload again.
const bool REVERSE_Z_DIRECTION = false;
const bool REVERSE_Y_DIRECTION = false;


// ---------------------------------------------------------------------------
// Safety limits
// ---------------------------------------------------------------------------
// All positions are relative to the software zero created at startup. Place
// the mechanism at a safe central pose before powering/resetting the Mega.
// Expand or tighten these limits only after measuring the available CAD
// travel. Every z/y command is an absolute target inside these limits.
const float Z_MINIMUM_POSITION_DEGREES = -30.0;
const float Z_MAXIMUM_POSITION_DEGREES = 30.0;
const float Y_MINIMUM_POSITION_DEGREES = -135.0;
const float Y_MAXIMUM_POSITION_DEGREES = 45.0;

const unsigned long SERIAL_BAUD_RATE = 9600;
const byte SERIAL_BUFFER_LENGTH = 48;
char serialBuffer[SERIAL_BUFFER_LENGTH];
byte serialBufferLength = 0;


enum ActiveAxis {
  NO_AXIS,
  Z_AXIS,
  Y_AXIS,
  STOPPING_AXIS
};

ActiveAxis activeAxis = NO_AXIS;


float stepsToDegrees(long steps) {
  return steps * 360.0 / STEPS_PER_REVOLUTION;
}


long degreesToSteps(float degrees) {
  return lround(degrees * STEPS_PER_REVOLUTION / 360.0);
}


void printHelp() {
  Serial.println();
  Serial.println("TWO-AXIS CAMERA JOG CONTROLLER");
  Serial.println("----------------------------------------");
  Serial.println("z10   : move Z / Motor 1 to +10 degrees");
  Serial.println("z-10  : move Z / Motor 1 to -10 degrees");
  Serial.println("y10   : move Y / Motor 2 to +10 degrees");
  Serial.println("y-50  : move Y / Motor 2 to -50 degrees");
  Serial.println("s     : smoothly stop the moving axis");
  Serial.println("p     : print both software positions");
  Serial.println("zero  : make the current pose software zero");
  Serial.println("h     : print this help menu");
  Serial.println("----------------------------------------");
  Serial.println("Use Newline or Both NL & CR line ending.");
  Serial.println("For the first direction test, use z1 and y1.");
  Serial.println("Z limits: -60 to +60 deg; Y limits: -100 to +60 deg.");
  Serial.println();
}


void printPositions() {
  Serial.print("Z / Motor 1: ");
  Serial.print(stepsToDegrees(zMotor.currentPosition()), 2);
  Serial.print(" deg  (");
  Serial.print(zMotor.currentPosition());
  Serial.println(" microsteps)");

  Serial.print("Y / Motor 2: ");
  Serial.print(stepsToDegrees(yMotor.currentPosition()), 2);
  Serial.print(" deg  (");
  Serial.print(yMotor.currentPosition());
  Serial.println(" microsteps)");
}


bool motorsAreStationary() {
  return activeAxis == NO_AXIS
    && zMotor.distanceToGo() == 0
    && yMotor.distanceToGo() == 0;
}


void startMoveToPosition(char axis, float targetDegrees) {
  if (!motorsAreStationary()) {
    Serial.println("REJECTED: an axis is moving. Wait or send s.");
    return;
  }

  AccelStepper *motor = axis == 'z' ? &zMotor : &yMotor;
  float currentDegrees = stepsToDegrees(motor->currentPosition());
  float minimumDegrees = axis == 'z'
    ? Z_MINIMUM_POSITION_DEGREES
    : Y_MINIMUM_POSITION_DEGREES;
  float maximumDegrees = axis == 'z'
    ? Z_MAXIMUM_POSITION_DEGREES
    : Y_MAXIMUM_POSITION_DEGREES;

  if (targetDegrees < minimumDegrees || targetDegrees > maximumDegrees) {
    Serial.print("REJECTED: ");
    Serial.print(axis == 'z' ? "Z" : "Y");
    Serial.print(" target ");
    Serial.print(targetDegrees, 2);
    Serial.print(" deg is outside [");
    Serial.print(minimumDegrees, 1);
    Serial.print(", ");
    Serial.print(maximumDegrees, 1);
    Serial.println("] deg.");
    return;
  }

  long targetSteps = degreesToSteps(targetDegrees);
  if (targetSteps == motor->currentPosition()) {
    Serial.print(axis == 'z' ? "Z" : "Y");
    Serial.print(" is already at approximately ");
    Serial.print(currentDegrees, 2);
    Serial.println(" degrees.");
    return;
  }

  motor->moveTo(targetSteps);
  activeAxis = axis == 'z' ? Z_AXIS : Y_AXIS;

  Serial.print("MOVING ");
  Serial.print(axis == 'z' ? "Z / Motor 1" : "Y / Motor 2");
  Serial.print(" from ");
  Serial.print(currentDegrees, 2);
  Serial.print(" deg to absolute software position ");
  Serial.print(targetDegrees, 2);
  Serial.print(" deg at ");
  Serial.print(
    360.0 * MAX_SPEED_MICROSTEPS_PER_SECOND / STEPS_PER_REVOLUTION,
    1
  );
  Serial.println(" deg/s maximum.");
}


void stopMotion() {
  if (motorsAreStationary()) {
    Serial.println("Both axes are already stopped.");
    return;
  }

  zMotor.stop();
  yMotor.stop();
  activeAxis = STOPPING_AXIS;
  Serial.println("STOP REQUESTED: decelerating the active axis.");
}


char *trimWhitespace(char *text) {
  while (isspace(*text)) {
    text++;
  }

  char *end = text + strlen(text);
  while (end > text && isspace(*(end - 1))) {
    end--;
  }
  *end = '\0';
  return text;
}


void processSerialLine(char *line) {
  char *command = trimWhitespace(line);
  if (*command == '\0') {
    return;
  }

  if (strcasecmp(command, "s") == 0) {
    stopMotion();
    return;
  }

  if (strcasecmp(command, "p") == 0) {
    printPositions();
    return;
  }

  if (strcasecmp(command, "h") == 0) {
    printHelp();
    return;
  }

  if (strcasecmp(command, "zero") == 0) {
    if (!motorsAreStationary()) {
      Serial.println("REJECTED: stop both axes before setting zero.");
      return;
    }
    zMotor.setCurrentPosition(0);
    yMotor.setCurrentPosition(0);
    Serial.println("Current pose is now software Z=0, Y=0.");
    return;
  }

  char axis = tolower(command[0]);
  if (axis != 'z' && axis != 'y') {
    Serial.println("INVALID: use z<number>, y<number>, s, p, zero, or h.");
    return;
  }

  char *numberText = trimWhitespace(command + 1);
  char *parseEnd = nullptr;
  float targetDegrees = strtod(numberText, &parseEnd);
  parseEnd = trimWhitespace(parseEnd);

  if (
    parseEnd == numberText
    || *parseEnd != '\0'
    || isnan(targetDegrees)
    || isinf(targetDegrees)
  ) {
    Serial.println("INVALID ANGLE: examples are z10 and y-50.");
    return;
  }

  startMoveToPosition(axis, targetDegrees);
}


void readSerialWithoutBlocking() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\n') {
      serialBuffer[serialBufferLength] = '\0';
      processSerialLine(serialBuffer);
      serialBufferLength = 0;
    } else if (incoming != '\r') {
      if (serialBufferLength < SERIAL_BUFFER_LENGTH - 1) {
        serialBuffer[serialBufferLength++] = incoming;
      } else {
        serialBufferLength = 0;
        Serial.println("INVALID: serial command was too long.");
      }
    }

    zMotor.run();
    yMotor.run();
  }
}


void updateCompletedMotion() {
  if (activeAxis == Z_AXIS && zMotor.distanceToGo() == 0) {
    Serial.println("Z / Motor 1 movement complete.");
    activeAxis = NO_AXIS;
    printPositions();
  } else if (activeAxis == Y_AXIS && yMotor.distanceToGo() == 0) {
    Serial.println("Y / Motor 2 movement complete.");
    activeAxis = NO_AXIS;
    printPositions();
  } else if (
    activeAxis == STOPPING_AXIS
    && zMotor.distanceToGo() == 0
    && yMotor.distanceToGo() == 0
  ) {
    Serial.println("Both axes stopped.");
    activeAxis = NO_AXIS;
    printPositions();
  }
}


void configureMotor(AccelStepper &motor, bool reverseDirection) {
  motor.setMaxSpeed(MAX_SPEED_MICROSTEPS_PER_SECOND);
  motor.setAcceleration(ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED);
  motor.setMinPulseWidth(MINIMUM_STEP_PULSE_MICROSECONDS);
  motor.setPinsInverted(reverseDirection, false, false);

  // This is only a software reference; there are no homing sensors yet.
  motor.setCurrentPosition(0);
  motor.moveTo(0);
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  configureMotor(zMotor, REVERSE_Z_DIRECTION);
  configureMotor(yMotor, REVERSE_Y_DIRECTION);

  Serial.println();
  Serial.println("TWO-AXIS CAMERA JOG CONTROLLER READY");
  Serial.print("Resolution: ");
  Serial.print(STEPS_PER_REVOLUTION);
  Serial.println(" microsteps/revolution (1/8 mode)");
  Serial.println("Startup pose is assumed to be software Z=0, Y=0.");
  Serial.println("CAUTION: no limit switches or physical homing are present.");
  printHelp();
}


void loop() {
  // run() is non-blocking and must be called as frequently as possible.
  zMotor.run();
  yMotor.run();

  updateCompletedMotion();
  readSerialWithoutBlocking();
}
