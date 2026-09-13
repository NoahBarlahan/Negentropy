#include <AccelStepper.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ===========================================================================
// TWO-AXIS CAMERA CONTROLLER FOR HAND-CENTER FOLLOWING
// ===========================================================================
// Axis assignment:
//   Z axis / Motor 1: DIR 32, STEP 34
//   Y axis / Motor 2: DIR 38, STEP 36
//
// Commands are absolute software angles and may update while either motor is
// moving. This lets Python continuously adjust both targets while following a
// hand. There are intentionally no angular travel limits in this sketch.
//
// Serial commands (9600 baud, Newline line ending):
//   z10     Set the absolute Z target to +10 degrees.
//   z-10    Set the absolute Z target to -10 degrees.
//   y5      Set the absolute Y target to +5 degrees.
//   y-50    Set the absolute Y target to -50 degrees.
//   s       Stop both axes with AccelStepper deceleration.
//   p       Print both current software positions.
//   i       Label current Y=+60, then move Y to -50 degrees.
//   zero    Declare the stationary current pose to be Z=0 and Y=0.
//   h       Print help.
//
// WARNING: Software position is open-loop. Without homing switches or
// encoders, the Mega cannot know its physical position after a restart, stall,
// skipped step, or manual movement. Use a physical power cutoff during tests.


// A4988 pin assignments -----------------------------------------------------
const byte Z_DIR_PIN = 32;
const byte Z_STEP_PIN = 34;
const byte Y_DIR_PIN = 38;
const byte Y_STEP_PIN = 36;

AccelStepper zMotor(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper yMotor(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);


// Resolution and motion ----------------------------------------------------
const long FULL_STEPS_PER_REVOLUTION = 200;
const long MICROSTEPS_PER_FULL_STEP = 8;
const long STEPS_PER_REVOLUTION =
  FULL_STEPS_PER_REVOLUTION * MICROSTEPS_PER_FULL_STEP;

// MOTION TUNING: 180 microsteps/s is 40.5 degrees/s at 1/8 microstepping.
// This remains faster than Python's 22 deg/s PID ceiling while using a
// moderate acceleration that avoids the previous abrupt jumps and vibration.
const float MAX_SPEED_MICROSTEPS_PER_SECOND = 180.0;
const float ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED = 2000.0;
const unsigned int MINIMUM_STEP_PULSE_MICROSECONDS = 2;

// Pressing i assumes the gravity-resting pose is Y=+60 degrees, then moves to
// Y=-50 degrees: a physical -110 degree move. Boot itself causes no movement.
const float STARTUP_Y_REFERENCE_DEGREES = 60.0;
const float STARTUP_Y_TARGET_DEGREES = -50.0;

// These remain available for fixed wiring-direction corrections. Python runs
// full PID on both axes and applies its configured Z direction sign.
const bool REVERSE_Z_DIRECTION = false;
const bool REVERSE_Y_DIRECTION = false;


// Serial and telemetry -----------------------------------------------------
const unsigned long SERIAL_BAUD_RATE = 9600;
const unsigned long POSITION_REPORT_INTERVAL_MILLISECONDS = 100;
const unsigned long ACK_REPORT_INTERVAL_MILLISECONDS = 200;
const byte SERIAL_BUFFER_LENGTH = 48;

char serialBuffer[SERIAL_BUFFER_LENGTH];
byte serialBufferLength = 0;
unsigned long lastPositionReportTime = 0;
unsigned long lastAckReportTime = 0;
bool zWasMoving = false;
bool yWasMoving = false;


float stepsToDegrees(long steps) {
  return steps * 360.0 / STEPS_PER_REVOLUTION;
}


long degreesToSteps(float degrees) {
  return lround(degrees * STEPS_PER_REVOLUTION / 360.0);
}


bool motorsAreStationary() {
  return zMotor.distanceToGo() == 0 && yMotor.distanceToGo() == 0;
}


void printMachinePosition() {
  // TwoAxisCameraControl.py parses this exact compact line.
  Serial.print("POS Z=");
  Serial.print(stepsToDegrees(zMotor.currentPosition()), 2);
  Serial.print(" Y=");
  Serial.println(stepsToDegrees(yMotor.currentPosition()), 2);
}


void printPositions() {
  printMachinePosition();

  Serial.print("Z / Motor 1: ");
  Serial.print(stepsToDegrees(zMotor.currentPosition()), 2);
  Serial.print(" deg, target ");
  Serial.print(stepsToDegrees(zMotor.targetPosition()), 2);
  Serial.println(" deg");

  Serial.print("Y / Motor 2: ");
  Serial.print(stepsToDegrees(yMotor.currentPosition()), 2);
  Serial.print(" deg, target ");
  Serial.print(stepsToDegrees(yMotor.targetPosition()), 2);
  Serial.println(" deg");
}


void printHelp() {
  Serial.println();
  Serial.println("TWO-AXIS CAMERA FOLLOW CONTROLLER");
  Serial.println("----------------------------------------");
  Serial.println("z10   : set absolute Z target to +10 deg");
  Serial.println("z-10  : set absolute Z target to -10 deg");
  Serial.println("y10   : set absolute Y target to +10 deg");
  Serial.println("y-50  : set absolute Y target to -50 deg");
  Serial.println("s     : stop both axes");
  Serial.println("p     : print positions and targets");
  Serial.println("i     : initialize gravity-rest Y=+60, move to -50");
  Serial.println("zero  : make the stationary pose software zero");
  Serial.println("h     : print this help menu");
  Serial.println("----------------------------------------");
  Serial.println("Angular software limits: NONE");
  Serial.println("Use Newline or Both NL & CR line ending.");
  Serial.println();
}


void setAxisTarget(char axis, float targetDegrees) {
  AccelStepper *motor = axis == 'z' ? &zMotor : &yMotor;
  long targetSteps = degreesToSteps(targetDegrees);

  motor->moveTo(targetSteps);
  if (axis == 'z') {
    zWasMoving = zMotor.distanceToGo() != 0;
  } else {
    yWasMoving = yMotor.distanceToGo() != 0;
  }

  // At a 20 Hz two-axis update rate, acknowledging every command would nearly
  // fill a 9600-baud output channel. Position telemetry remains at 10 Hz and
  // this throttled ACK proves that commands continue to arrive.
  unsigned long currentTime = millis();
  if (currentTime - lastAckReportTime >= ACK_REPORT_INTERVAL_MILLISECONDS) {
    lastAckReportTime = currentTime;
    Serial.print("ACK ");
    Serial.print(axis == 'z' ? "Z_TARGET=" : "Y_TARGET=");
    Serial.println(stepsToDegrees(targetSteps), 2);
  }
}


void initializeYFromGravityRest() {
  if (!motorsAreStationary()) {
    Serial.println("REJECTED: stop both axes before Y initialization.");
    return;
  }

  yMotor.setCurrentPosition(degreesToSteps(STARTUP_Y_REFERENCE_DEGREES));
  yMotor.moveTo(degreesToSteps(STARTUP_Y_TARGET_DEGREES));
  yWasMoving = yMotor.distanceToGo() != 0;
  Serial.println("INITIALIZING Y: assumed +60 deg, target -50 deg.");
  printMachinePosition();
}


void stopMotion() {
  zMotor.stop();
  yMotor.stop();
  zWasMoving = zMotor.distanceToGo() != 0;
  yWasMoving = yMotor.distanceToGo() != 0;
  Serial.println("STOP REQUESTED");
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

  if (strcasecmp(command, "i") == 0) {
    initializeYFromGravityRest();
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
    zMotor.moveTo(0);
    yMotor.moveTo(0);
    zWasMoving = false;
    yWasMoving = false;
    Serial.println("Current pose is now software Z=0, Y=0.");
    printMachinePosition();
    return;
  }

  char axis = tolower(command[0]);
  if (axis != 'z' && axis != 'y') {
    Serial.println("INVALID: use z<number>, y<number>, i, s, p, zero, or h.");
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

  setAxisTarget(axis, targetDegrees);
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


void updateMotionState() {
  bool zMoving = zMotor.distanceToGo() != 0;
  bool yMoving = yMotor.distanceToGo() != 0;
  bool anAxisFinished = false;

  if (zWasMoving && !zMoving) {
    Serial.println("DONE Z");
    anAxisFinished = true;
  }
  if (yWasMoving && !yMoving) {
    Serial.println("DONE Y");
    anAxisFinished = true;
  }

  // Short corrections can finish before the periodic report interval. Always
  // publish the final position so Python never builds a target from stale data.
  if (anAxisFinished) {
    printMachinePosition();
  }

  zWasMoving = zMoving;
  yWasMoving = yMoving;

  unsigned long currentTime = millis();
  if (
    (zMoving || yMoving)
    && currentTime - lastPositionReportTime
      >= POSITION_REPORT_INTERVAL_MILLISECONDS
  ) {
    lastPositionReportTime = currentTime;
    printMachinePosition();
  }
}


void configureMotor(
  AccelStepper &motor,
  bool reverseDirection,
  long assumedStartingPositionSteps
) {
  motor.setMaxSpeed(MAX_SPEED_MICROSTEPS_PER_SECOND);
  motor.setAcceleration(ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED);
  motor.setMinPulseWidth(MINIMUM_STEP_PULSE_MICROSECONDS);
  motor.setPinsInverted(reverseDirection, false, false);

  // This is an unverified software reference, not physical homing.
  motor.setCurrentPosition(assumedStartingPositionSteps);
  motor.moveTo(assumedStartingPositionSteps);
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  configureMotor(zMotor, REVERSE_Z_DIRECTION, 0);
  configureMotor(yMotor, REVERSE_Y_DIRECTION, 0);

  Serial.println();
  Serial.println("TWO-AXIS CAMERA FOLLOW CONTROLLER READY");
  Serial.print("Resolution: ");
  Serial.print(STEPS_PER_REVOLUTION);
  Serial.println(" microsteps/revolution");
  Serial.print("Maximum speed: ");
  Serial.print(
    360.0 * MAX_SPEED_MICROSTEPS_PER_SECOND / STEPS_PER_REVOLUTION,
    1
  );
  Serial.println(" deg/s");
  Serial.println("Angular software limits: NONE");
  Serial.println("Boot causes no motion. Press i to initialize Y +60 -> -50.");
  Serial.println("CAUTION: no homing, limit switches, or encoders are present.");
  printHelp();
}


void loop() {
  // Both motors are serviced every pass, so they can move simultaneously.
  zMotor.run();
  yMotor.run();

  updateMotionState();
  readSerialWithoutBlocking();
}
