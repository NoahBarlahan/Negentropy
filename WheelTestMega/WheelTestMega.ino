/* Standalone test-stand wheel test: Arduino Mega 2560 + two A4988 drivers.
   No camera, IMU, computer Python program, or balancing initialization needed.
   Upload this sketch, open Serial Monitor at 115200 baud with Newline ending.

   l90 / l-90   Left wheel forward/backward by 90 degrees
   r90 / r-90   Right wheel forward/backward by 90 degrees
   b90 / b-90   Both wheels forward/backward by 90 degrees
   t90 / t-90   Wheels in opposite directions (turning test on the stand)
   s           Stop immediately (also STOP)
   speed30     Set wheel speed in degrees/s (5-90, only while stopped)
   micro1/2/4/8/16  Match software scaling to the physical MS1/MS2/MS3 wiring
   invert l    Reverse left wheel's definition of forward (only while stopped)
   invert r    Reverse right wheel's definition of forward
   p           Print software step counts / angles and direction settings
   zero        Reset software counts; no movement (only while stopped)
   h           Help

   All moves are relative, limited to +/-360 degrees, and finish automatically.
   Speed starts at 30 degrees/s, with gentle acceleration. No startup movement.
   Lift both wheels clear of the floor; positive motion should drive the robot
   forward if put on the floor. Mirrored motors need opposite electrical DIR.
   Rotation is counted STEP pulses, not an encoder measurement of wheel motion.
   Disconnecting USB does not cancel an already accepted finite move. The move
   finishes automatically; a 90-second deadline is an additional fallback.
*/
#include <AccelStepper.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega 2560 for this sketch.
#endif

const uint8_t LEFT_DIR_PIN = 32, LEFT_STEP_PIN = 34;
const uint8_t RIGHT_DIR_PIN = 38, RIGHT_STEP_PIN = 36;
const long FULL_STEPS_PER_REVOLUTION = 200;
const long DEFAULT_MICROSTEPS = 8; // MS1 HIGH, MS2 HIGH, MS3 LOW
long microsteps = DEFAULT_MICROSTEPS;
long stepsPerRevolution = FULL_STEPS_PER_REVOLUTION * DEFAULT_MICROSTEPS;
const float WHEEL_DIAMETER_MM = 127.0; // 5 inches, direct drive
const float ACCELERATION_DEG_S2 = 90.0;
const unsigned long MOVE_TIMEOUT_MS = 90000;

// These match the balancing dashboard's default signs: left +1, right -1.
// Use invert l/r during testing; copy the printed signs into the dashboard.
const bool DEFAULT_REVERSE_LEFT = false;
const bool DEFAULT_REVERSE_RIGHT = true;

class DiagnosticStepper : public AccelStepper {
 public:
  DiagnosticStepper(uint8_t stepPin, uint8_t dirPin)
      : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin), pulses(0) {}
  unsigned long pulses;
 protected:
  void step1(long step) override {
    AccelStepper::step1(step); // Actually execute the driver's STEP GPIO writes.
    ++pulses;                 // This is NOT an encoder or electrical measurement.
  }
};

DiagnosticStepper leftMotor(LEFT_STEP_PIN, LEFT_DIR_PIN);
DiagnosticStepper rightMotor(RIGHT_STEP_PIN, RIGHT_DIR_PIN);
bool reverseLeft = DEFAULT_REVERSE_LEFT, reverseRight = DEFAULT_REVERSE_RIGHT;
float speedDegS = 30.0;
bool moving = false;
unsigned long moveStarted = 0;
char rx[64];
uint8_t rxLength = 0;
bool discardLine = false;
unsigned long commandId = 0, moveId = 0;
unsigned long expectedLeft = 0, expectedRight = 0;
unsigned long lastProgress = 0;
bool reportedLeftOutput = false, reportedRightOutput = false;
char diagnosticTx[256];
uint8_t txHead = 0, txTail = 0;
unsigned long droppedDiagnostics = 0;
unsigned long lastByteAt = 0;

void queueDiagnostic(const char *message) {
  size_t length = strlen(message);
  uint8_t freeBytes = (uint8_t)(txTail - txHead - 1);
  if (length > freeBytes) { ++droppedDiagnostics; return; }
  while (*message) { diagnosticTx[txHead++] = *message++; }
}

void flushDiagnostic() {
  // Bounded, nonblocking serial output so diagnostics do not starve run().
  for (uint8_t n = 0; n < 24 && txTail != txHead && Serial.availableForWrite(); ++n) {
    Serial.write((uint8_t)diagnosticTx[txTail++]);
  }
}

void drainDiagnosticWhileStopped() {
  while (txTail != txHead) flushDiagnostic();
}

void reportPulses(const char *prefix) {
  char message[96];
  snprintf(message, sizeof(message), "%s L=%lu/%lu R=%lu/%lu (GPIO writes; no encoder)\n",
           prefix, leftMotor.pulses, expectedLeft, rightMotor.pulses, expectedRight);
  queueDiagnostic(message);
}

void reportLiveOutput() {
  char message[80];
  if (leftMotor.pulses && !reportedLeftOutput) {
    snprintf(message, sizeof(message), "OUTPUT L: STEP writes started GPIO34; DIR32=%s\n",
             digitalRead(LEFT_DIR_PIN) ? "HIGH" : "LOW");
    queueDiagnostic(message); reportedLeftOutput = true;
  }
  if (rightMotor.pulses && !reportedRightOutput) {
    snprintf(message, sizeof(message), "OUTPUT R: STEP writes started GPIO36; DIR38=%s\n",
             digitalRead(RIGHT_DIR_PIN) ? "HIGH" : "LOW");
    queueDiagnostic(message); reportedRightOutput = true;
  }
  if (millis() - lastProgress >= 1000) {
    reportPulses("PROGRESS");
    lastProgress = millis();
  }
}

float stepsToDegrees(long steps) {
  return steps * (360.0 / stepsPerRevolution);
}

void printStatus() {
  reportPulses("LAST MOVE OUTPUT");
  drainDiagnosticWhileStopped();
  Serial.print(F("Left: ")); Serial.print(leftMotor.currentPosition());
  Serial.print(F(" commanded steps / ")); Serial.print(stepsToDegrees(leftMotor.currentPosition()), 2);
  Serial.print(F(" deg; forward sign ")); Serial.println(reverseLeft ? -1 : 1);
  Serial.print(F("Right: ")); Serial.print(rightMotor.currentPosition());
  Serial.print(F(" commanded steps / ")); Serial.print(stepsToDegrees(rightMotor.currentPosition()), 2);
  Serial.print(F(" deg; forward sign ")); Serial.println(reverseRight ? -1 : 1);
  Serial.print(F("Speed ")); Serial.print(speedDegS, 1);
  Serial.println(F(" deg/s; counts are NOT measured wheel position."));
  Serial.print(F("Commands received: ")); Serial.print(commandId);
  Serial.print(F("; last accepted move: ")); Serial.println(moveId);
  Serial.print(F("Diagnostic lines dropped: ")); Serial.println(droppedDiagnostics);
  Serial.print(F("SOFTWARE microsteps=")); Serial.print(microsteps);
  Serial.print(F("; pulses/revolution=")); Serial.println(stepsPerRevolution);
  Serial.println(F("MS1/MS2/MS3 are not connected to this software: check actual driver wiring."));
}

void printHelp() {
  Serial.println(F("WHEEL TEST -- Mega/A4988, 115200 baud, Newline ending"));
  Serial.println(F("No startup movement. Begin with l30, r30 while on the stand."));
  Serial.println(F("l90/r90: one wheel; b90: both forward; t90: opposite directions."));
  Serial.println(F("Negative values reverse motion. Every move is relative, +/-360 deg max."));
  Serial.println(F("s/STOP: immediate stop; p: status; zero: reset software counts."));
  Serial.println(F("speed30: 5-90 deg/s; invert l / invert r: change forward sign."));
  Serial.println(F("micro1/2/4/8/16: match software to driver wiring; requires stopped wheels."));
  Serial.println(F("Direction/speed/zero changes require both wheels stopped."));
  Serial.println(F("USB disconnect: accepted move still finishes. Use motor power switch if needed."));
  Serial.println(F("RX=received; ACCEPTED=scheduled; OUTPUT/PROGRESS=STEP GPIO writes executed."));
  Serial.println(F("Pulses do NOT prove driver reception or wheel rotation. Send p for details."));
}

void stopNow() {
  // setCurrentPosition also zeroes speed and the target, so run() emits no
  // further pulses. AccelStepper.stop() would instead schedule deceleration.
  leftMotor.setCurrentPosition(leftMotor.currentPosition());
  rightMotor.setCurrentPosition(rightMotor.currentPosition());
  moving = false;
}

char *trim(char *text) {
  while (isspace((unsigned char)*text)) ++text;
  char *end = text + strlen(text);
  while (end > text && isspace((unsigned char)end[-1])) --end;
  *end = 0;
  return text;
}

bool parseNumber(char *text, float &value) {
  text = trim(text);
  char *end;
  double parsed = strtod(text, &end);
  if (end == text || *trim(end) || isnan(parsed) || isinf(parsed)) return false;
  value = (float)parsed;
  return true;
}

void startMove(char axis, float degrees) {
  if (moving) { Serial.println(F("Wait for DONE or send s before a new move.")); return; }
  if (degrees < -360 || degrees > 360) {
    Serial.println(F("REJECTED: angle outside +/-360 degrees. NO MOVE SENT."));
    return;
  }
  long steps = lround(degrees * stepsPerRevolution / 360.0);
  if (steps == 0) { Serial.println(F("REJECTED: zero steps. NO MOVE SENT.")); return; }
  bool useLeft = axis == 'l' || axis == 'b' || axis == 't';
  bool useRight = axis == 'r' || axis == 'b' || axis == 't';
  long rightSteps = axis == 't' ? -steps : steps;
  // Report before starting motion; serial printing must not pause step timing.
  Serial.print(F("ACCEPTED MOVE #")); Serial.print(commandId);
  Serial.print(F(" ")); Serial.print(axis);
  Serial.print(F(" relative ")); Serial.print(stepsToDegrees(steps), 2);
  Serial.print(F(" deg at ")); Serial.print(speedDegS, 1); Serial.println(F(" deg/s max."));
  Serial.print(F("STEP pulses requested: L=")); Serial.print(useLeft ? steps : 0);
  Serial.print(F(" R=")); Serial.println(useRight ? rightSteps : 0);
  leftMotor.pulses = rightMotor.pulses = 0;
  expectedLeft = useLeft ? labs(steps) : 0;
  expectedRight = useRight ? labs(rightSteps) : 0;
  reportedLeftOutput = reportedRightOutput = false;
  moveId = commandId;
  lastProgress = millis();
  leftMotor.move(useLeft ? steps : 0);
  rightMotor.move(useRight ? rightSteps : 0);
  moving = true;
  moveStarted = millis();
}

void processLine() {
  char *command = trim(rx);
  for (char *p = command; *p; ++p) *p = tolower((unsigned char)*p);
  if (!*command) return;
  ++commandId;
  char message[96];
  snprintf(message, sizeof(message), "RX #%lu: [%s]\n", commandId, command);
  queueDiagnostic(message);
  if (!strcmp(command, "s") || !strcmp(command, "stop")) {
    stopNow();
    reportPulses("STOPPED OUTPUT"); drainDiagnosticWhileStopped();
    Serial.println(F("STOPPED: no further STEP pulses.")); return;
  }
  // All logging during motion is buffered; p requests a nonblocking pulse report.
  if (moving) {
    if (!strcmp(command, "p")) reportPulses("PROGRESS");
    else queueDiagnostic("REJECTED: busy. NO NEW MOVE SENT; wait or send s.\n");
    return;
  }
  drainDiagnosticWhileStopped();
  if (!strcmp(command, "h") || !strcmp(command, "help")) { printHelp(); return; }
  if (!strcmp(command, "p")) { printStatus(); return; }
  if (!strcmp(command, "zero")) {
    leftMotor.setCurrentPosition(0); rightMotor.setCurrentPosition(0);
    Serial.println(F("Software counts reset; wheels have not moved.")); return;
  }
  if (!strcmp(command, "invert l") || !strcmp(command, "invert r")) {
    if (command[7] == 'l') {
      reverseLeft = !reverseLeft;
      leftMotor.setPinsInverted(reverseLeft, false, false);
      leftMotor.setCurrentPosition(0);
    } else {
      reverseRight = !reverseRight;
      rightMotor.setPinsInverted(reverseRight, false, false);
      rightMotor.setCurrentPosition(0);
    }
    Serial.println(F("Forward sign changed; this wheel's software count reset to zero."));
    printStatus(); return;
  }
  if (!strncmp(command, "micro", 5)) {
    float value;
    if (!parseNumber(command + 5, value) ||
        !(value == 1 || value == 2 || value == 4 || value == 8 || value == 16)) {
      Serial.println(F("REJECTED: use micro1, micro2, micro4, micro8 or micro16.")); return;
    }
    microsteps = (long)value;
    stepsPerRevolution = FULL_STEPS_PER_REVOLUTION * microsteps;
    leftMotor.setCurrentPosition(0); rightMotor.setCurrentPosition(0);
    leftMotor.pulses = rightMotor.pulses = 0;
    expectedLeft = expectedRight = 0;
    leftMotor.setMaxSpeed(speedDegS * stepsPerRevolution / 360.0);
    rightMotor.setMaxSpeed(speedDegS * stepsPerRevolution / 360.0);
    leftMotor.setAcceleration(ACCELERATION_DEG_S2 * stepsPerRevolution / 360.0);
    rightMotor.setAcceleration(ACCELERATION_DEG_S2 * stepsPerRevolution / 360.0);
    Serial.println(F("SOFTWARE scaling changed; counts reset. Physical microstep pins NOT changed."));
    Serial.println(F("Required MS1/MS2/MS3: (H=VDD, L=GND)"));
    if (microsteps == 1) Serial.println(F("L L L"));
    else if (microsteps == 2) Serial.println(F("H L L"));
    else if (microsteps == 4) Serial.println(F("L H L"));
    else if (microsteps == 8) Serial.println(F("H H L"));
    else Serial.println(F("H H H"));
    printStatus(); return;
  }
  if (!strncmp(command, "speed", 5)) {
    float value;
    if (!parseNumber(command + 5, value) || value < 5 || value > 90) {
      Serial.println(F("REJECTED: use speed5 through speed90. NO MOVE SENT.")); return;
    }
    speedDegS = value;
    leftMotor.setMaxSpeed(speedDegS * stepsPerRevolution / 360.0);
    rightMotor.setMaxSpeed(speedDegS * stepsPerRevolution / 360.0);
    printStatus(); return;
  }
  char axis = command[0];
  float degrees;
  if ((axis == 'l' || axis == 'r' || axis == 'b' || axis == 't') &&
      parseNumber(command + 1, degrees)) { startMove(axis, degrees); return; }
  Serial.println(F("REJECTED: unknown command. NO MOVE SENT. Send h for help."));
}

void setup() {
  Serial.begin(115200);
  leftMotor.setPinsInverted(reverseLeft, false, false);
  rightMotor.setPinsInverted(reverseRight, false, false);
  leftMotor.setMinPulseWidth(3); rightMotor.setMinPulseWidth(3);
  leftMotor.setMaxSpeed(speedDegS * stepsPerRevolution / 360.0);
  rightMotor.setMaxSpeed(speedDegS * stepsPerRevolution / 360.0);
  leftMotor.setAcceleration(ACCELERATION_DEG_S2 * stepsPerRevolution / 360.0);
  rightMotor.setAcceleration(ACCELERATION_DEG_S2 * stepsPerRevolution / 360.0);
  stopNow();
  Serial.println(F("READY WHEELTEST v3: startup stopped; waiting for a newline command."));
  printHelp();
  printStatus();
  Serial.print(F("Wheel circumference: ")); Serial.print(PI * WHEEL_DIAMETER_MM, 1);
  Serial.print(F(" mm. One full revolution = ")); Serial.print(stepsPerRevolution);
  Serial.println(F(" pulses at the configured microstepping."));
}

void loop() {
  leftMotor.run(); rightMotor.run();
  for (uint8_t count = 0; count < 24 && Serial.available(); ++count) {
    char c = Serial.read();
    lastByteAt = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discardLine) { rx[rxLength] = 0; processLine(); }
      rxLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (c == 0 || rxLength >= sizeof(rx) - 1) {
        stopNow(); rxLength = 0; discardLine = true;
        Serial.println(F("STOPPED: invalid or oversized input."));
      } else rx[rxLength++] = c;
    }
    leftMotor.run(); rightMotor.run();
  }
  // Catch a missing line ending instead of silently waiting indefinitely.
  if (rxLength && millis() - lastByteAt >= 1500) {
    queueDiagnostic("NOT ACCEPTED: no newline received. Select Newline in Serial Monitor.\n");
    rxLength = 0;
  }
  if (moving) reportLiveOutput();
  flushDiagnostic();
  if (moving && millis() - moveStarted >= MOVE_TIMEOUT_MS) {
    stopNow(); reportPulses("TIMEOUT OUTPUT"); drainDiagnosticWhileStopped();
    Serial.println(F("STOPPED: move time limit reached."));
  } else if (moving && leftMotor.distanceToGo() == 0 && rightMotor.distanceToGo() == 0) {
    moving = false;
    reportPulses("DONE OUTPUT"); drainDiagnosticWhileStopped();
    bool allSent = leftMotor.pulses == expectedLeft && rightMotor.pulses == expectedRight;
    Serial.println(allSent ? F("DONE: all requested STEP GPIO writes executed; wheel movement unverified.")
                           : F("ERROR: requested pulse count differs from GPIO writes."));
    printStatus();
  }
}
