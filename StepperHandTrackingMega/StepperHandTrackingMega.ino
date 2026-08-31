#include <AccelStepper.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// A4988 wiring
// ---------------------------------------------------------------------------
const byte DIR_PIN = 32;
const byte STEP_PIN = 34;

// AccelStepper::DRIVER is the STEP/DIR interface used by the A4988.
// Constructor order is STEP pin first, then DIR pin.
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ---------------------------------------------------------------------------
// Motor settings to tune
// ---------------------------------------------------------------------------
// A common 1.8-degree NEMA 17 has 200 full steps per revolution. If MS1, MS2,
// or MS3 are later enabled, multiply this by the microstep setting.
const long STEPS_PER_REVOLUTION = 200;

// Conservative starting values. Increase gradually after confirming the A4988
// current limit, motor power, load, and mechanics are correct.
const float MAX_SPEED_STEPS_PER_SECOND = 300.0;
const float ACCELERATION_STEPS_PER_SECOND_SQUARED = 250.0;
const unsigned int MINIMUM_STEP_PULSE_MICROSECONDS = 2;

// ---------------------------------------------------------------------------
// Hand-input calibration and smoothing
// ---------------------------------------------------------------------------
// The current ServoCameraSync.py sends values from 0 through 180, so those are
// the default calibration endpoints. If the computer later sends normalized
// values, use 0.0 and 1.0. For 1280-pixel X coordinates, use 0.0 and 1279.0.
const float HAND_INPUT_MINIMUM = 0.0;
const float HAND_INPUT_MAXIMUM = 180.0;

// Exponential moving-average strength copied from the servo tracker. Lower is
// smoother/slower. Use 1.0 to disable Arduino-side smoothing when the computer
// has already smoothed the values enough.
const float HAND_SMOOTHING_ALPHA = 0.18;

// Set true if physical motion is opposite the desired camera direction.
const bool REVERSE_HAND_DIRECTION = false;

// Status is throttled so Serial printing does not interfere with step timing.
const unsigned long STATUS_INTERVAL_MILLISECONDS = 250;

// ---------------------------------------------------------------------------
// Serial protocol
// ---------------------------------------------------------------------------
// At 9600 baud, send one command per line:
//   g       Enable hand tracking.
//   s       Disable tracking and decelerate to a stop.
//   90      Hand value as a bare number.
//   H90     The same hand value with an optional H prefix.
//   H,90    The same hand value with an optional comma.
const unsigned long SERIAL_BAUD_RATE = 9600;
const byte SERIAL_BUFFER_LENGTH = 48;
char serialBuffer[SERIAL_BUFFER_LENGTH];
byte serialBufferLength = 0;

bool trackingEnabled = false;
bool hasFilteredHandValue = false;
float filteredHandValue = 0.0;
unsigned long lastStatusTime = 0;
unsigned long lastDisabledMessageTime = 0;


float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) {
    return minimumValue;
  }
  if (value > maximumValue) {
    return maximumValue;
  }
  return value;
}


long positiveModulo(long value, long modulus) {
  long result = value % modulus;
  return result < 0 ? result + modulus : result;
}


float handValueToAngle(float handValue) {
  float calibratedValue = clampFloat(
    handValue,
    HAND_INPUT_MINIMUM,
    HAND_INPUT_MAXIMUM
  );
  float inputRange = HAND_INPUT_MAXIMUM - HAND_INPUT_MINIMUM;
  float normalizedPosition = 0.0;

  if (inputRange > 0.0) {
    normalizedPosition = (
      calibratedValue - HAND_INPUT_MINIMUM
    ) / inputRange;
  }

  if (REVERSE_HAND_DIRECTION) {
    normalizedPosition = 1.0 - normalizedPosition;
  }

  return normalizedPosition * 360.0;
}


long angleToCanonicalStep(float targetAngleDegrees) {
  long stepPosition = lround(
    targetAngleDegrees
    * static_cast<float>(STEPS_PER_REVOLUTION)
    / 360.0
  );

  // 0 degrees and 360 degrees represent the same physical orientation.
  return positiveModulo(stepPosition, STEPS_PER_REVOLUTION);
}


long nearestEquivalentTarget(long canonicalTargetStep) {
  long currentAbsoluteStep = stepper.currentPosition();
  long currentCanonicalStep = positiveModulo(
    currentAbsoluteStep,
    STEPS_PER_REVOLUTION
  );
  long stepDifference = canonicalTargetStep - currentCanonicalStep;
  long halfRevolution = STEPS_PER_REVOLUTION / 2;

  // Select the equivalent target no more than half a revolution away. This
  // avoids unnecessary full rotations when crossing between 0 and 360 degrees.
  if (stepDifference > halfRevolution) {
    stepDifference -= STEPS_PER_REVOLUTION;
  } else if (stepDifference < -halfRevolution) {
    stepDifference += STEPS_PER_REVOLUTION;
  }

  return currentAbsoluteStep + stepDifference;
}


void printTargetStatus(
  float receivedHandValue,
  float smoothedHandValue,
  float targetAngleDegrees,
  long canonicalTargetStep,
  long absoluteTargetStep
) {
  unsigned long currentTime = millis();
  if (currentTime - lastStatusTime < STATUS_INTERVAL_MILLISECONDS) {
    return;
  }
  lastStatusTime = currentTime;

  Serial.print("HAND=");
  Serial.print(receivedHandValue, 2);
  Serial.print(" FILTERED=");
  Serial.print(smoothedHandValue, 2);
  Serial.print(" ANGLE=");
  Serial.print(targetAngleDegrees, 1);
  Serial.print("deg TARGET_STEP=");
  Serial.print(canonicalTargetStep);
  Serial.print(" MOVE_TO=");
  Serial.print(absoluteTargetStep);
  Serial.print(" CURRENT=");
  Serial.println(stepper.currentPosition());
}


void processHandValue(float receivedHandValue) {
  if (!trackingEnabled) {
    unsigned long currentTime = millis();
    if (
      currentTime - lastDisabledMessageTime
      >= STATUS_INTERVAL_MILLISECONDS
    ) {
      Serial.println("IGNORED: tracking disabled; send g first.");
      lastDisabledMessageTime = currentTime;
    }
    return;
  }

  float calibratedHandValue = clampFloat(
    receivedHandValue,
    HAND_INPUT_MINIMUM,
    HAND_INPUT_MAXIMUM
  );

  if (!hasFilteredHandValue) {
    filteredHandValue = calibratedHandValue;
    hasFilteredHandValue = true;
  } else {
    float alpha = clampFloat(HAND_SMOOTHING_ALPHA, 0.0001, 1.0);
    filteredHandValue += alpha * (
      calibratedHandValue - filteredHandValue
    );
  }

  float targetAngleDegrees = handValueToAngle(filteredHandValue);
  long canonicalTargetStep = angleToCanonicalStep(targetAngleDegrees);
  long absoluteTargetStep = nearestEquivalentTarget(canonicalTargetStep);

  // moveTo() only changes the destination. Repeated calls to run() in loop()
  // generate non-blocking accelerated/decelerated steps toward that target.
  stepper.moveTo(absoluteTargetStep);

  printTargetStatus(
    receivedHandValue,
    filteredHandValue,
    targetAngleDegrees,
    canonicalTargetStep,
    absoluteTargetStep
  );
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

  if (
    (command[0] == 'g' || command[0] == 'G')
    && command[1] == '\0'
  ) {
    trackingEnabled = true;
    hasFilteredHandValue = false;
    lastStatusTime = millis() - STATUS_INTERVAL_MILLISECONDS;
    Serial.println("TRACKING ENABLED: waiting for hand values.");
    return;
  }

  if (
    (command[0] == 's' || command[0] == 'S')
    && command[1] == '\0'
  ) {
    trackingEnabled = false;
    hasFilteredHandValue = false;
    stepper.stop();
    Serial.println("TRACKING DISABLED: decelerating to a stop.");
    return;
  }

  // Allow an optional H prefix and separator before the numeric hand value.
  if (command[0] == 'h' || command[0] == 'H') {
    command++;
    while (*command == ',' || *command == ':' || isspace(*command)) {
      command++;
    }
  }

  char *parseEnd = nullptr;
  float handValue = strtod(command, &parseEnd);
  parseEnd = trimWhitespace(parseEnd);
  if (
    parseEnd == command
    || *parseEnd != '\0'
    || isnan(handValue)
    || isinf(handValue)
  ) {
    Serial.println("INVALID: send g, s, or a numeric hand value.");
    return;
  }

  processHandValue(handValue);
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

    // Keep servicing the motor even while several serial bytes are waiting.
    stepper.run();
  }
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  stepper.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
  stepper.setAcceleration(ACCELERATION_STEPS_PER_SECOND_SQUARED);
  stepper.setMinPulseWidth(MINIMUM_STEP_PULSE_MICROSECONDS);

  // This establishes a software coordinate only. It does not measure the
  // motor's real mechanical orientation.
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);

  Serial.println();
  Serial.println("MEGA A4988 HAND-TRACKING STEPPER READY");
  Serial.println("Motor is stationary. Send g to enable tracking.");
  Serial.println("Then send hand values from 0 to 180, one per line.");
  Serial.println("Send s to disable tracking and decelerate to a stop.");
  Serial.println(
    "WARNING: position 0 is only a software startup reference."
  );
  Serial.println(
    "Use a limit switch or homing sensor for a known physical reference."
  );
}


void loop() {
  // run() is non-blocking and must be called as often as possible. It performs
  // at most one due step while applying the configured acceleration profile.
  stepper.run();
  readSerialWithoutBlocking();
}
