/* Two-wheel balance pulse generator, Arduino Mega 2560 / A4988.
   Same wires as the camera prototype: motor 1 DIR32 STEP34, motor 2 DIR38 STEP36.
   Both drivers must retain the original 1/8 microstep wiring (MS1/2 high, MS3 low).
   Timer1 supplies simultaneous, nonblocking pulse trains; no extra library.
   USB serial 115200. Startup, STOP, malformed packets and 120ms silence stop steps.
   This stops pulse output, not falling or motor supply power. Optional EN pins
   below disable A4988 outputs when disarmed; -1 preserves existing wiring.

   H -> BAL1 2000 120
   A <nonzero session> -> arm at zero speed
   V <session> <increasing sequence> <left steps/s> <right steps/s>
   S -> stop and disarm
   T <session> <sequence> <armed> <fault> <left> <right> (25Hz status)
   fault: 0=none, 1=timeout, 2=bad packet. Timed-out sessions cannot be rearmed.
   Timer1 is reserved: do not combine with Servo or other Timer1 users.
*/
#include <Arduino.h>
#include <util/atomic.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega 2560 for this sketch.
#endif

const uint8_t LEFT_DIR = 32, LEFT_STEP = 34;
const uint8_t RIGHT_DIR = 38, RIGHT_STEP = 36;
const int8_t LEFT_ENABLE = -1, RIGHT_ENABLE = -1; // Optional active-low EN wiring
const uint16_t TICK_HZ = 20000;
const int16_t MAX_STEPS_S = 2000;
const uint16_t WATCHDOG_TICKS = 2400; // 120ms, independent of main-loop stalls

volatile uint16_t leftRate = 0, rightRate = 0, ageTicks = 0;
volatile uint16_t leftPhase = 0, rightPhase = 0;
volatile bool armed = false, leftHigh = false, rightHigh = false;
volatile uint8_t fault = 0;
volatile int16_t leftCommand = 0, rightCommand = 0;
bool leftDirNegative = false, rightDirNegative = false; // ISR-only after setup
uint32_t session = 0, sequence = 0;
char rx[80];
uint8_t rxLength = 0;
bool discardLine = false;
unsigned long lastStatus = 0;

void enableDrivers(bool enabled) {
  if (LEFT_ENABLE >= 0) digitalWrite(LEFT_ENABLE, enabled ? LOW : HIGH);
  if (RIGHT_ENABLE >= 0) digitalWrite(RIGHT_ENABLE, enabled ? LOW : HIGH);
}

void stopMotors(uint8_t reason) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    armed = false;
    leftRate = rightRate = 0;
    leftPhase = rightPhase = 0;
    digitalWrite(LEFT_STEP, LOW);
    digitalWrite(RIGHT_STEP, LOW);
    leftHigh = rightHigh = false;
    fault = reason;
  }
  leftCommand = rightCommand = 0;
  enableDrivers(false);
}

ISR(TIMER1_COMPA_vect) {
  bool wasLeftHigh = leftHigh, wasRightHigh = rightHigh;
  if (leftHigh) { digitalWrite(LEFT_STEP, LOW); leftHigh = false; }
  if (rightHigh) { digitalWrite(RIGHT_STEP, LOW); rightHigh = false; }
  if (!armed) return;
  if (++ageTicks >= WATCHDOG_TICKS) {
    armed = false;
    leftRate = rightRate = 0;
    fault = 1;
    return;
  }
  // On reversal, preserve a full pulse and a full direction-hold/setup tick.
  // Main-loop commands never truncate a pulse or change DIR during a pulse.
  bool leftReversing = (leftCommand < 0) != leftDirNegative;
  bool rightReversing = (rightCommand < 0) != rightDirNegative;
  if (leftReversing) {
    leftPhase = 0;
    if (!wasLeftHigh) {
      leftDirNegative = leftCommand < 0;
      digitalWrite(LEFT_DIR, leftDirNegative ? LOW : HIGH);
    }
  } else leftPhase += leftRate;
  if (rightReversing) {
    rightPhase = 0;
    if (!wasRightHigh) {
      rightDirNegative = rightCommand < 0;
      digitalWrite(RIGHT_DIR, rightDirNegative ? LOW : HIGH);
    }
  } else rightPhase += rightRate;
  if (leftPhase >= TICK_HZ) {
    leftPhase -= TICK_HZ;
    digitalWrite(LEFT_STEP, HIGH);
    leftHigh = true;
  }
  if (rightPhase >= TICK_HZ) {
    rightPhase -= TICK_HZ;
    digitalWrite(RIGHT_STEP, HIGH);
    rightHigh = true;
  }
}

bool parseUnsigned(char *token, uint32_t &value) {
  if (!token || !*token) return false;
  for (char *p = token; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char *end;
  value = strtoul(token, &end, 10);
  return !errno && !*end;
}

bool parseSpeed(char *token, int16_t &value) {
  if (!token || !*token) return false;
  char *start = token;
  if (*start == '-' || *start == '+') ++start;
  if (!*start) return false;
  for (char *p = start; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char *end;
  long parsed = strtol(token, &end, 10);
  if (errno || *end || parsed < -MAX_STEPS_S || parsed > MAX_STEPS_S) return false;
  value = (int16_t)parsed;
  return true;
}

void processLine() {
  char *context;
  char *command = strtok_r(rx, " \t", &context);
  if (!command) { stopMotors(2); return; }
  if (!strcmp(command, "S") && !strtok_r(NULL, " \t", &context)) {
    stopMotors(0);
    return;
  }
  if (!strcmp(command, "H") && !strtok_r(NULL, " \t", &context)) {
    // Never block the main loop waiting for the serial transmit buffer.
    if (Serial.availableForWrite() >= 16) Serial.println(F("BAL1 2000 120"));
    return;
  }
  uint32_t incomingSession;
  if (!strcmp(command, "A")) {
    bool valid = parseUnsigned(strtok_r(NULL, " \t", &context), incomingSession);
    if (!valid || !incomingSession || incomingSession == session ||
        strtok_r(NULL, " \t", &context)) { stopMotors(2); return; }
    stopMotors(0);
    session = incomingSession;
    sequence = 0;
    enableDrivers(true);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ageTicks = 0; armed = true; }
    return;
  }
  if (!strcmp(command, "V")) {
    uint32_t incomingSequence;
    int16_t left, right;
    bool valid = parseUnsigned(strtok_r(NULL, " \t", &context), incomingSession);
    valid = parseUnsigned(strtok_r(NULL, " \t", &context), incomingSequence) && valid;
    valid = parseSpeed(strtok_r(NULL, " \t", &context), left) && valid;
    valid = parseSpeed(strtok_r(NULL, " \t", &context), right) && valid;
    if (!valid || strtok_r(NULL, " \t", &context)) { stopMotors(2); return; }
    // Delayed commands never restart a stopped robot or refresh its watchdog.
    if (!armed || incomingSession != session || incomingSequence <= sequence) return;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // Check again: the timer watchdog could have fired during parsing.
      if (armed) {
        leftRate = abs(left); rightRate = abs(right);
        leftCommand = left; rightCommand = right;
        sequence = incomingSequence;
        ageTicks = 0;
      }
    }
    return;
  }
  stopMotors(2);
}

void setup() {
  pinMode(LEFT_DIR, OUTPUT); pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_STEP, OUTPUT); pinMode(RIGHT_STEP, OUTPUT);
  digitalWrite(LEFT_DIR, HIGH); digitalWrite(RIGHT_DIR, HIGH);
  if (LEFT_ENABLE >= 0) pinMode(LEFT_ENABLE, OUTPUT);
  if (RIGHT_ENABLE >= 0) pinMode(RIGHT_ENABLE, OUTPUT);
  stopMotors(0);
  Serial.begin(115200);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
    OCR1A = (F_CPU / 8UL / TICK_HZ) - 1;
    TCCR1B = _BV(WGM12) | _BV(CS11); // CTC, prescaler 8
    TIMSK1 = _BV(OCIE1A);
  }
}

void loop() {
  // A bounded batch keeps command floods from starving other housekeeping.
  for (uint8_t count = 0; count < 48 && Serial.available(); ++count) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discardLine) { rx[rxLength] = 0; processLine(); }
      rxLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (c == 0 || rxLength >= sizeof(rx) - 1) {
        stopMotors(2); rxLength = 0; discardLine = true;
      } else rx[rxLength++] = c;
    }
  }
  if (!armed) { leftCommand = rightCommand = 0; enableDrivers(false); }
  unsigned long now = millis();
  if (now - lastStatus >= 40) {
    char tx[64];
    bool isArmed;
    uint8_t faultCode;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { isArmed = armed; faultCode = fault; }
    int length = snprintf(tx, sizeof(tx), "T %lu %lu %u %u %d %d\n",
                          (unsigned long)session, (unsigned long)sequence,
                          isArmed, faultCode, isArmed ? leftCommand : 0,
                          isArmed ? rightCommand : 0);
    if (length > 0 && length < (int)sizeof(tx) && Serial.availableForWrite() >= length) {
      Serial.write((uint8_t *)tx, length);
      lastStatus = now;
    }
  }
}
