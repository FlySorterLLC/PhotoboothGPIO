#include "pins.h"

#define VERSION_STRING "Arduino Relay - version 1.7"

enum {
  // Motor speeds / PWM values (out of 255)
  VANE_MOTOR_SPEED = 255,
  GATE_MOTOR_SPEED = 150,
  SELECT_MOTOR_SPEED = 150,

  // Durations / timeouts
  GATE_MOTOR_TIMEOUT_MS = 1000,
  SELECT_MOTOR_TIMEOUT_MS = 1000,
  VANE_MOTOR_TIMEOUT_MS = 1000,
  VANE_MOTOR_REV_DELAY_MS = 200,

  // Counts
  SWITCH_DEBOUNCE_READS = 5,
};

enum Motor {
  MOTOR_SELECT = 0,
  MOTOR_VANE = 1,
  MOTOR_GATE = 2
};

enum MotorDirection {
  MOTOR_FWD = 0,
  MOTOR_REV = 1
};

enum VaneChoice {
  VANE_UPPER = 0,
  VANE_LOWER = 1
};

enum GatePosition {
  GATE_INLET = 0,
  GATE_OUTLET = 1,
  GATE_CLOSED = 2
};

enum Status {
  SUCCESS = 0,
  MOTOR_TIMEOUT = 1,
  INVALID_POSITION = 2,
};

void setupPins();
Status initialize();
void motorsOff();
void driveMotor(Motor m, MotorDirection d, int pwm);
Status driveMotorUntil(Motor m, MotorDirection d, int pwm, int switchPin, boolean desiredState, int t);

Status driveGate(GatePosition g);
Status driveVane(VaneChoice v);
Status homeSelect();
Status driveSelect(int desiredPosition, int currentPosition);

void printStatus(Status s);

