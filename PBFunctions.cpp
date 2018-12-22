#include "arduino.h"
#include "pins.h"
#include "PBFunctions.h"

int switches[4] = { SEL_SW_HOME, SEL_SW_1, SEL_SW_2, SEL_SW_3 };

void setupPins() {
  pinMode(VANE_SW_1, INPUT);
  pinMode(VANE_SW_2, INPUT);

  pinMode(LOWER_ENABLE1, OUTPUT);
  pinMode(LOWER_ENABLE2, OUTPUT);
  pinMode(UPPER_ENABLE1, OUTPUT);
  pinMode(UPPER_ENABLE2, OUTPUT);

  digitalWrite(LOWER_ENABLE1, HIGH);
  digitalWrite(LOWER_ENABLE2, HIGH);
  digitalWrite(UPPER_ENABLE1, HIGH);
  digitalWrite(UPPER_ENABLE2, HIGH);

  pinMode(PUMP_ENABLE, OUTPUT);
  digitalWrite(PUMP_ENABLE, LOW);

  pinMode(SOL1_ENABLE, OUTPUT);
  pinMode(SOL2_ENABLE, OUTPUT);
  pinMode(SOL3_ENABLE, OUTPUT);
  digitalWrite(SOL1_ENABLE, LOW);
  digitalWrite(SOL2_ENABLE, LOW);
  digitalWrite(SOL3_ENABLE, LOW);

  pinMode(SEL_SW_HOME, INPUT);
  pinMode(SEL_SW_1, INPUT);
  pinMode(SEL_SW_2, INPUT);
  pinMode(SEL_SW_3, INPUT);

  pinMode(VANE_FWD, OUTPUT);
  pinMode(VANE_REV, OUTPUT);
  pinMode(VANE_PWM, OUTPUT);

  pinMode(SELECT_FWD, OUTPUT);
  pinMode(SELECT_REV, OUTPUT);
  pinMode(SELECT_PWM, OUTPUT);

  pinMode(GATE_FWD, OUTPUT);
  pinMode(GATE_REV, OUTPUT);
  pinMode(GATE_PWM, OUTPUT);

  motorsOff();

}

void motorsOff() {
  driveMotor(MOTOR_SELECT, MOTOR_FWD, 0);
  driveMotor(MOTOR_VANE, MOTOR_FWD, 0);
  driveMotor(MOTOR_GATE, MOTOR_FWD, 0);

}

void driveMotor(Motor m, MotorDirection d, int pwm) {

  int fwdPin, revPin, pwmPin;

  switch ( m ) {
    case MOTOR_SELECT:
      fwdPin = SELECT_FWD;
      revPin = SELECT_REV;
      pwmPin = SELECT_PWM;
      break;
    case MOTOR_VANE:
      fwdPin = VANE_FWD;
      revPin = VANE_REV;
      pwmPin = VANE_PWM;
      break;
    case MOTOR_GATE:
      fwdPin = GATE_FWD;
      revPin = GATE_REV;
      pwmPin = GATE_PWM;
      break;
    default:
      return;
  }
  
  if (pwm == 0) {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, HIGH);   
  } else if (d == MOTOR_FWD) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);
  } else {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);
  }

  analogWrite(pwmPin, pwm);
  
}

Status driveMotorUntil(Motor m, MotorDirection d, int pwm, int switchPin, boolean desiredState, int t, int r) {
  unsigned long int startTime = millis();
  int switchReads = 0;
  boolean timeOut = false;
  
  driveMotor(m, d, pwm);
  while ( switchReads < r ) {
    delayMicroseconds(100);
    if ( digitalRead(switchPin) == desiredState ) { switchReads++;  }
    if ( millis() - startTime > t ) { timeOut = true; break; }
  }
  driveMotor(m, d, 0);
  if ( timeOut ) { return MOTOR_TIMEOUT; }

  return SUCCESS;
}

Status driveVane(VaneChoice v) {

  int switchPin = (v == VANE_UPPER) ? VANE_SW_1 : VANE_SW_2;

  Status s = driveMotorUntil(MOTOR_VANE, MOTOR_FWD, VANE_MOTOR_SPEED, switchPin, LOW, VANE_MOTOR_TIMEOUT_MS, 1);
  if ( s != SUCCESS ) { return s; }

  delay(25);
  return driveMotorUntil(MOTOR_VANE, MOTOR_REV, VANE_MOTOR_SPEED/2, switchPin, LOW, VANE_MOTOR_TIMEOUT_MS, 1);

}

Status homeSelect() {
  return driveMotorUntil(MOTOR_SELECT, MOTOR_FWD, SELECT_MOTOR_SPEED, SEL_SW_HOME, LOW, SELECT_MOTOR_TIMEOUT_MS);
}

Status driveSelect(int desiredPosition, int currentPosition) {
  Status s;
  if (( desiredPosition < 1 ) || ( desiredPosition > 3)) {
    return INVALID_POSITION;
  }
  if ( desiredPosition == currentPosition ) {
    return SUCCESS;
  } else if ( desiredPosition > currentPosition ) {
    
    s = driveMotorUntil(MOTOR_SELECT, MOTOR_REV, SELECT_MOTOR_SPEED, switches[desiredPosition], LOW, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    return SUCCESS;
  
  } else if ( desiredPosition < currentPosition ) {

    s = driveMotorUntil(MOTOR_SELECT, MOTOR_FWD, SELECT_MOTOR_SPEED, switches[desiredPosition], LOW, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    s = driveMotorUntil(MOTOR_SELECT, MOTOR_FWD, SELECT_MOTOR_SPEED, switches[desiredPosition], HIGH, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    s = driveMotorUntil(MOTOR_SELECT, MOTOR_REV, SELECT_MOTOR_SPEED, switches[desiredPosition], LOW, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    return (SUCCESS);
  }
  return INVALID_POSITION;
}

Status homeGates() {
  Status s = driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_INPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return s; }

  s = driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_CLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return s; }
  
}

Status driveGate(GatePosition g) {
  if ( g == GATE_INLET ) {
    return driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_INPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  } else if ( g == GATE_OUTLET ) {
    return driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_OUTPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  } else if ( g == GATE_CLOSED ) {
    if ( digitalRead(GATE_SW_INPUT) == LOW ) {
      return driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_CLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
    } else if ( digitalRead(GATE_SW_OUTPUT) == LOW ) {
      return driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_CLOSED, LOW, GATE_MOTOR_TIMEOUT_MS);
    } else {
      return homeGates();
    }
  }
}



