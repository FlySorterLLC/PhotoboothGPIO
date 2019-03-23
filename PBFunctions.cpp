#include "arduino.h"
#include "pins.h"
#include "PBFunctions.h"
#include "Encoder.h"

// Encoder library by Paul Stoffregen
// See: https://www.pjrc.com/teensy/td_libs_Encoder.html
// Use Teensy++ 2.0 pin numbers here
Encoder lvane_Encoder(LVANE_A_NUM, LVANE_B_NUM);
Encoder uvane_Encoder(UVANE_A_NUM, UVANE_B_NUM);

int switches[4] = { SEL_SW_HOME, SEL_SW_1, SEL_SW_2, SEL_SW_3 };

void setupPins() {

  pinMode(LOWER_LED1_EN, OUTPUT);
  pinMode(LOWER_LED2_EN, OUTPUT);
  pinMode(LOWER_LED3_EN, OUTPUT);
  pinMode(UPPER_LED1_EN, OUTPUT);
  pinMode(UPPER_LED2_EN, OUTPUT);
  pinMode(UPPER_LED3_EN, OUTPUT);
  
  digitalWrite(LOWER_LED1_EN, LOW);
  digitalWrite(LOWER_LED2_EN, LOW);
  digitalWrite(LOWER_LED3_EN, LOW);
  digitalWrite(UPPER_LED1_EN, LOW);
  digitalWrite(UPPER_LED2_EN, LOW);
  digitalWrite(UPPER_LED3_EN, LOW);

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

  pinMode(GATE_SW_INPUT, INPUT);
  pinMode(GATE_SW_OUTPUT, INPUT);
  pinMode(GATE_SW_CLOSED, INPUT);

  pinMode(UVANE_FWD, OUTPUT);
  pinMode(UVANE_REV, OUTPUT);
  pinMode(UVANE_PWM, OUTPUT);

  pinMode(LVANE_FWD, OUTPUT);
  pinMode(LVANE_REV, OUTPUT);
  pinMode(LVANE_PWM, OUTPUT);

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
  driveMotor(MOTOR_LVANE, MOTOR_FWD, 0);
  driveMotor(MOTOR_UVANE, MOTOR_FWD, 0);
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
    case MOTOR_LVANE:
      fwdPin = LVANE_FWD;
      revPin = LVANE_REV;
      pwmPin = LVANE_PWM;
      break;
    case MOTOR_UVANE:
      fwdPin = UVANE_FWD;
      revPin = UVANE_REV;
      pwmPin = UVANE_PWM;
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

Status homeSelect() {
  return driveMotorUntil(MOTOR_SELECT, MOTOR_REV, SELECT_MOTOR_SPEED, SEL_SW_HOME, LOW, SELECT_MOTOR_TIMEOUT_MS);
}

Status driveSelect(int desiredPosition, int currentPosition) {
  Status s;
  if (( desiredPosition < 1 ) || ( desiredPosition > 3)) {
    return INVALID_POSITION;
  }
  if ( desiredPosition == currentPosition ) {
    return SUCCESS;
  } else if ( desiredPosition > currentPosition ) {
    
    s = driveMotorUntil(MOTOR_SELECT, MOTOR_FWD, SELECT_MOTOR_SPEED, switches[desiredPosition], LOW, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    return SUCCESS;
  
  } else if ( desiredPosition < currentPosition ) {

    s = driveMotorUntil(MOTOR_SELECT, MOTOR_REV, SELECT_MOTOR_SPEED, switches[desiredPosition], LOW, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    s = driveMotorUntil(MOTOR_SELECT, MOTOR_REV, SELECT_MOTOR_SPEED, switches[desiredPosition], HIGH, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    s = driveMotorUntil(MOTOR_SELECT, MOTOR_FWD, SELECT_MOTOR_SPEED, switches[desiredPosition], LOW, SELECT_MOTOR_TIMEOUT_MS);
    if ( s != SUCCESS ) { return s; }

    return (SUCCESS);
  }
  return INVALID_POSITION;
}

Status homeGates() {
  Status s = driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_INPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return s; }

  s = driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_CLOSED, HIGH, GATE_MOTOR_TIMEOUT_MS);
  if ( s != SUCCESS ) { return s; }
  
}

Status driveGate(GatePosition g) {
  if ( g == GATE_INLET ) {
    return driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_INPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  } else if ( g == GATE_OUTLET ) {
    return driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_OUTPUT, LOW, GATE_MOTOR_TIMEOUT_MS);
  } else if ( g == GATE_CLOSED ) {
    if ( digitalRead(GATE_SW_INPUT) == LOW ) {
      return driveMotorUntil(MOTOR_GATE, MOTOR_FWD, GATE_MOTOR_SPEED, GATE_SW_CLOSED, HIGH, GATE_MOTOR_TIMEOUT_MS);
    } else if ( digitalRead(GATE_SW_OUTPUT) == LOW ) {
      return driveMotorUntil(MOTOR_GATE, MOTOR_REV, GATE_MOTOR_SPEED, GATE_SW_CLOSED, HIGH, GATE_MOTOR_TIMEOUT_MS);
    } else {
      return homeGates();
    }
  }
}

Status homeVane(VaneChoice v) {
  Motor m;
  int p;
  Encoder *enc;
  
  if ( v == VANE_UPPER ) {
    m = MOTOR_UVANE; 
    p = UVANE_N;
    enc = &uvane_Encoder;
  } else {
    m = MOTOR_LVANE;
    p = LVANE_N;
    enc = &lvane_Encoder;
  }
  if ( driveMotorUntil(m, MOTOR_FWD, VANE_MOTOR_SPEED, p, LOW, VANE_MOTOR_TIMEOUT_MS) != SUCCESS ) {
    return MOTOR_TIMEOUT;
  }
  delay(VANE_MOTOR_REV_DELAY_MS);

  if ( driveMotorUntil(m, MOTOR_REV, VANE_MOTOR_SPEED/4, p, LOW, VANE_MOTOR_TIMEOUT_MS) != SUCCESS ) {
    return MOTOR_TIMEOUT;
  }

  enc->write(0);
}


Status driveVane(VaneChoice v, VanePosition p) {
  /* Motors are 297.92:1 gear ratio, and the encoders are 12 counts per revolution,
   * so there are a total of 3575 counts per revolution.
   * This is conveniently close to (degrees of rotation)*10. 
   * 
   * The vane disc has 6 locations:
   *    A. filter #1 (VANE_1)
   *    B. background in between 1 & 2 (VANE_1P)
   *    C. filter #2 (VANE_2)
   *    D. background in between 2 & 3 (VANE_2P)
   *    E. filter #3 (VANE_3)
   *    F. background in between 3 & 1 (VANE_3P)
   * 
   * The zero position (where the slot is) is actually position F (VANE_3P)
   * 
   * Running the motors MOTOR_FWD actually causes the encoder count to go down.
   * 
   */
  int lastPosition, currentPosition, targetPosition, error;
  int output;
  double kp = 2.0, ki = 0.00001, kd = 20.0;
  double pTerm, iTerm=0, dTerm;
  unsigned long startTime = millis();
  Encoder *enc;
  Motor m;
  boolean timeout = false;

  if ( v == VANE_UPPER ) {
    enc = &uvane_Encoder;
    m = MOTOR_UVANE;
  } else {
    enc = &lvane_Encoder;
    m = MOTOR_LVANE;
  }

  if ( p == VANE_1 ) {
    targetPosition = 596*5;
  } else if ( p == VANE_1P ) {
    targetPosition = 596*4;
  } else if ( p == VANE_2 ) {
    targetPosition = 596*3;
  } else if ( p == VANE_2P ) {
    targetPosition = 596*2;
  } else if ( p == VANE_3 ) {
    targetPosition = 596;
  } else if ( p == VANE_3P ) {
    targetPosition = 0;
  }

  currentPosition = lastPosition = enc->read();
  error = targetPosition - currentPosition;
  
  while ( ( abs(error) > ENCODER_DEADBAND_SIZE ) && ( !timeout ) ) {
    delay(2);
    currentPosition = enc->read();
    error = targetPosition - currentPosition;
    pTerm = ( error * kp );
    iTerm += ( error * ki );
    if ( iTerm > 255 ) { iTerm = 255; }
    if ( iTerm < -255 ) { iTerm = -255; }
    dTerm = ( currentPosition - lastPosition ) * kd;
    output = int ( pTerm + iTerm - dTerm );
    if (output > 255 ) { output = 255; }
    if (output < -255 ) { output = -255; }
    if (output > 0 ) {
      driveMotor(m, MOTOR_REV, output);
    } else {
      driveMotor(m, MOTOR_FWD, -output);
    }
    lastPosition = currentPosition;
    if ( ( millis() - startTime ) > VANE_MOTOR_TIMEOUT_MS ) {
      timeout = true;
    }
  }

  driveMotor(m, MOTOR_FWD, 0);

  if ( timeout ) {
    return MOTOR_TIMEOUT;
  }

  return SUCCESS;
}

