#include "Arduino.h"
#include <Servo.h>
#include <EEPROM.h>
#include "pins.h"

/*******************************
 * Revision hisory
 * 
 * 1.5 Switch to AT90USB1286-based PCB (instead of Arduino Nano)
 * 1.4 Added subroutines to modularize and reuse code.
 * 1.3 Added motor, switches, and valve switches on turntable board.
 * 
 *******************************/

/* Commands:

A - all LED outputs enabled
O - all LED outputs disabled
U - enable upper camera foreground LEDs
u - enable upper camera background LEDs
L - enable lower camera foreground LEDs
l - enable lower camera background LEDs

S - position vanes for UPPER camera
s - position vanes for LOWER camera

P - turn on air pump
p - turn off air pump

F - open front gate
f - close front gate

B - open back gate
b - close back gate

G?? - load front gate set points (? are each bytes that represent angles from 0-180, open set point first, then close)
g?? - load back gate set points (? are each bytes that represent angles from 0-180, open set point first, then close)

H - home turntable motor
1 - move turntable to path/vial 1
2 - move turntable to path/vial 2
3 - move turntable to path/vial 3

0 - turn off all vacuum solenoid valves
q - turn on vacuum solenoid 1
w - turn on vacuum solenoid 2
e - turn on vacuum solenoid 3

V - version
I - info

*/

int switches[4] = { SEL_SW_HOME, SEL_SW_1, SEL_SW_2, SEL_SW_3 };

Servo pbServo1, pbServo2;

byte FRONT_GATE_OPEN=90, FRONT_GATE_CLOSE=90, BACK_GATE_OPEN=90, BACK_GATE_CLOSE=90;

#define VANE_MOTOR_SPEED_FAST 150 // out of 255
#define VANE_MOTOR_SPEED_SLOW 50 // out of 255

// for debouncing photogates (might need more than 1 consec. read, try it)
#define PHOTOGATE_READS 1

// arbitrary constants, should be enums
#define VANE_DIR_FORWARD  1  
#define VANE_DIR_REVERSE -1

#define SELECT_MOTOR_SPEED 255 // out of 255
#define MAX_MOTOR_TIME 2000 // Milliseconds before motor moves time out
#define DEBOUNCE_COUNT 5

#define FGO_ADDRESS  0x02
#define FGC_ADDRESS  0x03
#define BGO_ADDRESS  0x04
#define BGC_ADDRESS  0x05

void setup() {

  Serial.begin(9600); //Open Serial connection for data logging & communication

  pinMode(PHOTOGATE1, INPUT_PULLUP);
  pinMode(PHOTOGATE2, INPUT_PULLUP);

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

  pinMode(PAGER_ENABLE, OUTPUT);
  digitalWrite(PAGER_ENABLE, LOW);

  pbServo1.attach(INPUT_SERVO);
  pbServo2.attach(OUTPUT_SERVO);

  pinMode(SOL1_ENABLE, OUTPUT);
  pinMode(SOL2_ENABLE, OUTPUT);
  pinMode(SOL3_ENABLE, OUTPUT);
  digitalWrite(SOL1_ENABLE, LOW);
  digitalWrite(SOL2_ENABLE, LOW);
  digitalWrite(SOL3_ENABLE, LOW);

  pinMode(SEL_SW_HOME, INPUT_PULLUP);
  pinMode(SEL_SW_1, INPUT_PULLUP);
  pinMode(SEL_SW_2, INPUT_PULLUP);
  pinMode(SEL_SW_3, INPUT_PULLUP);

  pinMode(VANE_IN1, OUTPUT);
  pinMode(VANE_IN2, OUTPUT);
  pinMode(VANE_PWM, OUTPUT);
  digitalWrite(VANE_IN1, HIGH);
  digitalWrite(VANE_IN2, HIGH);
  analogWrite(VANE_PWM, 0);

  pinMode(SELECT_IN1, OUTPUT);
  pinMode(SELECT_IN2, OUTPUT);
  pinMode(SELECT_PWM, OUTPUT);
  digitalWrite(SELECT_IN1, HIGH);
  digitalWrite(SELECT_IN2, HIGH);
  analogWrite(SELECT_PWM, 0);

  // Should do some validation here to make sure the values
  // are between 0-180

  FRONT_GATE_OPEN = EEPROM.read(FGO_ADDRESS);
  FRONT_GATE_CLOSE = EEPROM.read(FGC_ADDRESS);
  BACK_GATE_OPEN = EEPROM.read(BGO_ADDRESS);
  BACK_GATE_CLOSE = EEPROM.read(BGC_ADDRESS);
  
  pbServo1.write(90);
  pbServo2.write(90);

}

void vaneMotorCtrl(int dir, int pwm) {

  if (pwm == 0) {
    digitalWrite(VANE_IN1, HIGH);
    digitalWrite(VANE_IN2, HIGH);   
  } else if (dir == VANE_DIR_FORWARD) {
    digitalWrite(VANE_IN1, LOW);
    digitalWrite(VANE_IN2, HIGH);
  } else {
    digitalWrite(VANE_IN1, HIGH);
    digitalWrite(VANE_IN2, LOW);
  }

  analogWrite(VANE_PWM, pwm);
  
}

// turns on vane motor and waits for state
// if timeout, turns off motor and returns 1
// if success, leaves motor on and returns 0
int vaneMotorTimeout(int dir, int pwm, int whichPhoto, int desiredState) {

  vaneMotorCtrl(dir, pwm);

  unsigned long startTime = millis();
  
  int debounce_counter = 0;

  while ( (millis() - startTime) < MAX_MOTOR_TIME ) {

    int photoState = digitalRead(whichPhoto);
    if (photoState == desiredState) {
      debounce_counter += 1;
      if (debounce_counter >= PHOTOGATE_READS) {
        return 0;
      }
    } else {
      debounce_counter = 0;
    }
    
  }

  vaneMotorCtrl(dir, 0);
  return 1;

}

int advanceVanes( int upDown ) {
  
  int whichPhoto = (upDown == HIGH ? PHOTOGATE1 : PHOTOGATE2);
  
  int initPhotoState = digitalRead(whichPhoto);
  
  // Step 1) fast forward until we see falling edge
  
  // if photo is not yet high, run until we are high
  if (initPhotoState == LOW) {
    if (vaneMotorTimeout(VANE_DIR_FORWARD, VANE_MOTOR_SPEED_FAST, whichPhoto, HIGH)) {
      return 1;
    }
  }

  // now photo is high, wait for it to get low
  if (vaneMotorTimeout(VANE_DIR_FORWARD, VANE_MOTOR_SPEED_FAST, whichPhoto, LOW)) {
    return 1;
  }

  // Step 2) slow back until we see high value.
  if (vaneMotorTimeout(VANE_DIR_REVERSE, VANE_MOTOR_SPEED_SLOW, whichPhoto, HIGH)) {
    return 1;
  }

  // Step 3) slow forward until we see low value.
  if (vaneMotorTimeout(VANE_DIR_FORWARD, VANE_MOTOR_SPEED_SLOW, whichPhoto, LOW)) {
    return 1;
  }

  // need to turn off motor because vaneMotorTimeout leaves it on
  vaneMotorCtrl(VANE_DIR_FORWARD, 0);
  return 0;
  
}

int homeSelectMotor() {
  int homed = 0;
  digitalWrite(SELECT_IN1, LOW);
  digitalWrite(SELECT_IN2, HIGH);
  analogWrite(SELECT_PWM, SELECT_MOTOR_SPEED);
  unsigned long startTime = millis();
  int switchCount = 0;
  while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
    if ( digitalRead(SEL_SW_HOME) == HIGH ) {
      switchCount++;
    }
    if ( switchCount == DEBOUNCE_COUNT ) {
      homed=1;
      break;
    }
  }
  digitalWrite(SELECT_IN1, HIGH);
  digitalWrite(SELECT_IN2, HIGH);
  analogWrite(SELECT_PWM, 0);
  return homed;

}

int gotoPosition(int desiredPosition, int currentPosition) {
  int there = 0, past = 0;
  int switchCount = 0;
  if (( desiredPosition < 1 ) || ( desiredPosition > 3)) {
    return 0;
  }
  if ( desiredPosition == currentPosition ) {
    return 1;
  } else if ( desiredPosition > currentPosition ) {
    digitalWrite(SELECT_IN1, LOW);
    digitalWrite(SELECT_IN2, HIGH);
    analogWrite(SELECT_PWM, SELECT_MOTOR_SPEED);
    unsigned long startTime = millis();
    while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
      if ( digitalRead(switches[desiredPosition]) == HIGH ) {
        switchCount++;
      }
      if ( switchCount == DEBOUNCE_COUNT ) {
        there=1;
        past=1;
        switchCount = 0;
        break;
      }
    }
    
    if ( there ) {
      delay(10);
      while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
        if ( digitalRead(switches[desiredPosition]) == LOW ) {
          switchCount++;
        }
        if ( switchCount == DEBOUNCE_COUNT ) {
          past=1;
          there=0;
          switchCount = 0;
          break;
        }
      }
    }
    if ( past ) {
      delay(10);
      digitalWrite(SELECT_IN1, HIGH);
      digitalWrite(SELECT_IN2, LOW);
      while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
        if ( digitalRead(switches[desiredPosition]) == HIGH ) {
          switchCount++;
        }
        if ( switchCount == DEBOUNCE_COUNT ) {
          there=1;
          switchCount = 0;
          break;
        }
      }      
    }
    digitalWrite(SELECT_IN1, HIGH);
    digitalWrite(SELECT_IN2, HIGH);
    analogWrite(SELECT_PWM, 0);
    return (there && past);
  
  } else if ( desiredPosition < currentPosition ) {
    digitalWrite(SELECT_IN1, HIGH);
    digitalWrite(SELECT_IN2, LOW);
    analogWrite(SELECT_PWM, SELECT_MOTOR_SPEED);
    unsigned long startTime = millis();
    while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
      if ( digitalRead(switches[desiredPosition]) == HIGH ) {
        there=1;
        break;
      }
    }
    digitalWrite(SELECT_IN1, HIGH);
    digitalWrite(SELECT_IN2, HIGH);
    analogWrite(SELECT_PWM, 0);
    return (there);
  }
  return 0;
}

void loop() {
  
  static int pos = -1;
  static int homed = 0;
  
  while ( Serial.available() > 0 ) {
    char serialCmd = Serial.read();
    if ( serialCmd == 'A') {
      digitalWrite(UPPER_ENABLE1, LOW);
      digitalWrite(UPPER_ENABLE2, LOW);
      digitalWrite(LOWER_ENABLE1, LOW);
      digitalWrite(LOWER_ENABLE2, LOW);
      Serial.println("A");
    }
    else if ( serialCmd == 'O') {
      digitalWrite(UPPER_ENABLE1, HIGH);
      digitalWrite(UPPER_ENABLE2, HIGH);
      digitalWrite(LOWER_ENABLE1, HIGH);
      digitalWrite(LOWER_ENABLE2, HIGH);
      Serial.println("O");
    }
    else if ( serialCmd == 'U') {
      digitalWrite(UPPER_ENABLE1, LOW);
      Serial.println("U");
    }
    else if ( serialCmd == 'u') {
      digitalWrite(UPPER_ENABLE2, LOW);
      Serial.println("u");
    }
    else if ( serialCmd == 'L') {
      digitalWrite(LOWER_ENABLE1, LOW);
      Serial.println("L");
    }
    else if ( serialCmd == 'l') {
      digitalWrite(LOWER_ENABLE2, LOW);
      Serial.println("l");
    }
    else if ( serialCmd == 'S' ) {
      int r = advanceVanes(0);
      if ( r ) {
        Serial.println("Timeout advancing vanes.");
      } else {
        Serial.println("S");
      }
    }
    else if ( serialCmd == 's' ) {
      int r = advanceVanes(1);
      if ( r ) {
        Serial.println("Timeout advancing vanes.");
      } else {
        Serial.println("s");
      }
    }
    else if ( serialCmd == 'P' ) {
      digitalWrite(PUMP_ENABLE, HIGH);
      Serial.println("P");
    }
    else if ( serialCmd == 'p' ) {
      digitalWrite(PUMP_ENABLE, LOW);
      Serial.println("p");
    }
    else if ( serialCmd == 'F' ) {
      pbServo1.write( FRONT_GATE_OPEN );
      Serial.println("F");
    }
    else if ( serialCmd == 'f' ) {
      pbServo1.write(FRONT_GATE_CLOSE);
      Serial.println("f");
    }
    else if ( serialCmd == 'B' ) {
      pbServo2.write(BACK_GATE_OPEN);
      Serial.println("B");
    }
    else if ( serialCmd == 'b' ) {
      pbServo2.write(BACK_GATE_CLOSE);
      Serial.println("b");
    }
    else if ( serialCmd == 'G' ) {
      delay(100); // Sleep for a bit to wait for serial data.
      if ( Serial.available() != 2 ) {
        Serial.println("Error: expected two bytes to follow 'G' command.");
      } else {
        // Should do some validation here to make sure the values
        // are between 0-180
        FRONT_GATE_OPEN = Serial.read();
        FRONT_GATE_CLOSE = Serial.read();
        EEPROM.write(FGO_ADDRESS, FRONT_GATE_OPEN);
        EEPROM.write(FGC_ADDRESS, FRONT_GATE_CLOSE);
        Serial.println("G");
      }
    }
    else if ( serialCmd == 'g' ) {
      delay(100); // Sleep for a bit to wait for serial data.
      if ( Serial.available() != 2 ) {
        Serial.println("Error: expected two bytes to follow 'g' command.");
      } else {
        // Should do some validation here to make sure the values
        // are between 0-180
        BACK_GATE_OPEN = Serial.read();
        BACK_GATE_CLOSE = Serial.read();
        EEPROM.write(BGO_ADDRESS, BACK_GATE_OPEN);
        EEPROM.write(BGC_ADDRESS, BACK_GATE_CLOSE);
        Serial.println("g");
      }
    }
    else if ( serialCmd == 'H' ) {
      // Home motor
      homed = homeSelectMotor();
      if ( homed == 1 ) {
        pos = 4;
        Serial.println("H");
      } else {
        Serial.println("Failed to home motor");
      }
    }
    else if ( serialCmd >= '1' && serialCmd <= '3' ) {
      if ( homed == 0 ) { Serial.println("Not homed."); continue; }
      int r = gotoPosition(serialCmd-'0', pos);
      if ( r == 1 ) {
        pos = serialCmd-'0';
        Serial.println(serialCmd);
      } else {
        Serial.println("Failed to position motor");
      }
    }
    else if ( serialCmd == '0' ) {
      // turn off all valves
      digitalWrite(SOL1_ENABLE, LOW);
      digitalWrite(SOL2_ENABLE, LOW);
      digitalWrite(SOL3_ENABLE, LOW);
      Serial.println("0");
    }
    else if ( serialCmd == 'q' ) {
      // turn on valve #1
      digitalWrite(SOL1_ENABLE, HIGH);
      Serial.println("q");
    }
    else if ( serialCmd == 'w' ) {
      // turn on valve #2
      digitalWrite(SOL2_ENABLE, HIGH);
      Serial.println("w");
    }
    else if ( serialCmd == 'e' ) {
      // turn on valve #3
      digitalWrite(SOL3_ENABLE, HIGH);
      Serial.println("e");
    }
    else if ( serialCmd == 'V' ) {
      Serial.println("Arduino Relay - version 1.5");
    }
    else if ( serialCmd == 'I' ) {
      Serial.print("FGO: "); Serial.println(FRONT_GATE_OPEN);
      Serial.print("FGC: "); Serial.println(FRONT_GATE_CLOSE);
      Serial.print("BGO: "); Serial.println(BACK_GATE_OPEN);
      Serial.print("BGC: "); Serial.println(BACK_GATE_CLOSE);
      Serial.print("Homed: "); Serial.println(homed);
      Serial.print("Current position: "); Serial.println(pos);
      Serial.print("Switch 0: "); Serial.println(digitalRead(SEL_SW_HOME));
      Serial.print("Switch 1: "); Serial.println(digitalRead(SEL_SW_1));
      Serial.print("Switch 2: "); Serial.println(digitalRead(SEL_SW_2));
      Serial.print("Switch 3: "); Serial.println(digitalRead(SEL_SW_3));
      Serial.print("Vane photogate 1: "); Serial.println(digitalRead(PHOTOGATE1));
      Serial.print("Vane photogate 2: "); Serial.println(digitalRead(PHOTOGATE2));
    }
    else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


