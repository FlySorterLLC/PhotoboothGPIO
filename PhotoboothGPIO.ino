#include <Servo.h>
#include <EEPROM.h>

/*******************************
 * Revision hisory
 * 
 * 1.3 Added motor, switches, and valve switches on turntable board.
 * 
 *******************************/

int UPPER_CAM_FG = 9;
//int UPPER_CAM_BG = 10;
int LOWER_CAM_FG = 11;
//int LOWER_CAM_BG = 12;

int STEPPER_ENABLE = A0;
int STEPPER_STEP = A1;
int STEPPER_DIR = A2;

int PUMP_ENABLE = 7;

int SERVO1 = 5;
int SERVO2 = 6;

int VALVE1_ENABLE = A3;
int VALVE2_ENABLE = 12;
int VALVE3_ENABLE = 10;

int SWITCH0 = A4;
int SWITCH1 = A5;
int SWITCH2 = 8;
int SWITCH3 = 2;

int MOTOR_IN1 = 13;
int MOTOR_IN2 = 4;
int MOTOR_PWM = 3;

Servo pbServo1, pbServo2;

byte FRONT_GATE_OPEN=90, FRONT_GATE_CLOSE=90, BACK_GATE_OPEN=90, BACK_GATE_CLOSE=90;

#define STEP_TIME 1000
#define STEPS_PER_VANE  267 // = 8 microsteps per step * 60 deg / (1.8 degrees per step)

#define MOTOR_SPEED 255 // out of 255
#define MAX_MOTOR_TIME 2000 // Milliseconds before motor moves time out

#define FGO_ADDRESS  0x02
#define FGC_ADDRESS  0x03
#define BGO_ADDRESS  0x04
#define BGC_ADDRESS  0x05

void setup() {

  Serial.begin(9600); //Open Serial connection for data logging & communication

  pinMode(UPPER_CAM_FG, OUTPUT);
  //pinMode(UPPER_CAM_BG, OUTPUT);
  pinMode(LOWER_CAM_FG, OUTPUT);
  //pinMode(LOWER_CAM_BG, OUTPUT);

  digitalWrite(UPPER_CAM_FG, LOW);
  //digitalWrite(UPPER_CAM_BG, LOW);
  digitalWrite(LOWER_CAM_FG, LOW);
  //digitalWrite(LOWER_CAM_BG, LOW);

  pinMode(STEPPER_ENABLE, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  
  digitalWrite(STEPPER_ENABLE, HIGH);
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_DIR, LOW);
  
  pinMode(PUMP_ENABLE, OUTPUT);
  
  digitalWrite(PUMP_ENABLE, LOW);

  pbServo1.attach(SERVO1);
  pbServo2.attach(SERVO2);

  pinMode(VALVE1_ENABLE, OUTPUT);
  pinMode(VALVE2_ENABLE, OUTPUT);
  pinMode(VALVE3_ENABLE, OUTPUT);

  digitalWrite(VALVE1_ENABLE, LOW);
  digitalWrite(VALVE2_ENABLE, LOW);
  digitalWrite(VALVE3_ENABLE, LOW);

  pinMode(SWITCH0, INPUT_PULLUP);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(SWITCH3, INPUT_PULLUP);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_PWM, 0);
  
  // Should do some validation here to make sure the values
  // are between 0-180

  FRONT_GATE_OPEN = EEPROM.read(FGO_ADDRESS);
  FRONT_GATE_CLOSE = EEPROM.read(FGC_ADDRESS);
  BACK_GATE_OPEN = EEPROM.read(BGO_ADDRESS);
  BACK_GATE_CLOSE = EEPROM.read(BGC_ADDRESS);
  
  pbServo1.write(90);
  pbServo2.write(90);

}

/* Commands:

A - all LED outputs enabled
O - all LED outputs disabled
U - enable upper camera foreground LEDs
u - enable upper camera background LEDs
L - enable lower camera foreground LEDs
l - enable lower camera background LEDs

S - step one vane forward
s - disable stepper

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
void loop() {
  
  static int dir = 0;
  static int pos = -1;
  
  while ( Serial.available() > 0 ) {
    char serialCmd = Serial.read();
    if ( serialCmd == 'A') {
      digitalWrite(UPPER_CAM_FG, HIGH);
      //digitalWrite(UPPER_CAM_BG, HIGH);
      digitalWrite(LOWER_CAM_FG, HIGH);
      //digitalWrite(LOWER_CAM_BG, HIGH);
      Serial.println("A");
    }
    else if ( serialCmd == 'O') {
      digitalWrite(UPPER_CAM_FG, LOW);
      //digitalWrite(UPPER_CAM_BG, LOW);
      digitalWrite(LOWER_CAM_FG, LOW);
      //digitalWrite(LOWER_CAM_BG, LOW);
      Serial.println("O");
    }
    else if ( serialCmd == 'U') {
      digitalWrite(UPPER_CAM_FG, HIGH);
      Serial.println("U");
    }
    else if ( serialCmd == 'u') {
      //digitalWrite(UPPER_CAM_BG, HIGH);
      Serial.println("u");
    }
    else if ( serialCmd == 'L') {
      digitalWrite(LOWER_CAM_FG, HIGH);
      Serial.println("L");
    }
    else if ( serialCmd == 'l') {
      //digitalWrite(LOWER_CAM_BG, HIGH);
      Serial.println("l");
    }
    else if ( serialCmd == 'S' ) {
      dir = !dir;
      digitalWrite(STEPPER_DIR, dir); delay(10);
      digitalWrite(STEPPER_ENABLE, LOW); delay(10);
      for (int i=0; i < STEPS_PER_VANE; i++ ) {
        digitalWrite(STEPPER_STEP, HIGH); delayMicroseconds(STEP_TIME); digitalWrite(STEPPER_STEP, LOW); delayMicroseconds(STEP_TIME); 
      }
      Serial.println("S");
    }
    else if ( serialCmd == 's' ) {
      digitalWrite(STEPPER_ENABLE, HIGH);
      Serial.println("s");
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
      int homed = 0;
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_PWM, MOTOR_SPEED);
      unsigned long startTime = millis();
      while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
        if ( digitalRead(SWITCH0) == HIGH ) {
          homed=1;
          break;
        }
      }
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, HIGH);
      analogWrite(MOTOR_PWM, 0);
      if ( homed == 1 ) {
        pos = 0;
        Serial.println("H");
      } else {
        Serial.println("Failed to home motor");
      }
    }
    else if ( serialCmd == '1' ) {
      int there = 0;
      // Drive motor to slot #1
      if ( pos < 1 ) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
      } else {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
      }
      analogWrite(MOTOR_PWM, MOTOR_SPEED);
      unsigned long startTime = millis();
      while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
        if ( digitalRead(SWITCH1) == HIGH ) {
          there=1;
          break;
        }
      }
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, HIGH);
      analogWrite(MOTOR_PWM, 0);
      if ( there == 1 ) {
        pos = 1;
        Serial.println("1");
      } else {
        Serial.println("Failed to position motor");
      }

    }
    else if ( serialCmd == '2' ) {
      int there = 0;
      // Drive motor to slot #2
      if ( pos < 2 ) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
      } else {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
      }
      analogWrite(MOTOR_PWM, MOTOR_SPEED);
      unsigned long startTime = millis();
      while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
        if ( digitalRead(SWITCH2) == HIGH ) {
          there=1;
          break;
        }
      }
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, HIGH);
      analogWrite(MOTOR_PWM, 0);
      if ( there == 1 ) {
        pos = 2;
        Serial.println("2");
      } else {
        Serial.println("Failed to position motor");
      }
    }
    else if ( serialCmd == '3' ) {
      int there = 0;
      // Drive motor to slot #3
      if ( pos < 3 ) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
      } else {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
      }
      analogWrite(MOTOR_PWM, MOTOR_SPEED);
      unsigned long startTime = millis();
      while ( ( millis() - startTime ) < MAX_MOTOR_TIME ) {
        if ( digitalRead(SWITCH3) == HIGH ) {
          there=1;
          break;
        }
      }
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, HIGH);
      analogWrite(MOTOR_PWM, 0);
      if ( there == 1 ) {
        pos = 3;
        Serial.println("3");
      } else {
        Serial.println("Failed to position motor");
      }
    }
    else if ( serialCmd == '0' ) {
      // turn off all solenoid valves
      digitalWrite(VALVE1_ENABLE, LOW);
      digitalWrite(VALVE2_ENABLE, LOW);
      digitalWrite(VALVE3_ENABLE, LOW);
      Serial.println("0");
    }
    else if ( serialCmd == 'q' ) {
      // turn on valve #1
      digitalWrite(VALVE1_ENABLE, HIGH);
      Serial.println("q");
    }
    else if ( serialCmd == 'w' ) {
      // turn on valve #2
      digitalWrite(VALVE2_ENABLE, HIGH);
      Serial.println("w");
    }
    else if ( serialCmd == 'e' ) {
      // turn on valve #3
      digitalWrite(VALVE3_ENABLE, HIGH);
      Serial.println("e");
    }
    else if ( serialCmd == 'V' ) {
      Serial.println("Arduino Relay - version 1.3");
    }
    else if ( serialCmd == 'I' ) {
      Serial.print("FGO: "); Serial.println(FRONT_GATE_OPEN);
      Serial.print("FGC: "); Serial.println(FRONT_GATE_CLOSE);
      Serial.print("BGO: "); Serial.println(BACK_GATE_OPEN);
      Serial.print("BGC: "); Serial.println(BACK_GATE_CLOSE);
      Serial.print("Switch 0: "); Serial.println(digitalRead(SWITCH0));
      Serial.print("Switch 1: "); Serial.println(digitalRead(SWITCH1));
      Serial.print("Switch 2: "); Serial.println(digitalRead(SWITCH2));
      Serial.print("Switch 3: "); Serial.println(digitalRead(SWITCH3));
    }
    else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


