#include <Servo.h>
#include <EEPROM.h>

int UPPER_CAM_FG = 9;
int UPPER_CAM_BG = 10;
int LOWER_CAM_FG = 11;
int LOWER_CAM_BG = 12;

int STEPPER_ENABLE = 1;
int STEPPER_STEP = 0;
int STEPPER_DIR = 4;

int PUMP_ENABLE = 7;

int SERVO1 = 5;
int SERVO2 = 6;

Servo pbServo1, pbServo2;

byte FRONT_GATE_OPEN=90, FRONT_GATE_CLOSE=90, BACK_GATE_OPEN=90, BACK_GATE_CLOSE=90;

#define STEP_TIME 1500
#define STEPS_PER_VANE  66

#define FGO_ADDRESS  0x02
#define FGC_ADDRESS  0x03
#define BGO_ADDRESS  0x04
#define BGC_ADDRESS  0x05

void setup() {

  Serial.begin(9600); //Open Serial connection for data logging & communication

  pinMode(UPPER_CAM_FG, OUTPUT);
  pinMode(UPPER_CAM_BG, OUTPUT);
  pinMode(LOWER_CAM_FG, OUTPUT);
  pinMode(LOWER_CAM_BG, OUTPUT);

  digitalWrite(UPPER_CAM_FG, LOW);
  digitalWrite(UPPER_CAM_BG, LOW);
  digitalWrite(LOWER_CAM_FG, LOW);
  digitalWrite(LOWER_CAM_BG, LOW);

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

G?? - load front gate set points (? are bytes that represent angles from 0-180, open set point first, then close)
g?? - load back gate set points (? are bytes that represent angles from 0-180, open set point first, then close)

V - version

*/
void loop() {
  
  static int dir = 0;
  
  while ( Serial.available() > 0 ) {
    char serialCmd = Serial.read();
    if ( serialCmd == 'A') {
      digitalWrite(UPPER_CAM_FG, HIGH);
      digitalWrite(UPPER_CAM_BG, HIGH);
      digitalWrite(LOWER_CAM_FG, HIGH);
      digitalWrite(LOWER_CAM_BG, HIGH);
      Serial.println("A");
    }
    else if ( serialCmd == 'O') {
      digitalWrite(UPPER_CAM_FG, LOW);
      digitalWrite(UPPER_CAM_BG, LOW);
      digitalWrite(LOWER_CAM_FG, LOW);
      digitalWrite(LOWER_CAM_BG, LOW);
      Serial.println("O");
    }
    else if ( serialCmd == 'U') {
      digitalWrite(UPPER_CAM_FG, HIGH);
      Serial.println("U");
    }
    else if ( serialCmd == 'u') {
      digitalWrite(UPPER_CAM_BG, HIGH);
      Serial.println("u");
    }
    else if ( serialCmd == 'L') {
      digitalWrite(LOWER_CAM_FG, HIGH);
      Serial.println("L");
    }
    else if ( serialCmd == 'l') {
      digitalWrite(LOWER_CAM_BG, HIGH);
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
    else if ( serialCmd == 'V' ) {
      Serial.println("Arduino Relay - version 1.2");
    }
    else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


