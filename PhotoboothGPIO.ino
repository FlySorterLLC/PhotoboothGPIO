int UPPER_CAM_FG = 2;
int UPPER_CAM_BG = 3;
int LOWER_CAM_FG = 4;
int LOWER_CAM_BG = 5;

int STEPPER_ENABLE = 6;
int STEPPER_STEP = 7;
int STEPPER_DIR = 8;

int PUMP_ENABLE = 9;

int LED = 13;

#define STEP_TIME 1500
#define STEPS_PER_VANE  66

void setup() {
  
  pinMode(LED, OUTPUT);

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

V - version

*/
void loop() {
  
  static int dir = 0;
  
  while ( Serial.available() > 0 ) {
    digitalWrite(LED, HIGH); delay(100); digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED, HIGH); delay(100); digitalWrite(LED,LOW); delay(100);
    String serialCmd = Serial.readStringUntil('\n');
    if ( serialCmd == "A") {
      digitalWrite(UPPER_CAM_FG, HIGH);
      digitalWrite(UPPER_CAM_BG, HIGH);
      digitalWrite(LOWER_CAM_FG, HIGH);
      digitalWrite(LOWER_CAM_BG, HIGH);
      Serial.println("A");
    }
    else if ( serialCmd == "O") {
      digitalWrite(UPPER_CAM_FG, LOW);
      digitalWrite(UPPER_CAM_BG, LOW);
      digitalWrite(LOWER_CAM_FG, LOW);
      digitalWrite(LOWER_CAM_BG, LOW);
      Serial.println("O");
    }
    else if ( serialCmd == "U") {
      digitalWrite(UPPER_CAM_FG, HIGH);
      Serial.println("U");
    }
    else if ( serialCmd == "u") {
      digitalWrite(UPPER_CAM_BG, HIGH);
      Serial.println("u");
    }
    else if ( serialCmd == "L") {
      digitalWrite(LOWER_CAM_FG, HIGH);
      Serial.println("L");
    }
    else if ( serialCmd == "l") {
      digitalWrite(LOWER_CAM_BG, HIGH);
      Serial.println("l");
    }
    else if ( serialCmd == "S" ) {
      dir = !dir;
      digitalWrite(STEPPER_DIR, dir); delay(10);
      digitalWrite(STEPPER_ENABLE, LOW); delay(10);
      for (int i=0; i < STEPS_PER_VANE; i++ ) {
        digitalWrite(STEPPER_STEP, HIGH); delayMicroseconds(STEP_TIME); digitalWrite(STEPPER_STEP, LOW); delayMicroseconds(STEP_TIME); 
      }
      Serial.println("S");
    }
    else if ( serialCmd == "s" ) {
      digitalWrite(STEPPER_ENABLE, HIGH);
      Serial.println("s");
    }
    else if ( serialCmd == "P" ) {
      digitalWrite(PUMP_ENABLE, HIGH);
      Serial.println("P");
    }
    else if ( serialCmd == "p" ) {
      digitalWrite(PUMP_ENABLE, LOW);
      Serial.println("p");
    }
    else if ( serialCmd == "V" ) {
      Serial.println("Arduino Relay - version 1.0");
      digitalWrite(LED, HIGH); delay(1000); digitalWrite(LED,LOW);
    }
    else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


