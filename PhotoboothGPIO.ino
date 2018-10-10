#include "Arduino.h"
#include "PBFunctions.h"
#include "pins.h"

/*******************************
 * Revision hisory
 * 
 * 1.7 Update to 2.1 PCB (microswitches for vanes, new motor drivers,
 *     new pin assignments, etc.)
 * 1.6 Fixed vane stepping code
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

H - home selector motor
1 - move selector to path/vial 1
2 - move selector to path/vial 2
3 - move selector to path/vial 3

0 - turn off all vacuum solenoid valves
q - turn on vacuum solenoid 1
w - turn on vacuum solenoid 2
e - turn on vacuum solenoid 3

V - version
I - info

*/



void setup() {

  Serial.begin(9600); //Open Serial connection for data logging & communication

  setupPins();

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
      Status s = driveVane(VANE_UPPER);
      if ( s == SUCCESS ) {
        Serial.println("Timeout advancing vanes.");
      } else {
        Serial.println("S");
      }
    }
    else if ( serialCmd == 's' ) {
      Status s = driveVane(VANE_LOWER);
      if ( s == SUCCESS ) {
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
      driveGate(GATE_INLET);
//      pbServo1.write( FRONT_GATE_OPEN );
      Serial.println("F");
    }
    else if ( serialCmd == 'f' ) {
      driveGate(GATE_CLOSED);
      Serial.println("f");
    }
    else if ( serialCmd == 'B' ) {
      driveGate(GATE_OUTLET);
      Serial.println("B");
    }
    else if ( serialCmd == 'b' ) {
      driveGate(GATE_CLOSED);
      Serial.println("b");
    }
    else if ( serialCmd == 'H' ) {
      // Home motor
      Status s = homeSelect();
      if ( s == SUCCESS ) {
        homed = 1;
        pos = 0;
        Serial.println("H");
      } else {
        Serial.println("Failed to home motor");
      }
    }
    else if ( serialCmd >= '1' && serialCmd <= '3' ) {
      if ( homed == 0 ) { Serial.println("Not homed."); continue; }
      Status s = driveSelect(serialCmd-'0', pos);
      if ( s == SUCCESS ) {
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
      Serial.println(VERSION_STRING);
    }
    else if ( serialCmd == 'I' ) {
      Serial.print("Homed: "); Serial.println(homed);
      Serial.print("Current position: "); Serial.println(pos);
      Serial.print("Switch 0: "); Serial.println(digitalRead(SEL_SW_HOME));
      Serial.print("Switch 1: "); Serial.println(digitalRead(SEL_SW_1));
      Serial.print("Switch 2: "); Serial.println(digitalRead(SEL_SW_2));
      Serial.print("Switch 3: "); Serial.println(digitalRead(SEL_SW_3));
      Serial.print("Vane switch 1 (upper): "); Serial.println(digitalRead(VANE_SW_1));
      Serial.print("Vane switch 2 (lower): "); Serial.println(digitalRead(VANE_SW_2));
      Serial.print("Gate switch (input): "); Serial.println(digitalRead(GATE_SW_INPUT));
      Serial.print("Gate switch (output): "); Serial.println(digitalRead(GATE_SW_OUTPUT));
      Serial.print("Gate switch (closed): "); Serial.println(digitalRead(GATE_SW_CLOSED));
    }
    else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


