#include "Arduino.h"
#include "PBFunctions.h"
#include "pins.h"
#include "Encoder.h"

/*******************************
 * Revision hisory
 * 
 * 8.0 New PCBs for v8 design
 * 1.7 Update to 2.1 PCB (microswitches for vanes, new motor drivers,
 *     new pin assignments, etc.)
 * 1.6 Fixed vane stepping code
 * 1.5 Switch to AT90USB1286-based PCB (instead of Arduino Nano)
 * 1.4 Added subroutines to modularize and reuse code.
 * 1.3 Added motor, switches, and valve switches on turntable board.
 * 
 *******************************/

/* Commands:

V - Version string ("Fly Photobooth - Version 8.X")

I - Debug info (input list / status)

H - Move selector to home position
1 - Move selector to position 1
2 - Move selector to position 2
3 - Move selector to position 3

0 - Turn off all solenoids
q - Turn on solenoid 1
w - Turn on solenoid 2
e - Turn on solenoid 3

P - Turn on pump
p - Turn off pump

f - Open front gate (closes back)
b - Open back gate (closes front)
n - Close both gates

R or r - reset (home) upper or lower filter wheel

a, s, or d - Move upper filter wheel to position 1 (open), 2 (GFP), or 3 (RFP)
A, S, or D - Move upper filter wheel to position 1+ (white BG), 2+ (reserved), or 3+ (reserved)

z, x, or c - Move lower filter wheel to position 1 (open), 2 (GFP), or 3 (RFP)
Z, X, or C - Move lower filter wheel to position 1+ (white BG), 2+ (reserved), or 3+ (reserved)

j, k, or l - Enable upper LED set 1 (white), 2 (GFP), or 3 (RFP)
u, i, or o - Enable lower LED set 1 (white), 2 (GFP), or 3 (RFP)

Y or y - Disable all upper or lower LEDs

*/

extern Encoder lvane_Encoder, uvane_Encoder;

void setup() {

  Serial.begin(9600); //Open Serial connection for data logging & communication

  setupPins();
  
  homeVane(VANE_UPPER);
  homeVane(VANE_LOWER);
  homeSelect();
  homeGates();

}

void loop() {
  
  static int pos = -1;
  
  while ( Serial.available() > 0 ) {
    char serialCmd = Serial.read();
    if ( serialCmd == 'V' ) {
      Serial.println(VERSION_STRING);
    }
    else if ( serialCmd == 'I' ) {
      Serial.print("Current position: "); Serial.println(pos);
      Serial.print("Switch H: "); Serial.println(digitalRead(SEL_SW_HOME));
      Serial.print("Switch 1: "); Serial.println(digitalRead(SEL_SW_1));
      Serial.print("Switch 2: "); Serial.println(digitalRead(SEL_SW_2));
      Serial.print("Switch 3: "); Serial.println(digitalRead(SEL_SW_3));
      Serial.print("Lower vane position: "); Serial.println(lvane_Encoder.read());
      Serial.print("Lower vane index: "); Serial.println(digitalRead(LVANE_N));
      Serial.print("Upper vane position: "); Serial.println(uvane_Encoder.read());
      Serial.print("Upper vane index: "); Serial.println(digitalRead(UVANE_N));
      Serial.print("Gate switch (input): "); Serial.println(digitalRead(GATE_SW_INPUT));
      Serial.print("Gate switch (output): "); Serial.println(digitalRead(GATE_SW_OUTPUT));
      Serial.print("Gate switch (closed): "); Serial.println(digitalRead(GATE_SW_CLOSED));
    } else if ( serialCmd == '0' ) {
      digitalWrite(SOL1_ENABLE, LOW);
      digitalWrite(SOL2_ENABLE, LOW);
      digitalWrite(SOL3_ENABLE, LOW);
      Serial.println("0");
    } else if ( serialCmd == 'q' ) {
      digitalWrite(SOL1_ENABLE, HIGH);
      Serial.println("q");
    } else if ( serialCmd == 'w' ) {
      digitalWrite(SOL2_ENABLE, HIGH);
      Serial.println("w");
    } else if ( serialCmd == 'e' ) {
      digitalWrite(SOL3_ENABLE, HIGH);
      Serial.println("e");
    } else if ( serialCmd == 'Y' ) {
      digitalWrite(UPPER_LED1_EN, LOW);
      digitalWrite(UPPER_LED2_EN, LOW);
      digitalWrite(UPPER_LED3_EN, LOW);
      Serial.println("Y");
    } else if ( serialCmd == 'y' ) {
      digitalWrite(LOWER_LED1_EN, LOW);
      digitalWrite(LOWER_LED2_EN, LOW);
      digitalWrite(LOWER_LED3_EN, LOW);
      Serial.println("y");
    } else if ( serialCmd == 'j' ) {
      digitalWrite(UPPER_LED1_EN, HIGH);
      Serial.println("j");
    } else if ( serialCmd == 'k' ) {
      digitalWrite(UPPER_LED2_EN, HIGH);
      Serial.println("k");
    } else if ( serialCmd == 'l' ) {
      digitalWrite(UPPER_LED3_EN, HIGH);
      Serial.println("l");
    } else if ( serialCmd == 'u' ) {
      digitalWrite(LOWER_LED1_EN, HIGH);
      Serial.println("u");
    } else if ( serialCmd == 'i' ) {
      digitalWrite(LOWER_LED2_EN, HIGH);
      Serial.println("i");
    } else if ( serialCmd == 'o' ) {
      digitalWrite(LOWER_LED3_EN, HIGH);
      Serial.println("o");
    } else if ( serialCmd == 'P' ) {
      digitalWrite(PUMP_ENABLE, HIGH);
      Serial.println("P");
    } else if ( serialCmd == 'p' ) {
      digitalWrite(PUMP_ENABLE, LOW);
      Serial.println("p");
    } else if ( serialCmd == 'H' ) {
      if ( homeSelect() == SUCCESS ) {
        pos = 0;
        Serial.println("H");
      } else {
        Serial.println("F");
      }
    } else if (( serialCmd == '1' ) || ( serialCmd == '2' ) || ( serialCmd == '3' )) { 
      if ( pos == -1 ) {
        if ( homeSelect() == SUCCESS ) {
          pos = 0;
        } else {
          Serial.println("F");
          break;
        }
      }
      if ( driveSelect( serialCmd-'1'+1, pos) == SUCCESS ) {
        pos = serialCmd-'1'+1;
        Serial.println(serialCmd);
      } else {
        pos = -1;
        Serial.println("F");
      }
    } else if ( serialCmd == 'R' ) {
      if ( homeVane(VANE_UPPER) == SUCCESS ) {
        Serial.println("R");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'r' ) {
      if ( homeVane(VANE_LOWER) == SUCCESS ) {
        Serial.println("r");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'a' ) {
      if ( driveVane(VANE_UPPER, VANE_1) == SUCCESS ) {
        Serial.println("a");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 's' ) {
      if ( driveVane(VANE_UPPER, VANE_2) == SUCCESS ) {
        Serial.println("s");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'd' ) {
      if ( driveVane(VANE_UPPER, VANE_3) == SUCCESS ) {
        Serial.println("d");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'A' ) {
      if ( driveVane(VANE_UPPER, VANE_1P) == SUCCESS ) {
        Serial.println("A");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'S' ) {
      if ( driveVane(VANE_UPPER, VANE_2P) == SUCCESS ) {
        Serial.println("S");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'D' ) {
      if ( driveVane(VANE_UPPER, VANE_3P) == SUCCESS ) {
        Serial.println("D");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'z' ) {
      if ( driveVane(VANE_LOWER, VANE_1) == SUCCESS ) {
        Serial.println("z");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'x' ) {
      if ( driveVane(VANE_LOWER, VANE_2) == SUCCESS ) {
        Serial.println("x");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'c' ) {
      if ( driveVane(VANE_LOWER, VANE_3) == SUCCESS ) {
        Serial.println("c");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'Z' ) {
      if ( driveVane(VANE_LOWER, VANE_1P) == SUCCESS ) {
        Serial.println("Z");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'X' ) {
      if ( driveVane(VANE_LOWER, VANE_2P) == SUCCESS ) {
        Serial.println("X");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'C' ) {
      if ( driveVane(VANE_LOWER, VANE_3P) == SUCCESS ) {
        Serial.println("C");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'f' ) {
      if ( driveGate(GATE_INLET) == SUCCESS ) {
        Serial.println("f");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'b' ) {
      if ( driveGate(GATE_OUTLET) == SUCCESS ) {
        Serial.println("b");
      } else {
        Serial.println("F");
      }
    } else if ( serialCmd == 'n' ) {
      if ( driveGate(GATE_CLOSED) == SUCCESS ) {
        Serial.println("n");
      } else {
        Serial.println("F");
      }
    } else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


