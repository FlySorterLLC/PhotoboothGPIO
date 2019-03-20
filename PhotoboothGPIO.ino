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


// Encoder library by Paul Stoffregen
// See: https://www.pjrc.com/teensy/td_libs_Encoder.html
// Use Teensy++ 2.0 pin numbers here
Encoder lvane_Encoder(3, 2); // D3, D2
Encoder uvane_Encoder(1, 0); // D1, D0


void setup() {

  Serial.begin(9600); //Open Serial connection for data logging & communication

  setupPins();
  lvane_Encoder.write(0);
  uvane_Encoder.write(0);

}

void loop() {
  
  static int pos = -1;
  static int homed = 0;
  
  while ( Serial.available() > 0 ) {
    char serialCmd = Serial.read();
    if ( serialCmd == 'V' ) {
      Serial.println(VERSION_STRING);
    }
    else if ( serialCmd == 'I' ) {
      Serial.print("Homed: "); Serial.println(homed);
      Serial.print("Current position: "); Serial.println(pos);
      Serial.print("Switch 0: "); Serial.println(digitalRead(SEL_SW_HOME));
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
    }
    else {
      Serial.print("Command '");
      Serial.print(serialCmd);
      Serial.println("' not understood.");
    }
  }
  
  delay(10);
  
}


