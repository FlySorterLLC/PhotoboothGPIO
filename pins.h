#ifndef __PINS_H__
#define __PINS_H__

// These pins control the lower vane motor
#define LVANE_FWD          PIN_E0
#define LVANE_REV          PIN_D7
#define LVANE_PWM          PIN_C4

// These pins control the upper vane motor
#define UVANE_FWD          PIN_E6
#define UVANE_REV          PIN_E7
#define UVANE_PWM          PIN_B5

// These pins control the selector motor
#define SELECT_FWD         PIN_F3
#define SELECT_REV         PIN_F4
#define SELECT_PWM         PIN_C6

// These pins control the gate motor
#define GATE_FWD          PIN_D5
#define GATE_REV          PIN_D4
#define GATE_PWM          PIN_B6

// Lower vane position inputs
#define LVANE_A           PIN_D3 // Teensy++ 2.0 pin 3
#define LVANE_B           PIN_D2 // Teensy++ 2.0 pin 2
#define LVANE_N           PIN_D6

// Upper vane position inputs
#define UVANE_A           PIN_D1 // Teensy++ 2.0 pin 1
#define UVANE_B           PIN_D0 // Teensy++ 2.0 pin 0
#define UVANE_N           PIN_B4

// Selector module location switches
#define SEL_SW_HOME       PIN_F7
#define SEL_SW_1          PIN_A0
#define SEL_SW_2          PIN_A1
#define SEL_SW_3          PIN_A2

// Gate switches
#define GATE_SW_INPUT     PIN_B7
#define GATE_SW_OUTPUT    PIN_E4
#define GATE_SW_CLOSED    PIN_E5

// LED control pins (3 upper, 3 lower)
#define LOWER_LED1_EN      PIN_C1
#define LOWER_LED2_EN      PIN_C0
#define LOWER_LED3_EN      PIN_E1
#define UPPER_LED1_EN      PIN_F0
#define UPPER_LED2_EN      PIN_F1
#define UPPER_LED3_EN      PIN_F2

// Selector solenoid enable lines
#define SOL1_ENABLE       PIN_C7
#define SOL2_ENABLE       PIN_C5
#define SOL3_ENABLE       PIN_C3

// Enable line for the pump
#define PUMP_ENABLE       PIN_C2


#endif
