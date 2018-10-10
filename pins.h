#ifndef __PINS_H__
#define __PINS_H__

// These two photogates sense the rotation of the background vanes
#define VANE_SW_1         PIN_C0
#define VANE_SW_2         PIN_C1

// These pins control the background vane motor
#define VANE_FWD          PIN_C3
#define VANE_REV          PIN_C2
#define VANE_PWM          PIN_C5

// These pins control the selector motor
#define SELECT_FWD        PIN_E1
#define SELECT_REV        PIN_E0
#define SELECT_PWM        PIN_C4

// These pins control the gate motor
#define GATE_FWD          PIN_F7
#define GATE_REV          PIN_A0
#define GATE_PWM          PIN_C6

// Selector module location switches
#define SEL_SW_HOME       PIN_D4
#define SEL_SW_1          PIN_D5
#define SEL_SW_2          PIN_D6
#define SEL_SW_3          PIN_D7

// Selector solenoid enable lines
#define SOL1_ENABLE       PIN_D1
#define SOL2_ENABLE       PIN_D2
#define SOL3_ENABLE       PIN_D3

// Enable line for the pump
#define PUMP_ENABLE       PIN_E5

// Gate switches
#define GATE_SW_INPUT     PIN_F4
#define GATE_SW_OUTPUT    PIN_F5
#define GATE_SW_CLOSED    PIN_F6

// Enable lines for illumination
#define LOWER_ENABLE1     PIN_F2
#define LOWER_ENABLE2     PIN_F3
#define UPPER_ENABLE1     PIN_F1
#define UPPER_ENABLE2     PIN_F0

#endif
