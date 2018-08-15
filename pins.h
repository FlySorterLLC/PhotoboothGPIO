#ifndef __PINS_H__
#define __PINS_H__

// These two photogates sense the rotation of the background vanes
#define PHOTOGATE1        PIN_E6
#define PHOTOGATE2        PIN_E7

// These pins control the background vane motor
#define VANE_IN1          PIN_B7
#define VANE_IN2          PIN_B6
#define VANE_PWM          PIN_B4

// These pins control the selector motor
#define SELECT_IN1        PIN_E4
#define SELECT_IN2        PIN_E5
#define SELECT_PWM        PIN_B5

// Selector module location switches
#define SEL_SW_HOME       PIN_D6
#define SEL_SW_1          PIN_D7
#define SEL_SW_2          PIN_C0
#define SEL_SW_3          PIN_C1

// Selector solenoid enable lines
#define SOL1_ENABLE       PIN_D3
#define SOL2_ENABLE       PIN_D4
#define SOL3_ENABLE       PIN_D5

// Enable line for the pager motor
#define PAGER_ENABLE      PIN_F0

// Enable line for the pump
#define PUMP_ENABLE       PIN_A7

// Gate servo pins
#define INPUT_SERVO       PIN_C4
#define OUTPUT_SERVO      PIN_C5

// Enable lines for illumination
#define LOWER_ENABLE1     PIN_F1
#define LOWER_ENABLE2     PIN_B0
#define UPPER_ENABLE1     PIN_A6
#define UPPER_ENABLE2     PIN_A5

// Input pins for the photogates on the selector module
#define VIAL1PHOTO        PIN_D0
#define VIAL2PHOTO        PIN_D1
#define VIAL3PHOTO        PIN_D2

// These come from the datasheet for the processor (AT90USB1286)
// E.g. Pin PD2 is also labeled INT2
#define VIAL1PHOTO_INT    0
#define VIAL2PHOTO_INT    1
#define VIAL3PHOTO_INT    2

#endif
