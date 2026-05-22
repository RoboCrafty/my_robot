#pragma once

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"



// ================= JOINT DEFINITIONS =================
// Joint 1 
#define J1_STEP_PIN 33
#define J1_DIR_PIN  32

// Joint 2 
#define J2_STEP_PIN 13
#define J2_DIR_PIN  12

// Joint 3 
#define J3_STEP_PIN 19
#define J3_DIR_PIN  21

// Joint 4 
#define J4_STEP_PIN 14
#define J4_DIR_PIN  27

// Joint 5 
#define J5_STEP_PIN 22
#define J5_DIR_PIN  23

// Joint 6 
#define J6_STEP_PIN 26
#define J6_DIR_PIN  25


// ================= UART Driver Addresses =================

#define J1_ADDRESS 0b01
#define J2_ADDRESS 0b00
#define J3_ADDRESS 0b00
#define J4_ADDRESS 0b11
#define J5_ADDRESS 0b10
#define J6_ADDRESS 0b10

#define R_SENSE_BTT 0.11f // Match to your driver
#define R_SENSE_Random 0.10f // Match to your driver

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2

// ================= EXTERN VARIABLES =================

extern FastAccelStepperEngine engine;

extern FastAccelStepper *stepper1;
extern FastAccelStepper *stepper2;
extern FastAccelStepper *stepper3;
extern FastAccelStepper *stepper4;
extern FastAccelStepper *stepper5;
extern FastAccelStepper *stepper6;

extern TMC2209Stepper J1_driver;
extern TMC2209Stepper J2_driver;
extern TMC2209Stepper J3_driver;
extern TMC2209Stepper J4_driver;
extern TMC2209Stepper J5_driver;
extern TMC2209Stepper J6_driver;

// =============== Constants and Macros ===============
#define J1_CURRENT 1800
#define J2_CURRENT 1800
#define J3_CURRENT 1600
#define J4_CURRENT 1300
#define J5_CURRENT 1300
#define J6_CURRENT 965

#define J1_HOLD_MULTIPLIER 0.7
#define J2_HOLD_MULTIPLIER 0.7
#define J3_HOLD_MULTIPLIER 0.7
#define J4_HOLD_MULTIPLIER 0.7
#define J5_HOLD_MULTIPLIER 0.7
#define J6_HOLD_MULTIPLIER 0.7

#define J1_TPWMTHRS 100
#define J2_TPWMTHRS 100
#define J3_TPWMTHRS 100
#define J4_TPWMTHRS 50
#define J5_TPWMTHRS 50
#define J6_TPWMTHRS 50

#define J1_MICOSTEPS 8
#define J2_MICOSTEPS 8
#define J3_MICOSTEPS 8
#define J4_MICOSTEPS 8
#define J5_MICOSTEPS 8
#define J6_MICOSTEPS 8

#define J1_GEAR_RATIO 6.4
#define J2_GEAR_RATIO 20.0
#define J3_GEAR_RATIO 18.0952381
#define J4_GEAR_RATIO 4.0
#define J5_GEAR_RATIO 4.0
#define J6_GEAR_RATIO 10.0

#define J1_STEP_PER_REV (200.0 * J1_MICOSTEPS * J1_GEAR_RATIO)
#define J2_STEP_PER_REV (200.0 * J2_MICOSTEPS * J2_GEAR_RATIO)
#define J3_STEP_PER_REV (200.0 * J3_MICOSTEPS * J3_GEAR_RATIO)
#define J4_STEP_PER_REV (200.0 * J4_MICOSTEPS * J4_GEAR_RATIO)
#define J5_STEP_PER_REV (200.0 * J5_MICOSTEPS * J5_GEAR_RATIO)
#define J6_STEP_PER_REV (200.0 * J6_MICOSTEPS * J6_GEAR_RATIO)

// #define J1_DEG_PER_STEP (360.0 / J1_STEP_PER_REV)
// #define J2_DEG_PER_STEP (360.0 / J2_DEG_PER_STEP)
// #define J3_DEG_PER_STEP (360.0 / J3_STEP_PER_REV)
// #define J4_DEG_PER_STEP (360.0 / J4_STEP_PER_REV)
// #define J5_DEG_PER_STEP (360.0 / J5_STEP_PER_REV)
// #define J6_DEG_PER_STEP (360.0 / J6_STEP_PER_REV)

#define J1_STEPS_PER_DEG (J1_STEP_PER_REV / 360.0)
#define J2_STEPS_PER_DEG (J2_STEP_PER_REV / 360.0)
#define J3_STEPS_PER_DEG (J3_STEP_PER_REV / 360.0)
#define J4_STEPS_PER_DEG (J4_STEP_PER_REV / 360.0)
#define J5_STEPS_PER_DEG (J5_STEP_PER_REV / 360.0)
#define J6_STEPS_PER_DEG (J6_STEP_PER_REV / 360.0)



// ================= Start =================

void initJoints(bool debug, bool start_J1, bool start_J2, bool start_J3, bool start_J4, bool start_J5, bool start_J6);
