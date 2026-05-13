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
#define J2_ADDRESS 0b01
#define J3_ADDRESS 0b00
#define J4_ADDRESS 0b11
#define J5_ADDRESS 0b00
#define J6_ADDRESS 0b10

#define R_SENSE 0.11f // Match to your driver

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2

// ================= EXTERN VARIABLES =================

extern FastAccelStepperEngine engine;
extern FastAccelStepper *stepper;

extern TMC2209Stepper J1_driver;
extern TMC2209Stepper J2_driver;
extern TMC2209Stepper J3_driver;
extern TMC2209Stepper J4_driver;
extern TMC2209Stepper J5_driver;
extern TMC2209Stepper J6_driver;

// ================= Start =================

void initJoints(bool debug, bool start_J1, bool start_J2, bool start_J3, bool start_J4, bool start_J5, bool start_J6);
