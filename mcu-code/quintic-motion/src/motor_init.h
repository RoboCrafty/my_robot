#pragma once

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"

// Hardware ports are usually kept as macros or references 
// depending on your board (e.g., ESP32 uses HardwareSerial)
#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2

namespace Constants {

    namespace Pins {
        // ================= JOINT DEFINITIONS =================
        // Joint 1 
        constexpr uint8_t J1_STEP_PIN = 33;
        constexpr uint8_t J1_DIR_PIN = 32;

        // Joint 2 
        constexpr uint8_t J2_STEP_PIN = 13;
        constexpr uint8_t J2_DIR_PIN = 12;

        // Joint 3 
        constexpr uint8_t J3_STEP_PIN = 19;
        constexpr uint8_t J3_DIR_PIN = 21;

        // Joint 4 
        constexpr uint8_t J4_STEP_PIN = 14;
        constexpr uint8_t J4_DIR_PIN = 27;

        // Joint 5 
        constexpr uint8_t J5_STEP_PIN = 22;
        constexpr uint8_t J5_DIR_PIN = 23;

        // Joint 6 
        constexpr uint8_t J6_STEP_PIN = 26;
        constexpr uint8_t J6_DIR_PIN = 25;
    }

    namespace Driver {
        // ================= UART Driver Addresses =================
        constexpr uint8_t J1_ADDRESS = 0b01;
        constexpr uint8_t J2_ADDRESS = 0b00;
        constexpr uint8_t J3_ADDRESS = 0b00;
        constexpr uint8_t J4_ADDRESS = 0b11;
        constexpr uint8_t J5_ADDRESS = 0b10;
        constexpr uint8_t J6_ADDRESS = 0b10;

        constexpr float R_SENSE_BTT = 0.11f;     // Match to your driver
        constexpr float R_SENSE_Random = 0.10f;  // Match to your driver
    }

    namespace Config {
        // =============== Constants and Configurations ===============
        constexpr uint16_t J1_CURRENT = 1800;
        constexpr uint16_t J2_CURRENT = 1800;
        constexpr uint16_t J3_CURRENT = 1600;
        constexpr uint16_t J4_CURRENT = 1200;
        constexpr uint16_t J5_CURRENT = 1100;
        constexpr uint16_t J6_CURRENT = 965;

        constexpr float J1_HOLD_MULTIPLIER = 0.5f;
        constexpr float J2_HOLD_MULTIPLIER = 0.8f;
        constexpr float J3_HOLD_MULTIPLIER = 0.5f;
        constexpr float J4_HOLD_MULTIPLIER = 0.5f;
        constexpr float J5_HOLD_MULTIPLIER = 0.5f;
        constexpr float J6_HOLD_MULTIPLIER = 0.5f;

        constexpr uint32_t J1_TPWMTHRS = 100*0;
        constexpr uint32_t J2_TPWMTHRS = 100*0;
        constexpr uint32_t J3_TPWMTHRS = 100*0;
        constexpr uint32_t J4_TPWMTHRS = 50*0;
        constexpr uint32_t J5_TPWMTHRS = 50*0;
        constexpr uint32_t J6_TPWMTHRS = 50*1;

        constexpr uint16_t J1_MICOSTEPS = 8;
        constexpr uint16_t J2_MICOSTEPS = 8;
        constexpr uint16_t J3_MICOSTEPS = 8;
        constexpr uint16_t J4_MICOSTEPS = 8;
        constexpr uint16_t J5_MICOSTEPS = 8;
        constexpr uint16_t J6_MICOSTEPS = 8;

        constexpr float J1_GEAR_RATIO = 6.4f;
        constexpr float J2_GEAR_RATIO = 20.0f;
        constexpr float J3_GEAR_RATIO = 18.0952381f;
        constexpr float J4_GEAR_RATIO = 4.0f;
        constexpr float J5_GEAR_RATIO = 4.0f;
        constexpr float J6_GEAR_RATIO = 10.0f;

        // Kinematic Calculations 
        constexpr float J1_STEP_PER_REV = (200.0f * J1_MICOSTEPS * J1_GEAR_RATIO);
        constexpr float J2_STEP_PER_REV = (200.0f * J2_MICOSTEPS * J2_GEAR_RATIO);
        constexpr float J3_STEP_PER_REV = (200.0f * J3_MICOSTEPS * J3_GEAR_RATIO);
        constexpr float J4_STEP_PER_REV = (200.0f * J4_MICOSTEPS * J4_GEAR_RATIO);
        constexpr float J5_STEP_PER_REV = (200.0f * J5_MICOSTEPS * J5_GEAR_RATIO);
        constexpr float J6_STEP_PER_REV = (200.0f * J6_MICOSTEPS * J6_GEAR_RATIO);

        constexpr float J1_STEPS_PER_DEG = (J1_STEP_PER_REV / 360.0f);
        constexpr float J2_STEPS_PER_DEG = (J2_STEP_PER_REV / 360.0f);
        constexpr float J3_STEPS_PER_DEG = (J3_STEP_PER_REV / 360.0f);
        constexpr float J4_STEPS_PER_DEG = (J4_STEP_PER_REV / 360.0f);
        constexpr float J5_STEPS_PER_DEG = (J5_STEP_PER_REV / 360.0f);
        constexpr float J6_STEPS_PER_DEG = (J6_STEP_PER_REV / 360.0f);
    }

} 
    // Global variables
    // ================= EXTERN VARIABLES =================
    extern FastAccelStepperEngine engine;
    extern FastAccelStepper *steppers[6];

    extern TMC2209Stepper J1_driver;
    extern TMC2209Stepper J2_driver;
    extern TMC2209Stepper J3_driver;
    extern TMC2209Stepper J4_driver;
    extern TMC2209Stepper J5_driver;
    extern TMC2209Stepper J6_driver;

    // ================= FUNCTIONS =================
    void initJoints(bool debug, bool start_J1, bool start_J2, bool start_J3, bool start_J4, bool start_J5, bool start_J6);