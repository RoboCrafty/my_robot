#pragma once
#include <Arduino.h>
#include "motor_init.h"

namespace Constants{
    namespace Pins
    {
        // ================= LIMIT SWITCHES =================
        const uint8_t L1_PIN = 5;  // Base     (J1)      -- Normally low
        const uint8_t L2_PIN = 36; // Shoulder (J2)      -- Normally high
        const uint8_t L3_PIN = 39; // Elbow    (J3)      -- Normally high
        const uint8_t L4_PIN = 18; // Wrist1   (J4)      -- Normally low
        const uint8_t L5_PIN = 34; // Wrist2   (J5)      -- Normally high
        const uint8_t L6_PIN = 35; // Wrist3   (J6)      -- Normally low
    }
    namespace Config
    {
        constexpr bool J1_HOMING_DIR = 0;
        constexpr bool J2_HOMING_DIR = 0;
        constexpr bool J3_HOMING_DIR = 1;
        constexpr bool J4_HOMING_DIR = 0;
        constexpr bool J5_HOMING_DIR = 1;
        constexpr bool J6_HOMING_DIR = 1;

        constexpr uint16_t J1_HOMING_SPEED = 2000;
        constexpr uint16_t J2_HOMING_SPEED = 3000;
        constexpr uint16_t J3_HOMING_SPEED = 5000;
        constexpr uint16_t J4_HOMING_SPEED = 5000;
        constexpr uint16_t J5_HOMING_SPEED = 1000;
        constexpr uint16_t J6_HOMING_SPEED = 10000;

        // Homed position in degrees after homing sequence completes !! Positions are zerod after hominhg sequence, so these are relative to the homing switch trigger point and not final absolute positions !!
        constexpr int16_t J1_HOMED_POSITION = 25 * Constants::Config::J1_STEPS_PER_DEG; 
        constexpr int16_t J2_HOMED_POSITION = 48 * Constants::Config::J2_STEPS_PER_DEG;
        constexpr int16_t J3_HOMED_POSITION = -71 * Constants::Config::J3_STEPS_PER_DEG;
        constexpr int16_t J4_HOMED_POSITION = 143 * Constants::Config::J4_STEPS_PER_DEG;
        constexpr int16_t J5_HOMED_POSITION = -125 * Constants::Config::J5_STEPS_PER_DEG;
        constexpr int16_t J6_HOMED_POSITION = 190 * Constants::Config::J6_STEPS_PER_DEG;
    }
    

}




inline bool initLimitSwitches()
{
    pinMode(Constants::Pins::L1_PIN, INPUT_PULLUP);
    pinMode(Constants::Pins::L2_PIN, INPUT_PULLUP);
    pinMode(Constants::Pins::L3_PIN, INPUT_PULLUP);
    pinMode(Constants::Pins::L4_PIN, INPUT_PULLUP);
    pinMode(Constants::Pins::L5_PIN, INPUT_PULLUP); 
    pinMode(Constants::Pins::L6_PIN, INPUT_PULLUP);

    return true; 
}

inline bool isLimitSwitchTriggered(int joint)
{
    switch (joint) {
        case 1: return digitalRead(Constants::Pins::L1_PIN) == HIGH; 
        case 2: return digitalRead(Constants::Pins::L2_PIN) == LOW; 
        case 3: return digitalRead(Constants::Pins::L3_PIN) == LOW; 
        case 4: return digitalRead(Constants::Pins::L4_PIN) == HIGH; 
        case 5: return digitalRead(Constants::Pins::L5_PIN) == LOW;
        case 6: return digitalRead(Constants::Pins::L6_PIN) == HIGH; 
        default: return false;
    }
}

inline void homeAxis(FastAccelStepper* stepper, uint8_t joint, bool homing_direction, uint16_t homing_speed, int16_t homed_position, float steps_per_deg)
{
    uint16_t slowhomingSpeed = 500;
    
    stepper->setSpeedInHz(homing_speed);
    if(homing_direction){
        stepper->runForward();
    }
    else
    {
        stepper->runBackward();
    }


    while (isLimitSwitchTriggered(joint) == false)
    {
        
    }
    stepper->forceStopAndNewPosition(0);
    
    stepper->setSpeedInHz(homing_speed);
    if(homing_direction){
        stepper->move(steps_per_deg * -10);
    }
    else
    {
        stepper->move(steps_per_deg * 10);
    }

    while (stepper->isRunning())
    {

    }

    stepper->setSpeedInHz(slowhomingSpeed);
    if(homing_direction){
        stepper->runForward();
    }
    else
    {
        stepper->runBackward();
    }

    while (isLimitSwitchTriggered(joint) == false)
    {
        
    }

    stepper->forceStopAndNewPosition(0);
    stepper->setSpeedInHz(homing_speed);
     // Short delay to ensure the stepper has fully stopped before moving to the final position
    stepper->moveTo(homed_position);
    // delay(50);
    // stepper->setPositionAfterCommandsCompleted(0); TODO:: Doesnt seem to be working
   

    
     Serial.printf("Joint %d homed! Moving to home position \n", joint);
}

inline void homeAxis(uint8_t joint)
{
    switch (joint) {
        case 1: homeAxis(steppers[0], 1, Constants::Config::J1_HOMING_DIR, Constants::Config::J1_HOMING_SPEED, Constants::Config::J1_HOMED_POSITION, Constants::Config::J1_STEPS_PER_DEG); break;
        case 2: homeAxis(steppers[1], 2, Constants::Config::J2_HOMING_DIR, Constants::Config::J2_HOMING_SPEED, Constants::Config::J2_HOMED_POSITION, Constants::Config::J2_STEPS_PER_DEG); break;
        case 3: homeAxis(steppers[2], 3, Constants::Config::J3_HOMING_DIR, Constants::Config::J3_HOMING_SPEED, Constants::Config::J3_HOMED_POSITION, Constants::Config::J3_STEPS_PER_DEG); break;
        case 4: homeAxis(steppers[3], 4, Constants::Config::J4_HOMING_DIR, Constants::Config::J4_HOMING_SPEED, Constants::Config::J4_HOMED_POSITION, Constants::Config::J4_STEPS_PER_DEG); break;
        case 5: homeAxis(steppers[4], 5, Constants::Config::J5_HOMING_DIR, Constants::Config::J5_HOMING_SPEED, Constants::Config::J5_HOMED_POSITION, Constants::Config::J5_STEPS_PER_DEG); break;
        case 6: homeAxis(steppers[5], 6, Constants::Config::J6_HOMING_DIR, Constants::Config::J6_HOMING_SPEED, Constants::Config::J6_HOMED_POSITION, Constants::Config::J6_STEPS_PER_DEG); break;
        default: Serial.println("Invalid joint number for homing."); break;
    }
}