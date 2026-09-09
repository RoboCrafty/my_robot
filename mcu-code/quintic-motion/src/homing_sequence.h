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
        constexpr uint16_t J2_HOMING_SPEED = 5000;
        constexpr uint16_t J3_HOMING_SPEED = 5000;
        constexpr uint16_t J4_HOMING_SPEED = 5000;
        constexpr uint16_t J5_HOMING_SPEED = 1000;
        constexpr uint16_t J6_HOMING_SPEED = 10000;

        // Homed position in degrees after homing sequence completes !! Positions are zerod after hominhg sequence, so these are relative to the homing switch trigger point and not final absolute positions !!
        constexpr int16_t J1_HOMED_POSITION = 30 * Constants::Config::J1_STEPS_PER_DEG; // Example: -25 degrees from the switch
        constexpr int16_t J2_HOMED_POSITION = 52 * Constants::Config::J2_STEPS_PER_DEG;
        constexpr int16_t J3_HOMED_POSITION = -71 * Constants::Config::J3_STEPS_PER_DEG;
        constexpr int16_t J4_HOMED_POSITION = 143 * Constants::Config::J4_STEPS_PER_DEG;
        constexpr int16_t J5_HOMED_POSITION = -123 * Constants::Config::J5_STEPS_PER_DEG;
        constexpr int16_t J6_HOMED_POSITION = (285-360) * Constants::Config::J6_STEPS_PER_DEG;
        // J6_HOMED_POSITION is only where J5's limit switch flag lines up -- not
        // J6's real resting angle. Once J5 is done with it, J6 makes one more
        // move of this size (from that alignment point) to reach true home.
        constexpr int16_t J6_POST_J5_OFFSET = (85-180) * Constants::Config::J6_STEPS_PER_DEG;
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

// Restores the normal moveTimed operating speed/accel that setup() configures.
// homeAxis() runs its approach at a per-joint homing speed and never restores
// it, so without this every axis stays capped at its last homing speed for
// every subsequent move -- the likely cause of motion feeling rough/uneven
// after a rehome.
inline void restoreNormalMotion(uint8_t joint)
{
    FastAccelStepper* s = steppers[joint - 1];
    s->setAcceleration(6000);
    s->setSpeedInHz(6000);
    s->applySpeedAcceleration();
}

// homeAxis() returns once its final moveTo() is QUEUED, not once the axis has
// arrived -- so anything sequencing on it must wait explicitly. The old fixed
// delay(3000) was a guess, and J5's 123-degree return alone runs at 1000 Hz,
// which takes far longer than that; truncating it with forceStopAndNewPosition
// then declares the WRONG physical angle to be zero.
inline void waitUntilIdle(uint8_t joint)
{
    while (steppers[joint - 1]->isRunning()) {}
}
inline void waitAllIdle()
{
    for (int i = 0; i < 6; i++) while (steppers[i]->isRunning()) {}
}

// J6 only needs to SIT at J6_HOMED_POSITION while J5's switch is being found;
// its real home is one more move away from there. Shared by the full sequence
// and the per-joint path so the two can't drift apart (see J6_POST_J5_OFFSET).
inline void finishJ6Home()
{
    steppers[5]->setCurrentPosition(0);
    steppers[5]->moveTo(Constants::Config::J6_POST_J5_OFFSET);
    waitUntilIdle(6);
}

// The full six-axis sequence, shared by boot and the runtime rehome command so
// the two cannot drift apart. Leaves every axis stopped at its homed angle;
// the caller decides how to re-zero the counters.
inline void homeAllAxes()
{
    homeAxis(1);
    homeAxis(2);
    homeAxis(3);
    homeAxis(4);
    homeAxis(6);
    waitUntilIdle(6);   // J5's switch is only reachable once J6 has ARRIVED, not merely started moving
    homeAxis(5);
    waitUntilIdle(5);
    finishJ6Home();
    waitAllIdle();      // J1-J4's return moves overlap all of the wrist work above
    for (int i = 1; i <= 6; i++) restoreNormalMotion(i);
}

// Homes ONE joint for the web UI's per-joint "home Jx" command. J5's limit
// switch can only be reached with J6 parked at its alignment position (the arm
// hardware, not software, requires this), so a solo J5 home always re-homes
// J6 first -- the same order the full sequence uses. Homing J6 alone still
// needs its own extra move to reach true home (see finishJ6Home).
inline void homeJointWithDependency(uint8_t joint)
{
    if (joint == 5) {
        homeAxis(6);
        waitUntilIdle(6);
        homeAxis(5);
        waitUntilIdle(5);
        finishJ6Home();
        steppers[5]->forceStopAndNewPosition(0);
        steppers[4]->forceStopAndNewPosition(0);
        restoreNormalMotion(6);
        restoreNormalMotion(5);
        return;
    }
    homeAxis(joint);
    waitUntilIdle(joint);
    if (joint == 6) finishJ6Home();
    steppers[joint - 1]->forceStopAndNewPosition(0);
    restoreNormalMotion(joint);
}