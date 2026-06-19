#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <motor_init.h>
#include <math.h>
#include <ArduinoEigen.h>
#include <homing_sequence.h>
#include <kinematics.h>
#include <BasicLinearAlgebra.h>
#include <helper_functions.h>
#include <structs.h>
// #include <motion_planner.h>
#include <dds_stepgen.h>
#include <ruckig/ruckig.hpp>

// Create the DDS Engine
SixAxisDDS ddsEngine;
#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2
#define TMC_RX 15
#define TMC_TX 4
#define TMC2_RX 16
#define TMC2_TX 17

// Define your pins
const uint8_t STEP_PINS[6] = {Constants::Pins::J1_STEP_PIN, Constants::Pins::J2_STEP_PIN, Constants::Pins::J3_STEP_PIN, Constants::Pins::J4_STEP_PIN, Constants::Pins::J5_STEP_PIN, Constants::Pins::J6_STEP_PIN};
const uint8_t DIR_PINS[6]  = {Constants::Pins::J1_DIR_PIN, Constants::Pins::J2_DIR_PIN, Constants::Pins::J3_DIR_PIN, Constants::Pins::J4_DIR_PIN, Constants::Pins::J5_DIR_PIN, Constants::Pins::J6_DIR_PIN};
const float STEPS_PER_DEG[6] = {
    Constants::Config::J1_STEPS_PER_DEG,
    Constants::Config::J2_STEPS_PER_DEG,
    Constants::Config::J3_STEPS_PER_DEG,
    Constants::Config::J4_STEPS_PER_DEG,
    Constants::Config::J5_STEPS_PER_DEG,
    Constants::Config::J6_STEPS_PER_DEG
};
// Ruckig Setup (Assuming 1000 Hz loop)
ruckig::Ruckig<6> ruck(0.001); 
ruckig::InputParameter<6> input;
ruckig::OutputParameter<6> output;

// To maintain exact timing
unsigned long last_loop_time = 0;

int t0, t1, t2, t3, t4, t5;


void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    initJoints(1, 1, 1, 1, 1, 1, 1);
    
    // 1. Initialize the Hardware Timer and Pins
    ddsEngine.init(STEP_PINS, DIR_PINS);

    // 2. Setup Ruckig Targets (Example: Move to 1000 steps)
    // Note: Ruckig uses radians/degrees depending on your setup. 
    // For this example, let's pretend Ruckig operates directly in steps.
    for(int i=0; i<6; i++) {
        input.current_position[i] = 0.0;
        input.current_velocity[i] = 0.0;
        input.current_acceleration[i] = 0.0;
        
        input.target_position[i] = 0.0; 
        input.target_velocity[i] = 0.0;
        input.target_acceleration[i] = 0.0;

        input.synchronization = ruckig::Synchronization::Time;

        // The DDS step generator tops out at ~100,000 steps/s. Above that the
        // 32-bit phase increment overflows and the rate collapses to ~0, so the
        // axis loses steps (falls short of target) and stutters (jitter).
        // J6 has the most steps/deg (200*32*10/360 = 177.8), so a FLAT rad/s
        // limit silently blows past the ceiling on J6 only. Cap each axis in
        // STEP space, then convert to rad/s for Ruckig.
        const float MAX_STEP_RATE = 95000.0f;                 // steps/s, safe margin under 100k
        float max_deg_per_s = MAX_STEP_RATE / STEPS_PER_DEG[i];
        input.max_velocity[i]     = degToRad(max_deg_per_s);
        input.max_acceleration[i] = degToRad(max_deg_per_s) * 1.0;   // reach max vel in ~0.25 s
        input.max_jerk[i]         = degToRad(max_deg_per_s) * 1.5f;
    }
    t0 = 20;
    t1 = 20;
    t2 = 20;
    t3 = -90;
    t4 = 90;
    t5 = 90;
    input.target_position[0] = degToRad(t0);
    input.target_position[1] = degToRad(t1);
    input.target_position[2] = degToRad(t2);
    input.target_position[3] = degToRad(t3);
    input.target_position[4] = degToRad(t4);
    input.target_position[5] = degToRad(t5);
    last_loop_time = micros();
}
bool toggle = true;
void loop() {
    unsigned long current_time = micros();
    
    // Check if 1000 microseconds have passed
    if (current_time - last_loop_time >= 1000) {
        
        // BUGFIX: Add exactly 1000 to keep the clock perfectly rigid!
        // if (current_time - last_loop_time > 5000) {
        //     last_loop_time = current_time;
        // } else {
            last_loop_time += 1000;
        // }

        auto result = ruck.update(input, output);

        if (result == ruckig::Result::Working) {
            float target_speeds_hz[6];
            
            for(int i=0; i<6; i++) {
                // 1. Calculate base velocity from Ruckig
                float base_velocity = radToDeg(output.new_velocity[i]) * STEPS_PER_DEG[i];
                
                // 2. Calculate the exact physical drift
                float ideal_position = radToDeg(output.new_position[i]) * STEPS_PER_DEG[i]; 
                int32_t actual_position = ddsEngine.getSteps(i); 
                float position_error = ideal_position - (float)actual_position;
                
                // 3. APPLY SOFT CORRECTION (P-Gain of 40 instead of 1000)
                // This gently corrects 1 step of drift over 25 milliseconds
                // DEADBAND: Ignore mathematical rounding errors less than 1 step!
                float correction = 0.0f;
                if (position_error > 1.0f || position_error < -1.0f) {
                    correction = position_error * 3.0f;
                    
                    // Cap the correction so it doesn't violently spike
                    if (correction > 200.0f) correction = 200.0f;
                    if (correction < -200.0f) correction = -200.0f;
                }

                target_speeds_hz[i] = base_velocity + correction;
                
                /* * DEBUGGING STEP:
                 * If the motor STILL goes crazy, comment out the line above 
                 * and uncomment the line below to run PURE OPEN LOOP. 
                 * If open loop works, the DDS is perfect and the error is in your gear ratios.
                 */
                // target_speeds_hz[i] = base_velocity; 
            }

            ddsEngine.updateVelocities(target_speeds_hz);
            output.pass_to_input(input);
            
        } else if (result == ruckig::Result::Finished) {
            float zero_speeds[6] = {0, 0, 0, 0, 0, 0};
            ddsEngine.updateVelocities(zero_speeds);
            output.pass_to_input(input);
            if (toggle){
                input.target_position[0] = degToRad(0.0);
                input.target_position[1] = degToRad(0.0);
                input.target_position[2] = degToRad(0.0);
                input.target_position[3] = degToRad(0.0);
                input.target_position[4] = degToRad(0.0);
                input.target_position[5] = degToRad(0.0);
                toggle = false;
            }
            else
            {
                input.target_position[0] = degToRad(t0);
                input.target_position[1] = degToRad(t1);
                input.target_position[2] = degToRad(t2);
                input.target_position[3] = degToRad(t3);
                input.target_position[4] = degToRad(t4);
                input.target_position[5] = degToRad(t5);
                toggle = true;
            }
            
            
        }
        else if (result < 0) {
            // CATCH ERRORS: If Ruckig crashes, print it so we know!
            Serial.print("RUCKIG ERROR CODE: ");
            Serial.println(result);
        }
    }
}