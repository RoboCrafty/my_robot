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
// #include <dds_stepgen.h>   // replaced by FastAccelStepper moveTo() approach
#include <ruckig/ruckig.hpp>

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2
#define TMC_RX 15
#define TMC_TX 4
#define TMC2_RX 16
#define TMC2_TX 17
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

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

// FastAccelStepper direction polarity (dir-pin-high counts up) per axis,
// taken from your previous working setup.
const bool DIR_HIGH_COUNT_UP[6] = {false, true, true, false, false, false};

// --- Shared command buffer: written by the 1 kHz Ruckig loop, read by the
//     2 kHz servo ISR. This is the library's cmd.pos / cmd.vel pattern. ---
volatile int32_t  cmd_pos_steps[6] = {0, 0, 0, 0, 0, 0};   // absolute target (steps)
volatile uint32_t cmd_vel_mhz[6]   = {0, 0, 0, 0, 0, 0};   // feedforward speed |v| (milliHz)
volatile bool     machineEnabled   = false;

hw_timer_t* servoTimer = nullptr;

// Per-axis ramp-state cache (detects the accel->coast transition, exactly like
// prevRampState[] in the LinuxCNC library).
uint8_t prevRampState[6] = {0, 0, 0, 0, 0, 0};

// The library's axisVelScaleFactor. 1.0 = no scaling. Drop below 1 to globally
// trim commanded speed (kept so the structure matches the source 1:1).
const float axisVelScaleFactor = 1.0f;

// ---------------------------------------------------------------------------
// SERVO ISR @ 2 kHz (every 500 us) -- faithful port of ServoMovementCmds_ISR()
// from the ESP32_LinuxCNC controller. Source of position/velocity is Ruckig
// (via cmd_pos_steps[]/cmd_vel_mhz[]) instead of a LinuxCNC UDP packet. Only the
// LinuxCNC feedback (fb.*), the debug axis-state console viz, and the ESP-NOW
// telemetry were dropped; ALL motion-critical logic is preserved:
//   * soft linear-accel start when a stationary axis begins moving,
//   * conditional speed update with the +/-10000 mHz velDiff dead-zone,
//   * on a LARGE deceleration we do NOT lower the speed limit -- we let moveTo()
//     ramp down into the target so the stop isn't laggy,
//   * switch to mild linear-accel once the axis reaches coast speed,
//   * moveTo(absolute position) every cycle so position never drifts.
// NOTE: moveTo()/setSpeedInMilliHz() are not in IRAM (same as the source lib).
// Fine during steady-state motion; if you ever see random reboots, move this
// body into a high-priority FreeRTOS task instead.
// ---------------------------------------------------------------------------
void IRAM_ATTR ServoMovementCmds_ISR() {
    if (!machineEnabled) return;

    for (int i = 0; i < 6; i++) {
        if (!steppers[i]) continue;

        uint32_t newVel = cmd_vel_mhz[i];   // |velocity| feedforward (milliHz)

        if (newVel > 0) {                    // a move is wanted
            bool isRampActive = steppers[i]->isRampGeneratorActive();

            if (!isRampActive) {
                // Initial move request (stationary -> moving): gentle soft start.
                steppers[i]->setLinearAcceleration(2000);
                steppers[i]->setSpeedInMilliHz(newVel / axisVelScaleFactor);
            } else {
                // Already moving: decide whether to push a new speed limit.
                newVel = newVel / axisVelScaleFactor;
                int32_t curSpeed = steppers[i]->getCurrentSpeedInMilliHz(true);
                if (curSpeed < 0) curSpeed = -curSpeed;
                const int32_t velDiff = (int32_t)newVel - (int32_t)(curSpeed / axisVelScaleFactor);

                if (velDiff > 10000) {            // accelerating > 10 Hz
                    steppers[i]->setLinearAcceleration(0);
                    steppers[i]->setSpeedInMilliHz(newVel);
                } else if (velDiff > -10000) {    // steady / mild change
                    steppers[i]->setLinearAcceleration(0);
                    steppers[i]->setSpeedInMilliHz(newVel);
                }
                // else: large deceleration -> KEEP the current (higher) speed
                // limit and let moveTo() decelerate into the target. Lowering it
                // here causes a laggy stop. (Critical line from the library.)
            }

            // Absolute position anchor -- always called so position never drifts.
            MoveResultCode moveResult = steppers[i]->moveTo(cmd_pos_steps[i], false);

            if (moveResult == MOVE_OK && steppers[i]->isRampGeneratorActive()) {
                uint8_t rampState = steppers[i]->rampState();
                if (rampState != prevRampState[i]) {
                    if (rampState & RAMP_STATE_COAST) {
                        // Reached coasting/at-speed: relax to mild linear accel.
                        steppers[i]->setLinearAcceleration(100);
                    }
                    prevRampState[i] = rampState;
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(1000);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 1, 1, 1, 1, 1, 1);
    initLimitSwitches();

    // --- FastAccelStepper engine + per-axis setup ---
    engine.init();
    for (int i = 0; i < 6; i++) {
        steppers[i] = engine.stepperConnectToPin(STEP_PINS[i]);
        if (steppers[i]) {
            steppers[i]->setDirectionPin(DIR_PINS[i], DIR_HIGH_COUNT_UP[i]);
            steppers[i]->setAutoEnable(true);
            // High accel so FAS faithfully follows Ruckig's commanded velocity
            // each cycle -- Ruckig already does the jerk-limited smoothing.
            steppers[i]->setAcceleration(10000);
            steppers[i]->setSpeedInHz(6000);        // safe initial speed
            steppers[i]->applySpeedAcceleration();
            steppers[i]->setCurrentPosition(0);
        }
    }

    // homeAxis(1);
    // homeAxis(2);
    // homeAxis(3);
    // homeAxis(4);
    // homeAxis(6);
    // homeAxis(5);
    delay(3000); // Wait for homing to complete



    for (int i = 0; i < 6; i++) {

        steppers[i]->setAcceleration(1000000);
        steppers[i]->setSpeedInHz(1000);        // safe initial speed
        steppers[i]->applySpeedAcceleration();
        steppers[i]->setCurrentPosition(0);
    }
    // 2. Setup Ruckig limits (radians) and targets
    for(int i=0; i<6; i++) {
        input.current_position[i] = 0.0;
        input.current_velocity[i] = 0.0;
        input.current_acceleration[i] = 0.0;
        
        input.target_position[i] = 0.0; 
        input.target_velocity[i] = 0.0;
        input.target_acceleration[i] = 0.0;

        // Time sync = all axes finish together (coordinated). DO NOT use Phase:
        // when only some axes move (zero displacement on the rest) Phase sync is
        // degenerate for the single-precision solver and produces -110 errors
        // plus occasional garbage positions that jam the controller.
        // input.synchronization = ruckig::Synchronization::Time;

        // Cap each axis' velocity in STEP space (so high-resolution axes don't
        // demand absurd step rates), then convert to rad/s for Ruckig.
        const float MAX_DEG_PER_SEC = 50;                 // steps/s ceiling per axis
        float max_rad_per_s = degToRad(MAX_DEG_PER_SEC);
        input.max_velocity[i]     = max_rad_per_s / 1;
        input.max_acceleration[i] = max_rad_per_s * 5.0;   // reach max vel in ~0.25 s
        input.max_jerk[i]         = max_rad_per_s * 10.0f;
    }
    // t0 = 20;
    // t1 = 20;
    // t2 = -20;
    // t3 = -90;
    // t4 = 90;
    t0 = 0;
    t1 = 0;
    t2 = 0;
    t3 = 0;
    t4 = 0;
    t5 = 150;
    input.target_position[0] = degToRad(t0);
    input.target_position[1] = degToRad(t1);
    input.target_position[2] = degToRad(t2);
    input.target_position[3] = degToRad(t3);
    input.target_position[4] = degToRad(t4);
    input.target_position[5] = degToRad(t5);

    // --- Start the 2 kHz servo ISR (every 500 us) ---
    // servoTimer = timerBegin(1000000);                  // 1 MHz base
    // timerAttachInterrupt(servoTimer, &ServoMovementCmds_ISR);
    // timerAlarm(servoTimer, 500, true, 0);              // 500 ticks = 500 us = 2 kHz
    // machineEnabled = true;

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

            // portENTER_CRITICAL(&myMutex);
            for (int i = 0; i < 6; i++) {
                // Absolute position target (steps) + velocity feedforward (milliHz)
                float pos_steps = radToDeg(output.new_position[i]) * STEPS_PER_DEG[i];
                float vel_hz    = radToDeg(output.new_velocity[i]) * STEPS_PER_DEG[i];
                
                cmd_pos_steps[i] = (int32_t)lroundf(pos_steps);
                cmd_vel_mhz[i]   = (uint32_t)(fabsf(vel_hz) * 1000.0f);   // steps/s -> milliHz

                steppers[i]->setSpeedInMilliHz(cmd_vel_mhz[i]);
                steppers[i]->moveTo(cmd_pos_steps[i], false);
            }
            // portEXIT_CRITICAL(&myMutex);
            output.pass_to_input(input);

        } else if (result == ruckig::Result::Finished) {

            // Serial.println("Ruckig trajectory finished. ");
            // Hold position: feedforward velocity 0, target already at goal.
            for (int i = 0; i < 6; i++) {
                cmd_vel_mhz[i] = 0;
                steppers[i]->setSpeedInMilliHz(cmd_vel_mhz[i]);
            }
            output.pass_to_input(input);
            // Resync Ruckig's current position to the actual stepper position
            // (moveTo locks position so these already match -- belt & braces).
            // for(int i = 0; i < 6; i++) {
            //     if (steppers[i]) {
            //         float physical_degrees = (float)steppers[i]->getCurrentPosition() / STEPS_PER_DEG[i];
            //         input.current_position[i] = degToRad(physical_degrees);
            //     }
            // }
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
            
            // Serial.print("Current: ");
            // Serial.print(radToDeg(input.current_position[5]));
            // Serial.print("  Target: ");
            // Serial.println(radToDeg(input.target_position[5]));
        }
        else if (result < 0) {
            // CATCH ERRORS: If Ruckig crashes, print it so we know!
            // Serial.print("RUCKIG ERROR CODE: ");
            // Serial.println(result);

            // RECOVER: re-seed Ruckig from the actual stepper positions and zero
            // out velocity/acceleration. Without this, a single bad update leaves
            // current_position corrupt and every following update returns -110
            // forever (the endless error wall). This unjams the controller.
            // for (int i = 0; i < 6; i++) {
            //     cmd_vel_mhz[i] = 0;
            //     float physical_deg = steppers[i]
            //         ? (float)steppers[i]->getCurrentPosition() / STEPS_PER_DEG[i]
            //         : 0.0f;
            //     input.current_position[i]     = degToRad(physical_deg);
            //     input.current_velocity[i]     = 0.0;
            //     input.current_acceleration[i] = 0.0;
            // }
        }
    }
}