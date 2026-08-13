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

void setup() {
    Serial.begin(1000000);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(1000);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    // initJoints(1, 1, 1, 1, 1, 1, 1);
    initJoints(1, 0, 0, 0, 0, 0, 0);
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

        steppers[i]->setAcceleration(15000);
        steppers[i]->setSpeedInHz(10000);        // safe initial speed
        steppers[i]->applySpeedAcceleration();
        steppers[i]->setCurrentPosition(0);
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


    last_loop_time = micros();
}


PiToEspPacket rx_packet;
EspToPiPacket tx_packet;


void loop() {
     unsigned long current_time = millis();
    if (Serial1.available() >= sizeof(PiToEspPacket)) {
        
        // 2. Read the velocity commands
        Serial1.readBytes((uint8_t*)&rx_packet, sizeof(PiToEspPacket));
        
        // 3. Process velocity (rx_packet.v_cmd) and update motors
        // ...
        
        // 4. Read actual hardware positions into tx_packet
        for(int i=0; i<6; i++) {
            tx_packet.actual_position[i] = steppers[i]->getCurrentPosition();
        }
        
        // 5. Send positions back to Pi immediately
        Serial1.write((uint8_t*)&tx_packet, sizeof(EspToPiPacket));
    }

    if (current_time - last_loop_time >= 100) {
        last_loop_time += 100;
        
    }

}