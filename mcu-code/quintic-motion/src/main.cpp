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
#include <PacketSerial.h>
// #include <motion_planner.h>


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

// PacketSerial = COBS framing over Serial. It handles delimiting/resync for us;
// we add a CRC16 inside each payload (PacketSerial does not checksum).
PacketSerial packetSerial;
void onPacket(const uint8_t* buffer, size_t size); // defined below

void setup() {
    Serial.begin(921600);
    Serial.setTimeout(2);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Route PacketSerial over the USB UART and register the frame callback.
    packetSerial.setStream(&Serial);
    packetSerial.setPacketHandler(&onPacket);

    // Initialise
    delay(1000);
    // Serial.println("\n--- Initializing System ---");
    delay(100);

    // initJoints(1, 1, 1, 1, 1, 1, 1);
    initJoints(1, 0, 0, 0, 0, 0, 1);
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
            steppers[i]->setAcceleration(100000);
            steppers[i]->setSpeedInHz(6000);        // safe initial speed
            steppers[i]->applySpeedAcceleration();
            steppers[i]->setCurrentPosition(0);
        }
    }

    // homeAxis(1);
    // homeAxis(2);
    // homeAxis(3);
    // homeAxis(4);
    homeAxis(6);
    // homeAxis(5);
    delay(3000); // Wait for homing to complete



    for (int i = 0; i < 6; i++) {

        steppers[i]->setAcceleration(100000);
        steppers[i]->setSpeedInHz(00000);        // safe initial speed
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


    last_loop_time = millis();
}


PiToEspPacket rx_packet;
EspToPiPacket tx_packet;

// Called by PacketSerial once a complete COBS frame has been received and
// un-stuffed. Framing/resync is handled by the library; we verify the CRC16
// we appended to the payload, then act and reply.
void onPacket(const uint8_t* buffer, size_t size) {
    if (size != sizeof(PiToEspPacket) + 2) return;             // wrong length
    uint16_t crc = crc16_ccitt(buffer, sizeof(PiToEspPacket));
    uint16_t rx  = (uint16_t)buffer[sizeof(PiToEspPacket)] |
                   ((uint16_t)buffer[sizeof(PiToEspPacket) + 1] << 8);
    if (crc != rx) return;                                     // corrupt -> ignore

    memcpy(&rx_packet, buffer, sizeof(PiToEspPacket));

    // Process velocity (rx_packet.v_cmd) and update motors
    for (int i = 0; i < 6; i++) {
        float cmd_vel = rx_packet.v_cmd[i] * STEPS_PER_DEG[i];

        // Convert float Hz to integer milliHz for maximum resolution
        uint32_t speed_mHz = (uint32_t)(abs(cmd_vel) * 1000.0f);

        if (speed_mHz == 0) {
            steppers[i]->stopMove();
        } else {
            steppers[i]->setSpeedInMilliHz(speed_mHz);
            steppers[i]->applySpeedAcceleration();
            if (cmd_vel > 0) steppers[i]->runForward();
            else             steppers[i]->runBackward();
        }
    }

    // Read actual hardware positions and reply as [EspToPiPacket | crc16].
    for (int i = 0; i < 6; i++) {
        tx_packet.actual_position[i] = steppers[i]->getCurrentPosition() / STEPS_PER_DEG[i];
    }
    uint8_t payload[sizeof(EspToPiPacket) + 2];
    memcpy(payload, &tx_packet, sizeof(EspToPiPacket));
    uint16_t tcrc = crc16_ccitt(payload, sizeof(EspToPiPacket));
    payload[sizeof(EspToPiPacket)]     = (uint8_t)(tcrc & 0xFF);
    payload[sizeof(EspToPiPacket) + 1] = (uint8_t)(tcrc >> 8);
    packetSerial.send(payload, sizeof(payload)); // PacketSerial COBS-frames it
}


void loop() {
    unsigned long current_time = millis();

    // Pumps the UART: reads bytes, un-stuffs COBS frames, fires onPacket().
    packetSerial.update();

    if (current_time - last_loop_time >= 100) {
        last_loop_time += 100;

    }
}

void moveJoint()
{

}