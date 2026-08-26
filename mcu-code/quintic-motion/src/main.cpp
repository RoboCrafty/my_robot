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

// --- moveTimed feeder (step-separated, per FastAccelStepper issue #363) ---
// TICKS_PER_S and MIN_CMD_TICKS are provided by FastAccelStepper (pd_config.h).
static const uint32_t CTRL_TICKS         = TICKS_PER_S / 500; // 2 ms control period
static const uint32_t MIN_TICKS_PER_STEP = 160;             // ~100 kHz per-step ceiling guard
static const float    V_FLOOR            = 1.0f;            // steps/s below which we don't rate-time
int32_t  queued_steps[6]      = {0, 0, 0, 0, 0, 0};        // steps already appended per axis
uint32_t movetimed_underruns  = 0;                         // diagnostics: queue-empty events
int32_t tick_error[6] = {0, 0, 0, 0, 0, 0};                // Tracks accumulated timer quantization error per axis (in ticks)

void setup() {
    Serial.begin(921600);
    Serial.setTimeout(2);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Route PacketSerial over the USB UART and register the frame callback.
    packetSerial.setStream(&Serial);
    packetSerial.setPacketHandler(&onPacket);

    // Initialise
    // delay(1000);
    // // Serial.println("\n--- Initializing System ---");
    // delay(100);

    initJoints(1, 1, 1, 1, 1, 1, 1);
    // initJoints(1, 0, 0, 0, 0, 0, 0);
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
            steppers[i]->setAcceleration(6000);
            steppers[i]->setSpeedInHz(6000);        // safe initial speed
            steppers[i]->applySpeedAcceleration();
            steppers[i]->setCurrentPosition(0);
        }
    }

    homeAxis(1);
    homeAxis(2);
    homeAxis(3);
    homeAxis(4);
    homeAxis(6);
    homeAxis(5);
    delay(3000); // Wait for homing to complete



    for (int i = 0; i < 6; i++) {
        steppers[i]->setCurrentPosition(0);
    }


    last_loop_time = millis();
}


PiToEspPacket rx_packet;
EspToPiPacket tx_packet;
uint8_t applied_motor_enable_mask = 0x3f;

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

    uint8_t requested_mask = rx_packet.motor_enable_mask & 0x3f;
    uint8_t changed_mask = requested_mask ^ applied_motor_enable_mask;
    for (int i = 0; i < 6; i++) {
        if (changed_mask & (1u << i)) {
            if (requested_mask & (1u << i)) {
                turnDriverOn(i + 1);
            } else {
                int32_t executed_steps = steppers[i]->getCurrentPosition();
                steppers[i]->forceStopAndNewPosition(executed_steps);
                queued_steps[i] = executed_steps;
                turnDriverOff(i + 1);
            }
        }
    }
    applied_motor_enable_mask = requested_mask;

    if (rx_packet.motor_enable_mask & 0x40) {
        homeAxis(1);
        homeAxis(2);
        homeAxis(3);
        homeAxis(4);
        homeAxis(6);
        homeAxis(5);
        delay(3000);
        for (int i = 0; i < 6; i++) {
            steppers[i]->forceStopAndNewPosition(0);
            queued_steps[i] = 0;
            tick_error[i] = 0;
        }
        tx_packet.homing_sequence++;
    }

    // Sync (0x80): re-reference the queue to the executed position so the host
    // can adopt motor feedback as its planner state without commanding motion.
    if (rx_packet.motor_enable_mask & 0x80) {
        for (int i = 0; i < 6; i++) {
            queued_steps[i] = steppers[i]->getCurrentPosition();
            tick_error[i] = 0;
        }
    }

    // --- moveTimed feeder (step-separated, per gin66 issue #363) ---
    // Steps are pinned to the commanded POSITION (drift-free); each command's
    // DURATION is derived from the commanded VELOCITY, so the step RATE is
    // constant within the command -- this avoids the fixed-1ms-frame
    // quantization that produces the harsh 1/2-step-per-frame speed noise.
    for (int i = 0; i < 6; i++) {
        if (!(requested_mask & (1u << i))) {
            queued_steps[i] = steppers[i]->getCurrentPosition();
            continue;
        }

        int32_t target = lroundf(rx_packet.pos_cmd[i] * STEPS_PER_DEG[i]);
        int32_t n      = target - queued_steps[i];                  
        // FIX: Clamp `n` to int16_t bounds so the physical motor and planner never desync
        if (n > 32767) n = 32767;
        if (n < -32768) n = -32768;      
        float   v      = fabsf(rx_packet.vel_cmd[i]) * STEPS_PER_DEG[i];  

        if (n == 0) {
            if (!steppers[i]->isRunning() || steppers[i]->isQueueEmpty()) {
                steppers[i]->moveTimed(0, CTRL_TICKS, nullptr, true);
            }
            tick_error[i] = 0; // Reset error wind-up when stopped
            continue;
        }

        uint32_t abs_n = (n < 0) ? (uint32_t)(-n) : (uint32_t)n;
        uint32_t ideal_duration;

        if (v > V_FLOOR) {
            ideal_duration = (uint32_t)(((float)abs_n * (float)TICKS_PER_S) / v);
        } else {
            // "Catch-up" block for leftover steps when v == 0
            float safe_catchup_speed = 2000.0f; 
            ideal_duration = (uint32_t)(((float)abs_n * (float)TICKS_PER_S) / safe_catchup_speed);
            tick_error[i] = 0; // Reset error, we are off the ideal timeline anyway
        }

        // Apply our accumulated error to the requested duration
        int32_t requested_duration = (int32_t)ideal_duration - tick_error[i];

        // A very slow (time-synchronized) axis would otherwise stretch one step
        // across many control periods, keeping FAS BUSY so queued_steps lags the
        // target; the backlog then flushes in one burst at end-of-move (the J4
        // jump + lost steps). Cap the command so the axis frees up each cycle.
        bool capped = false;
        if (requested_duration > (int32_t)(2 * CTRL_TICKS)) {
            requested_duration = (int32_t)(2 * CTRL_TICKS);
            capped = true;
        }

        // Clamp to safe limits based on the library's requirements
        uint32_t min_dur = abs_n * MIN_TICKS_PER_STEP;
        if (requested_duration < (int32_t)min_dur) requested_duration = min_dur;
        if (requested_duration < MIN_CMD_TICKS)    requested_duration = MIN_CMD_TICKS;

        uint32_t actual_duration = 0;
        MoveTimedResultCode r = steppers[i]->moveTimed((int16_t)n, (uint32_t)requested_duration, &actual_duration, true);
        
        if (r == MOVE_TIMED_OK || r == MOVE_TIMED_EMPTY) {
            queued_steps[i] += n;                                 
            if (r == MOVE_TIMED_EMPTY) movetimed_underruns++;     

            // Accumulate timer-quantization error only while tracking the ideal
            // timeline; when capped the timeline is intentionally abandoned.
            if (v > V_FLOOR && !capped) {
                tick_error[i] += ((int32_t)actual_duration - (int32_t)ideal_duration);
            } else {
                tick_error[i] = 0;
            }
        }
    }

    // Feedback: report the actual (as-executed) position from the step counter.
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
        // printLimitSwitchStates();
    }
        
}

void moveJoint()
{

}

void printLimitSwitchStates()
{
    Serial.printf(
            "Limits raw: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d | triggered: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\n",
            digitalRead(Constants::Pins::L1_PIN),
            digitalRead(Constants::Pins::L2_PIN),
            digitalRead(Constants::Pins::L3_PIN),
            digitalRead(Constants::Pins::L4_PIN),
            digitalRead(Constants::Pins::L5_PIN),
            digitalRead(Constants::Pins::L6_PIN),
            isLimitSwitchTriggered(1),
            isLimitSwitchTriggered(2),
            isLimitSwitchTriggered(3),
            isLimitSwitchTriggered(4),
            isLimitSwitchTriggered(5),
            isLimitSwitchTriggered(6));
    
}