#include <iostream>
#include <time.h>
#include <iomanip>
#include <cstdint>
#include <serialib.h>

#include <ruckig/ruckig.hpp>

#include "protocol.h"   // EspToPiPacket / PiToEspPacket + COBS/CRC framing

#define SERIAL_PORT "/dev/ttyUSB0"

// Reads framed bytes from `serial` until a CRC-valid EspToPiPacket arrives or
// the timeout elapses. Returns true on success. Recovers from desync/corruption.
static bool readFramedPacket(serialib& serial, FrameReader<EspToPiPacket>& reader,
                             EspToPiPacket& out, int timeout_ms) {
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_nsec += (long)timeout_ms * 1000000L;
    while (deadline.tv_nsec >= 1000000000L) { deadline.tv_nsec -= 1000000000L; deadline.tv_sec++; }

    for (;;) {
        char b;
        if (serial.readChar(&b, 1) == 1) {
            if (reader.push((uint8_t)b, out)) return true;
        }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec)) {
            return false;
        }
    }
}

using namespace ruckig;
const int DOFs = 6;
inline Ruckig<DOFs> ruck(0.002);
inline InputParameter<DOFs> input;
inline OutputParameter<DOFs> output;
const float K_P = 10.0f;

int main() {
    // Open High-Speed Serial Port (e.g., /dev/ttyAMA0 or /dev/ttyUSB0)
    serialib serial;

    // Connection to serial port
    if (serial.openDevice(SERIAL_PORT, 921600) != 1) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return 1;
    }
    std::cout << "Successful connection to " << SERIAL_PORT << std::endl;

    PiToEspPacket tx_packet = {0};
    EspToPiPacket rx_packet = {0};
    FrameReader<EspToPiPacket> rx_reader;
    uint8_t txbuf[frameMaxLen<PiToEspPacket>()];

    // 1. Synchronize Initial Position
    // The ESP32 only replies when we send something. Send a dummy zero-velocity packet.
    serial.writeBytes(txbuf, frameEncode(tx_packet, txbuf));
    if (!readFramedPacket(serial, rx_reader, rx_packet, 200)) {
        std::cerr << "Warning: no initial position received from ESP32!" << std::endl;
    }

    for (int i = 0; i < DOFs; i++) {
        float initial_rads = rx_packet.actual_position[i];
        input.current_position[i] = initial_rads;
        input.current_velocity[i] = 0.0;
        input.current_acceleration[i] = 0.0;
        
        // Default target to where we currently are
        input.target_position[i] = initial_rads; 
        input.target_velocity[i] = 0.0;
        input.target_acceleration[i] = 0.0;

        input.max_velocity[i] = 314.0;       // ~114 deg/s
        input.max_acceleration[i] = 600.0;   // ~229 deg/s^2
        input.max_jerk[i] = 1500.0;          // Jerk limiting for smooth S-curves
    }

    // 2. Read Target Angle from User (Pauses execution here)
    std::cout << "Enter target angle for Joint 6: ";
    float target_angle = 0.0f;
    std::cin >> target_angle;
    input.target_position[5] = target_angle; // Update J6 target
    
    std::cout << "\nStarting 500Hz communication..." << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    serial.flushReceiver();

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (true) {

        auto res = ruck.update(input, output);
        
        // Pass output state to the next cycle's input
        input.current_position = output.new_position;
        input.current_velocity = output.new_velocity;
        input.current_acceleration = output.new_acceleration;

        // --- 1. CONTROL LOOP & UNIT CONVERSION ---
        for(int i = 0; i < DOFs; i++) {
            // Convert actual hardware steps to degrees
            float actual_pos_deg = rx_packet.actual_position[i];

            // Calculate error
            float pos_error = output.new_position[i] - actual_pos_deg;

            // Calculate command velocity (Feedforward + Proportional Error)
            float cmd_vel_deg = output.new_velocity[i] + (K_P * pos_error);

            // Assign to TX packet
            tx_packet.v_cmd[i] = cmd_vel_deg;
        }
        output.pass_to_input(input);

        // 2. Send Velocity Command (COBS-framed + CRC16)
        serial.writeBytes(txbuf, frameEncode(tx_packet, txbuf));

        // 3. Wait for a valid framed reply (bounded to the ~2ms cycle budget)
        if (!readFramedPacket(serial, rx_reader, rx_packet, 1)) {
            std::cerr << "\nWarning: no valid packet this cycle (dropped/corrupt)!" << std::endl;
            // rx_packet keeps its last good value; skip using stale data if desired.
        }

        // 4. Process rx_packet here...
        std::cout << "\n" 
                  << "J1: " << std::setw(6) << rx_packet.actual_position[0] << " | "
                  << "J2: " << std::setw(6) << rx_packet.actual_position[1] << " | "
                  << "J3: " << std::setw(6) << rx_packet.actual_position[2] << " | "
                  << "J4: " << std::setw(6) << rx_packet.actual_position[3] << " | "
                  << "J5: " << std::setw(6) << rx_packet.actual_position[4] << " | "
                  << "J6: " << std::setw(6) << rx_packet.actual_position[5] 
                  << "        " << std::flush; // <-- Extra spaces here

        // 5. Sleep exactly until the next 2ms interval (500 Hz)
        next_tick.tv_nsec += 2000000; // Add 2ms
        if (next_tick.tv_nsec >= 1000000000) {
            next_tick.tv_nsec -= 1000000000;
            next_tick.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, NULL);
    }

    serial.closeDevice();
    return 0;
}