#include <iostream>
#include <time.h>
#include <iomanip>
#include <cstdint>
#include <serialib.h>

#include <ruckig/ruckig.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <string>
#include <vector>
#include <array>

#include "protocol.h"   // EspToPiPacket / PiToEspPacket + COBS/CRC framing

#define SERIAL_PORT "/dev/ttyUSB1"

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

// --- Shared command state between the input REPL and the 500 Hz loop ---
static std::mutex               g_mtx;
static std::array<double, DOFs> g_target;          // last commanded targets (deg), guarded by g_mtx
static bool                     g_have_new = false; // guarded by g_mtx
static std::atomic<bool>        g_running{true};
static std::atomic<bool>        g_monitor{false};   // live one-line telemetry on/off

// Per-joint motion limits (deg/s, deg/s^2, deg/s^3). Editable live via the
// vel/acc/jerk commands (guarded by g_mtx once the REPL runs). J2 is the
// heaviest joint, so it starts more conservative -- tune these to taste.
static double g_max_vel[DOFs]  = {240, 80,  120,  314,  314,  314};
static double g_max_acc[DOFs]  = {600, 600,  600,  1200,  1200, 1200};
static double g_max_jerk[DOFs] = {1500, 800, 1000, 3000, 3000, 3000};
static bool   g_have_new_limits = false;  // guarded by g_mtx
static bool   g_stop_request    = false;  // guarded by g_mtx

static void printHelp() {
    std::cout <<
        "\nCommands:\n"
        "  a1 a2 a3 a4 a5 a6   set all six joint targets (deg)\n"
        "  <j> <angle>         move one joint (j = 1..6), e.g.  6 90\n"
        "  home                all joints to 0\n"
        "  stop                decelerate to a stop and hold\n"
        "  vel  <j|all> <v>    set max velocity (deg/s)\n"
        "  acc  <j|all> <v>    set max acceleration (deg/s^2)\n"
        "  jerk <j|all> <v>    set max jerk (deg/s^3)\n"
        "  limits              show current motion limits\n"
        "  mon                 toggle live position monitor\n"
        "  help                show this help\n"
        "  q / quit            exit\n";
}

// Background REPL: reads command lines and updates the shared targets. Runs on
// its own thread so the 500 Hz streaming loop is never blocked by user input.
static void inputThread() {
    printHelp();
    std::string line;
    while (g_running.load()) {
        std::cout << "\n> " << std::flush;
        if (!std::getline(std::cin, line)) { g_running = false; break; }  // Ctrl-D

        std::istringstream iss(line);
        std::vector<std::string> tok; std::string t;
        while (iss >> t) tok.push_back(t);
        if (tok.empty()) continue;

        if (tok[0] == "q" || tok[0] == "quit" || tok[0] == "exit") { g_running = false; break; }
        if (tok[0] == "help" || tok[0] == "h") { printHelp(); continue; }
        if (tok[0] == "mon") {
            g_monitor = !g_monitor.load();
            std::cout << "monitor " << (g_monitor ? "ON" : "OFF") << "\n";
            continue;
        }
        if (tok[0] == "stop") {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_stop_request = true;
            std::cout << "stopping\n";
            continue;
        }
        if (tok[0] == "limits") {
            std::lock_guard<std::mutex> lk(g_mtx);
            std::cout << "        vel      acc      jerk\n";
            for (int i = 0; i < DOFs; i++)
                std::cout << "  J" << (i + 1) << ":  " << g_max_vel[i]
                          << "     " << g_max_acc[i] << "     " << g_max_jerk[i] << "\n";
            continue;
        }
        if (tok[0] == "vel" || tok[0] == "acc" || tok[0] == "jerk") {
            if (tok.size() != 3) { std::cout << "usage: " << tok[0] << " <j|all> <value>\n"; continue; }
            double* arr = (tok[0] == "vel") ? g_max_vel : (tok[0] == "acc") ? g_max_acc : g_max_jerk;
            try {
                double val = std::stod(tok[2]);
                if (val <= 0.0) { std::cout << "value must be > 0\n"; continue; }
                std::lock_guard<std::mutex> lk(g_mtx);
                if (tok[1] == "all") {
                    for (int i = 0; i < DOFs; i++) arr[i] = val;
                } else {
                    int j = std::stoi(tok[1]);
                    if (j < 1 || j > DOFs) { std::cout << "joint must be 1.." << DOFs << "\n"; continue; }
                    arr[j - 1] = val;
                }
                g_have_new_limits = true;
                std::cout << tok[0] << " updated\n";
            } catch (const std::exception&) {
                std::cout << "could not parse -- usage: " << tok[0] << " <j|all> <value>\n";
            }
            continue;
        }

        try {
            std::lock_guard<std::mutex> lk(g_mtx);
            if (tok[0] == "home") {
                g_target.fill(0.0);
            } else if ((int)tok.size() == DOFs) {         // six numbers -> all joints
                for (int i = 0; i < DOFs; i++) g_target[i] = std::stod(tok[i]);
            } else if (tok.size() == 2) {                 // "<joint> <angle>"
                int j = std::stoi(tok[0]);
                if (j < 1 || j > DOFs) { std::cout << "joint must be 1.." << DOFs << "\n"; continue; }
                g_target[j - 1] = std::stod(tok[1]);
            } else {
                std::cout << "unrecognised command -- type 'help'\n";
                continue;
            }
            g_have_new = true;
            std::cout << "target updated\n";
        } catch (const std::exception&) {
            std::cout << "could not parse numbers -- type 'help'\n";
        }
    }
}

int main(int argc, char** argv) {
    // Serial port can be given as the first CLI arg, else defaults to SERIAL_PORT.
    const char* port = (argc > 1) ? argv[1] : SERIAL_PORT;

    // Open High-Speed Serial Port (e.g., /dev/ttyAMA0 or /dev/ttyUSB0)
    serialib serial;

    // Connection to serial port
    if (serial.openDevice(port, 921600) != 1) {
        std::cerr << "Failed to open serial port: " << port << std::endl;
        return 1;
    }
    std::cout << "Successful connection to " << port << std::endl;

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

        input.max_velocity[i]     = g_max_vel[i];
        input.max_acceleration[i] = g_max_acc[i];
        input.max_jerk[i]         = g_max_jerk[i];
    }

    // Seed the shared targets with the current position, then start the REPL.
    for (int i = 0; i < DOFs; i++) g_target[i] = input.target_position[i];
    std::thread repl(inputThread);

    std::cout << "\nStreaming at 500 Hz. Enter commands below.\n";

    serial.flushReceiver();

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (g_running.load()) {

        // Apply any freshly entered target(s)/limits/stop from the REPL thread.
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            if (g_have_new) {
                for (int i = 0; i < DOFs; i++) input.target_position[i] = g_target[i];
                g_have_new = false;
            }
            if (g_have_new_limits) {
                for (int i = 0; i < DOFs; i++) {
                    input.max_velocity[i]     = g_max_vel[i];
                    input.max_acceleration[i] = g_max_acc[i];
                    input.max_jerk[i]         = g_max_jerk[i];
                }
                g_have_new_limits = false;
            }
            if (g_stop_request) {
                // Retarget to the current commanded position -> Ruckig ramps to a stop.
                for (int i = 0; i < DOFs; i++) {
                    input.target_position[i] = input.current_position[i];
                    g_target[i]              = input.current_position[i];
                }
                g_stop_request = false;
            }
        }

        auto res = ruck.update(input, output);
        
        // Pass output state to the next cycle's input
        input.current_position = output.new_position;
        input.current_velocity = output.new_velocity;
        input.current_acceleration = output.new_acceleration;

        // --- STREAM POSITION + VELOCITY for the ESP moveTimed feeder ---
        // Position pins the step count (drift-free); velocity sets the step rate.
        for(int i = 0; i < DOFs; i++) {
            tx_packet.pos_cmd[i] = output.new_position[i];
            tx_packet.vel_cmd[i] = output.new_velocity[i];
        }
        output.pass_to_input(input);

        // 2. Send Velocity Command (COBS-framed + CRC16)
        serial.writeBytes(txbuf, frameEncode(tx_packet, txbuf));

        // 3. Wait for a valid framed reply (bounded to the ~2ms cycle budget)
        readFramedPacket(serial, rx_reader, rx_packet, 1);  // on drop, keep last good rx_packet

        // 4. Optional live telemetry -- throttled to ~5 Hz on a single line so it
        //    doesn't flood the REPL. Toggle with the 'mon' command (off by default).
        static int telem = 0;
        if (g_monitor.load() && (++telem % 100 == 0)) {
            std::cout << "\r[mon] " << std::fixed << std::setprecision(2)
                      << "J1:" << std::setw(7) << rx_packet.actual_position[0]
                      << " J2:" << std::setw(7) << rx_packet.actual_position[1]
                      << " J3:" << std::setw(7) << rx_packet.actual_position[2]
                      << " J4:" << std::setw(7) << rx_packet.actual_position[3]
                      << " J5:" << std::setw(7) << rx_packet.actual_position[4]
                      << " J6:" << std::setw(7) << rx_packet.actual_position[5]
                      << "     " << std::flush;
        }

        // 5. Sleep exactly until the next 2ms interval (500 Hz)
        next_tick.tv_nsec += 2000000; // Add 2ms
        if (next_tick.tv_nsec >= 1000000000) {
            next_tick.tv_nsec -= 1000000000;
            next_tick.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, NULL);
    }

    g_running = false;
    if (repl.joinable()) repl.join();
    serial.closeDevice();
    return 0;
}