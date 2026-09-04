#include <iostream>
#include <time.h>
#include <iomanip>
#include <cstdint>
#include <serialib.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <ruckig/ruckig.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>

#include "protocol.h"   // EspToPiPacket / PiToEspPacket + COBS/CRC framing
#include "kinematics.hpp" // Pinocchio FK/IK (radians, metres)

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
static std::atomic<bool>        g_stats{false};     // loop/serial counters on/off

// Per-joint motion limits (deg/s, deg/s^2, deg/s^3). Editable live via the
// vel/acc/jerk commands (guarded by g_mtx once the REPL runs). J2 is the
// heaviest joint, so it starts more conservative -- tune these to taste.
static double g_max_vel[DOFs]  = {240, 80,  120,  314,  314,  314};
static double g_max_acc[DOFs]  = {600, 600,  600,  1200,  1200, 1200};
static double g_max_jerk[DOFs] = {1500, 800, 1000, 3000, 3000, 3000};
static bool   g_have_new_limits = false;  // guarded by g_mtx
static bool   g_stop_request    = false;  // guarded by g_mtx
static uint8_t g_motor_enable_mask = 0x3f; // bits 0..5: J1..J6, guarded by g_mtx
static bool   g_rehome_request = false;    // guarded by g_mtx
static bool   g_rehome_hold = false;       // owned by the control loop
static uint8_t g_rehome_completion = 0;    // owned by the control loop
static bool   g_sync_request = false;     // guarded by g_mtx

// Cartesian "move" (non-linear): IK a target pose to joint angles, then let the
// joint-space Ruckig get there. Pose is [x y z rx ry rz] in metres/radians
// (base frame). Resolved by the control loop, which owns the Kinematics object.
static double g_move_pose[6]   = {0};   // guarded by g_mtx
static bool   g_move_request   = false; // guarded by g_mtx

// Velocity jogging ("hold the arrow" in the web UI). Non-zero entries put that
// joint under Ruckig's Velocity control interface instead of Position. Each
// non-zero entry carries a deadline; if the UI stops refreshing it (e.g. the
// browser tab dies while a button is "held"), the loop zeroes it -- a
// dead-man's-switch so a lost connection can't leave the arm jogging forever.
static double  g_jog_vel[DOFs]         = {0};    // deg/s, guarded by g_mtx
static int64_t g_jog_deadline_ns[DOFs] = {0};    // guarded by g_mtx
static const int64_t JOG_TIMEOUT_NS = 200'000'000LL; // 200 ms watchdog

// Zeroes all jog velocities/deadlines. Caller must already hold g_mtx.
static void clearJogLocked() {
    for (int i = 0; i < DOFs; i++) { g_jog_vel[i] = 0.0; g_jog_deadline_ns[i] = 0; }
}

// UDP command/telemetry socket (localhost). Commands in = the same text grammar
// as the REPL; state out = a compact JSON line for the Python web UI.
static int         g_udp_fd = -1;
static std::mutex  g_addr_mtx;
static sockaddr_in g_client{};
static bool        g_have_client = false;

static const char* HELP_TEXT =
    "\nCommands:\n"
    "  a1 a2 a3 a4 a5 a6   set all six joint targets (deg)\n"
    "  <j> <angle>         move one joint (j = 1..6), e.g.  6 90\n"
    "  jog <j> <delta>     move one joint by a relative amount (deg)\n"
    "  jogvel <j> <v>      velocity-jog one joint at v deg/s (0 to stop); \n"
    "                      must be refreshed within 200ms or it auto-stops\n"
    "  home                all joints to 0\n"
    "  rehome              run the ESP32 limit-switch homing sequence\n"
    "  stop                decelerate to a stop and hold\n"
    "  sync                align planner to motor feedback without motion\n"
    "  move x y z rx ry rz  IK to a Cartesian pose (m, rad), non-linear path\n"
    "  motor <j|all> <on|off>  enable or disable driver torque\n"
    "  vel  <j|all> <v>    set max velocity (deg/s)\n"
    "  acc  <j|all> <v>    set max acceleration (deg/s^2)\n"
    "  jerk <j|all> <v>    set max jerk (deg/s^3)\n"
    "  limits              show current motion limits\n"
    "  mon                 toggle live position monitor\n"
    "  stats               toggle loop and serial statistics\n"
    "  help                show this help\n"
    "  q / quit            exit\n";

// Parse and apply one command line. Returns a short status string. Shared by the
// stdin REPL and the UDP interface; thread-safe via g_mtx.
static std::string handleCommand(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tok; std::string t;
    while (iss >> t) tok.push_back(t);
    if (tok.empty()) return "";

    if (tok[0] == "ping") return "";                                   // just registers the client
    if (tok[0] == "q" || tok[0] == "quit" || tok[0] == "exit") { g_running = false; return "bye"; }
    if (tok[0] == "help" || tok[0] == "h") return HELP_TEXT;
    if (tok[0] == "mon") { g_monitor = !g_monitor.load(); return g_monitor ? "monitor ON" : "monitor OFF"; }
    if (tok[0] == "stats") { g_stats = !g_stats.load(); return g_stats ? "stats ON" : "stats OFF"; }
    if (tok[0] == "stop") { std::lock_guard<std::mutex> lk(g_mtx); g_stop_request = true; clearJogLocked(); return "stopping"; }
    if (tok[0] == "sync") { std::lock_guard<std::mutex> lk(g_mtx); g_sync_request = true; return "syncing"; }
    if (tok[0] == "move") {
        if (tok.size() != 7) return "usage: move x y z rx ry rz (m, rad)";
        try {
            double p[6];
            for (int i = 0; i < 6; i++) p[i] = std::stod(tok[i + 1]);
            std::lock_guard<std::mutex> lk(g_mtx);
            for (int i = 0; i < 6; i++) g_move_pose[i] = p[i];
            g_move_request = true;
            clearJogLocked();
            return "moving";
        } catch (const std::exception&) { return "bad number"; }
    }
    if (tok[0] == "rehome") {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_motor_enable_mask = 0x3f;
        g_target.fill(0.0);
        g_have_new = true;
        g_rehome_request = true;
        clearJogLocked();
        return "rehoming";
    }

    if (tok[0] == "motor") {
        if (tok.size() != 3 || (tok[2] != "on" && tok[2] != "off")) return "usage: motor <j|all> <on|off>";
        std::lock_guard<std::mutex> lk(g_mtx);
        uint8_t bits = 0;
        if (tok[1] == "all") bits = 0x3f;
        else {
            try {
                int j = std::stoi(tok[1]);
                if (j < 1 || j > DOFs) return "joint must be 1..6";
                bits = (uint8_t)(1u << (j - 1));
            } catch (const std::exception&) { return "joint must be 1..6"; }
        }
        if (tok[2] == "on") g_motor_enable_mask |= bits;
        else g_motor_enable_mask &= (uint8_t)~bits;
        return "motor torque updated";
    }

    if (tok[0] == "limits") {
        std::lock_guard<std::mutex> lk(g_mtx);
        std::ostringstream os; os << "        vel      acc      jerk\n";
        for (int i = 0; i < DOFs; i++)
            os << "  J" << (i + 1) << ":  " << g_max_vel[i] << "     "
               << g_max_acc[i] << "     " << g_max_jerk[i] << "\n";
        return os.str();
    }

    if (tok[0] == "vel" || tok[0] == "acc" || tok[0] == "jerk") {
        if (tok.size() != 3) return "usage: " + tok[0] + " <j|all> <value>";
        double* arr = (tok[0] == "vel") ? g_max_vel : (tok[0] == "acc") ? g_max_acc : g_max_jerk;
        try {
            double val = std::stod(tok[2]);
            if (val <= 0.0) return "value must be > 0";
            std::lock_guard<std::mutex> lk(g_mtx);
            if (tok[1] == "all") { for (int i = 0; i < DOFs; i++) arr[i] = val; }
            else {
                int j = std::stoi(tok[1]);
                if (j < 1 || j > DOFs) return "joint must be 1..6";
                arr[j - 1] = val;
            }
            g_have_new_limits = true;
            return tok[0] + " updated";
        } catch (const std::exception&) { return "bad number"; }
    }

    if (tok[0] == "jog") {
        if (tok.size() != 3) return "usage: jog <j> <delta>";
        try {
            int j = std::stoi(tok[1]); double d = std::stod(tok[2]);
            if (j < 1 || j > DOFs) return "joint must be 1..6";
            std::lock_guard<std::mutex> lk(g_mtx);
            g_target[j - 1] += d; g_have_new = true;
            clearJogLocked();
            return "jog ok";
        } catch (const std::exception&) { return "bad number"; }
    }

    if (tok[0] == "jogvel") {
        if (tok.size() != 3) return "usage: jogvel <j> <deg/s>";
        try {
            int j = std::stoi(tok[1]); double v = std::stod(tok[2]);
            if (j < 1 || j > DOFs) return "joint must be 1..6";
            std::lock_guard<std::mutex> lk(g_mtx);
            double cap = g_max_vel[j - 1];
            if (v > cap) v = cap; else if (v < -cap) v = -cap; // Velocity control ignores max_velocity, so clamp here
            g_jog_vel[j - 1] = v;
            if (v != 0.0) {
                struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
                g_jog_deadline_ns[j - 1] = (int64_t)now.tv_sec * 1000000000LL + now.tv_nsec + JOG_TIMEOUT_NS;
            } else {
                g_jog_deadline_ns[j - 1] = 0;
            }
            return "jogvel ok";
        } catch (const std::exception&) { return "bad number"; }
    }

    try {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (tok[0] == "home") {
            g_target.fill(0.0);
        } else if ((int)tok.size() == DOFs) {              // six numbers -> all joints
            for (int i = 0; i < DOFs; i++) g_target[i] = std::stod(tok[i]);
        } else if (tok.size() == 2) {                      // "<joint> <angle>"
            int j = std::stoi(tok[0]);
            if (j < 1 || j > DOFs) return "joint must be 1..6";
            g_target[j - 1] = std::stod(tok[1]);
        } else {
            return "unrecognised command -- type 'help'";
        }
        g_have_new = true;
        clearJogLocked();
        return "target updated";
    } catch (const std::exception&) { return "could not parse numbers"; }
}

// Background stdin REPL.
static void inputThread() {
    std::cout << HELP_TEXT;
    while (g_running.load()) {
        char* raw = readline("\n> ");
        if (!raw) { g_running = false; break; }  // Ctrl-D
        std::string line(raw);
        free(raw);
        if (!line.empty()) add_history(line.c_str());
        std::string r = handleCommand(line);
        if (!r.empty()) std::cout << r << "\n";
    }
}

// Background UDP command listener (localhost). Records the client address so the
// 500 Hz loop can publish state back to it. Uses a recv timeout for clean exit.
static void udpThread(int udp_port) {
    g_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_fd < 0) { std::cerr << "UDP: socket() failed\n"; return; }

    struct timeval tv{0, 200000}; // 200 ms recv timeout
    setsockopt(g_udp_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)udp_port);
    if (bind(g_udp_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "UDP: bind() failed on port " << udp_port << "\n";
        close(g_udp_fd); g_udp_fd = -1; return;
    }
    std::cout << "UDP command interface on 127.0.0.1:" << udp_port << "\n";

    char buf[1024];
    while (g_running.load()) {
        sockaddr_in cli{}; socklen_t clilen = sizeof(cli);
        ssize_t n = recvfrom(g_udp_fd, buf, sizeof(buf) - 1, 0, (sockaddr*)&cli, &clilen);
        if (n <= 0) continue;                                // timeout or error -> re-check g_running
        buf[n] = '\0';
        { std::lock_guard<std::mutex> lk(g_addr_mtx); g_client = cli; g_have_client = true; }
        handleCommand(std::string(buf, (size_t)n));
    }
    close(g_udp_fd); g_udp_fd = -1;
}

int main(int argc, char** argv) {
    // Serial port can be given as the first CLI arg, else defaults to SERIAL_PORT.
    const char* port = (argc > 1) ? argv[1] : SERIAL_PORT;

    // Kinematics lives entirely on this (control-loop) thread. It is NOT
    // thread-safe -- it holds mutable Pinocchio Data + Jacobian scratch buffers.
    Kinematics kin;

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
    tx_packet.motor_enable_mask = 0x3f;
    tx_packet.flags = FLAG_HOLD; // keep torque on, skip motion until pos received

    // Handshake: retry until we get actual position without commanding any motion.
    bool got_initial_pos = false;
    for (int retry = 0; retry < 30; retry++) {
        serial.writeBytes(txbuf, frameEncode(tx_packet, txbuf));
        if (readFramedPacket(serial, rx_reader, rx_packet, 50)) {
            got_initial_pos = true;
            break;
        }
    }
    if (!got_initial_pos) {
        std::cerr << "No position reply from ESP32 after 30 attempts. Is it running?\n";
        serial.closeDevice();
        return 1;
    }
    tx_packet.motor_enable_mask = 0x3f;
    tx_packet.flags = 0;

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

    // Seed the shared targets with the current position, then start REPL + UDP.
    for (int i = 0; i < DOFs; i++) g_target[i] = input.target_position[i];
    int udp_port = (argc > 2) ? std::atoi(argv[2]) : 5005;
    std::thread repl(inputThread);
    std::thread udp(udpThread, udp_port);

    std::cout << "\nStreaming at 500 Hz. Enter commands below (or use the web UI).\n";

    serial.flushReceiver();

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    // Per-second counters for the optional concise `stats` output.
    struct timespec t_prev = next_tick, t_now;
    long   loop_count = 0, tx_count = 0, rx_count = 0;
    double stats_window_us = 0;
    
    // Enter Main loop 
    while (g_running.load()) {

        // Measure elapsed time to calculate the loop rate once per second.
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double dt_us = (t_now.tv_sec - t_prev.tv_sec) * 1e6 + (t_now.tv_nsec - t_prev.tv_nsec) / 1e3;
        t_prev = t_now;
        loop_count++; stats_window_us += dt_us;
        if (stats_window_us >= 1e6) {
            if (g_stats.load()) {
                std::fprintf(stderr, "[stats] loop %.0f Hz  serial writes %ld/s  ESP replies %ld/s\n",
                    loop_count / (stats_window_us / 1e6), tx_count, rx_count);
            }
            loop_count = 0; tx_count = 0; rx_count = 0; stats_window_us = 0;
        }

        // Apply any freshly entered target(s)/limits/stop from the REPL thread.
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            if (g_have_new) {
                // A real position command always wins over an in-progress jog.
                input.control_interface = ControlInterface::Position;
                input.synchronization   = Synchronization::Time;
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
                input.control_interface = ControlInterface::Position;
                input.synchronization   = Synchronization::Time;
                for (int i = 0; i < DOFs; i++) {
                    input.target_position[i] = input.current_position[i];
                    g_target[i]              = input.current_position[i];
                }
                g_stop_request = false;
            }
            if (g_sync_request) {
                // Adopt motor feedback as the planner state. The ESP re-references
                // its queue on the 0x80 bit, so this produces no motion.
                for (int i = 0; i < DOFs; i++) {
                    input.current_position[i]     = rx_packet.actual_position[i];
                    input.current_velocity[i]     = 0.0;
                    input.current_acceleration[i] = 0.0;
                    input.target_position[i]      = rx_packet.actual_position[i];
                    g_target[i]                   = rx_packet.actual_position[i];
                }
            }
            if (g_move_request) {
                g_move_request = false;
                // deg -> rad for the IK seed (the whole loop works in degrees).
                Eigen::VectorXd q_seed(DOFs);
                for (int i = 0; i < DOFs; i++)
                    q_seed[i] = input.current_position[i] * M_PI / 180.0;

                Eigen::Matrix<double, 6, 1> pose;
                for (int i = 0; i < 6; i++) pose[i] = g_move_pose[i];
                auto ik = kin.InverseKinematics_Positional(Kinematics::poseToSE3(pose), q_seed);

                if (ik.status == 1) {
                    // rad -> deg back into the joint-space Ruckig target.
                    input.control_interface = ControlInterface::Position;
                    input.synchronization   = Synchronization::Time;
                    for (int i = 0; i < DOFs; i++) {
                        double deg = ik.q[i] * 180.0 / M_PI;
                        input.target_position[i] = deg;
                        g_target[i]              = deg;
                    }
                    std::fprintf(stderr, "[move] IK ok in %d iters\n", ik.iters);
                } else {
                    std::fprintf(stderr, "[move] IK failed (status %d) -- holding\n", ik.status);
                }
            }
            if (g_rehome_request) {
                input.control_interface = ControlInterface::Position;
                input.synchronization   = Synchronization::Time;
                for (int i = 0; i < DOFs; i++) {
                    input.current_position[i] = 0.0;
                    input.current_velocity[i] = 0.0;
                    input.current_acceleration[i] = 0.0;
                    input.target_position[i] = 0.0;
                    input.target_velocity[i] = 0.0;
                    input.target_acceleration[i] = 0.0;
                }
                g_rehome_hold = true;
                g_rehome_completion = (uint8_t)(rx_packet.homing_sequence + 1);
            }

            // Velocity jogging ("hold the arrow" in the web UI). Watchdog first: a
            // jog that hasn't been refreshed within JOG_TIMEOUT_NS auto-stops.
            struct timespec jog_now; clock_gettime(CLOCK_MONOTONIC, &jog_now);
            int64_t jog_now_ns = (int64_t)jog_now.tv_sec * 1000000000LL + jog_now.tv_nsec;
            bool any_jog = false;
            for (int i = 0; i < DOFs; i++) {
                if (g_jog_vel[i] != 0.0 && jog_now_ns > g_jog_deadline_ns[i]) g_jog_vel[i] = 0.0;
                if (g_jog_vel[i] != 0.0) any_jog = true;
            }
            // Once a jog starts we stay in Velocity control -- even after every
            // g_jog_vel hits 0 -- so releasing the arrow just decelerates to a
            // stop in place. Snapping back to a Position hold here would freeze
            // a target at the still-moving joint's position, forcing Ruckig to
            // plan a fresh position move from nonzero velocity, which overshoots
            // and corrects backward -- exactly the "bounces back" symptom. We
            // only leave Velocity control from an explicit position command above.
            if (any_jog || input.control_interface == ControlInterface::Velocity) {
                input.control_interface = ControlInterface::Velocity;
                input.synchronization   = Synchronization::None;
                for (int i = 0; i < DOFs; i++) {
                    input.target_velocity[i]     = g_jog_vel[i];
                    input.target_acceleration[i] = 0.0;
                }
                for (int i = 0; i < DOFs; i++) g_target[i] = input.current_position[i]; // keep the UI's target readout tracking live position
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
                        tx_packet.pos_cmd[i] = g_rehome_hold ? 0.0f : output.new_position[i];
                        tx_packet.vel_cmd[i] = g_rehome_hold ? 0.0f : output.new_velocity[i];
        }
                { std::lock_guard<std::mutex> lk(g_mtx);
                    tx_packet.motor_enable_mask = g_motor_enable_mask
                        | (g_rehome_request ? FLAG_REHOME : 0)
                        | (g_sync_request   ? FLAG_SYNC   : 0);
                    g_rehome_request = false;
                    g_sync_request = false; }
        output.pass_to_input(input);

        // 2. Send Velocity Command (COBS-framed + CRC16)
        serial.writeBytes(txbuf, frameEncode(tx_packet, txbuf));
        tx_count++;

        // 3. Wait for a valid framed reply (bounded to the ~2ms cycle budget)
        if (readFramedPacket(serial, rx_reader, rx_packet, 1)) {
            rx_count++;
            if (g_rehome_hold && rx_packet.homing_sequence == g_rehome_completion) {
                g_rehome_hold = false;
                std::cout << "Rehome complete\n";
            }
        }

        // 4. Optional live telemetry at ~5 Hz. Toggle with 'mon'.
        static int telem = 0;
        if (g_monitor.load() && (++telem % 100 == 0)) {
            const float* a = rx_packet.actual_position;
            const float* c = tx_packet.pos_cmd;
            const double* tg = input.target_position.data();
            std::fprintf(stdout,
                "[mon] act  J1:%7.2f  J2:%7.2f  J3:%7.2f  J4:%7.2f  J5:%7.2f  J6:%7.2f\n"
                "      cmd  J1:%7.2f  J2:%7.2f  J3:%7.2f  J4:%7.2f  J5:%7.2f  J6:%7.2f\n"
                "      tgt  J1:%7.2f  J2:%7.2f  J3:%7.2f  J4:%7.2f  J5:%7.2f  J6:%7.2f\n"
                "      err  J1:%7.2f  J2:%7.2f  J3:%7.2f  J4:%7.2f  J5:%7.2f  J6:%7.2f\n",
                a[0],a[1],a[2],a[3],a[4],a[5],
                c[0],c[1],c[2],c[3],c[4],c[5],
                tg[0],tg[1],tg[2],tg[3],tg[4],tg[5],
                a[0]-c[0], a[1]-c[1], a[2]-c[2],
                a[3]-c[3], a[4]-c[4], a[5]-c[5]);
            std::fflush(stdout);
        }

        // Publish state to the web UI over UDP at ~30 Hz.
        static int pub = 0;
        if (g_udp_fd >= 0 && (++pub % 16 == 0)) {
            bool have; sockaddr_in cli;
            { std::lock_guard<std::mutex> lk(g_addr_mtx); have = g_have_client; cli = g_client; }
            if (have) {
                                double tg[DOFs], vm[DOFs], am[DOFs], jm[DOFs]; uint8_t enabled_mask;
                { std::lock_guard<std::mutex> lk(g_mtx);
                                    for (int i = 0; i < DOFs; i++) { tg[i]=g_target[i]; vm[i]=g_max_vel[i]; am[i]=g_max_acc[i]; jm[i]=g_max_jerk[i]; }
                                    enabled_mask = g_motor_enable_mask; }
                char sb[640];
                int len = snprintf(sb, sizeof(sb),
                    "{\"pos\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                    "\"tgt\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                    "\"vmax\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                    "\"amax\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                    "\"jmax\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                    "\"enabled\":[%d,%d,%d,%d,%d,%d]}",
                    rx_packet.actual_position[0], rx_packet.actual_position[1], rx_packet.actual_position[2],
                    rx_packet.actual_position[3], rx_packet.actual_position[4], rx_packet.actual_position[5],
                    tg[0],tg[1],tg[2],tg[3],tg[4],tg[5],
                    vm[0],vm[1],vm[2],vm[3],vm[4],vm[5],
                    am[0],am[1],am[2],am[3],am[4],am[5],
                    jm[0],jm[1],jm[2],jm[3],jm[4],jm[5],
                    (enabled_mask & 0x01) != 0, (enabled_mask & 0x02) != 0,
                    (enabled_mask & 0x04) != 0, (enabled_mask & 0x08) != 0,
                    (enabled_mask & 0x10) != 0, (enabled_mask & 0x20) != 0);
                if (len > 0) sendto(g_udp_fd, sb, (size_t)len, 0, (sockaddr*)&cli, sizeof(cli));
            }
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
    if (udp.joinable())  udp.join();     // exits within the recv timeout
    if (repl.joinable()) repl.detach();  // stdin getline can't be unblocked; let it go
    serial.closeDevice();
    return 0;
}