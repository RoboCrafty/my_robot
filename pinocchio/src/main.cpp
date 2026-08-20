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
#include <cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>

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
    "  home                all joints to 0\n"
    "  stop                decelerate to a stop and hold\n"
    "  vel  <j|all> <v>    set max velocity (deg/s)\n"
    "  acc  <j|all> <v>    set max acceleration (deg/s^2)\n"
    "  jerk <j|all> <v>    set max jerk (deg/s^3)\n"
    "  limits              show current motion limits\n"
    "  mon                 toggle live position monitor\n"
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
    if (tok[0] == "stop") { std::lock_guard<std::mutex> lk(g_mtx); g_stop_request = true; return "stopping"; }

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
            return "jog ok";
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
        return "target updated";
    } catch (const std::exception&) { return "could not parse numbers"; }
}

// Background stdin REPL.
static void inputThread() {
    std::cout << HELP_TEXT;
    std::string line;
    while (g_running.load()) {
        std::cout << "\n> " << std::flush;
        if (!std::getline(std::cin, line)) { g_running = false; break; }  // Ctrl-D
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

    // Seed the shared targets with the current position, then start REPL + UDP.
    for (int i = 0; i < DOFs; i++) g_target[i] = input.target_position[i];
    int udp_port = (argc > 2) ? std::atoi(argv[2]) : 5005;
    std::thread repl(inputThread);
    std::thread udp(udpThread, udp_port);

    std::cout << "\nStreaming at 500 Hz. Enter commands below (or use the web UI).\n";

    serial.flushReceiver();

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    // Loop-rate diagnostic: measures the ACTUAL loop cadence vs the target 500 Hz.
    struct timespec t_prev = next_tick, t_now;
    long   rate_count = 0, rate_over = 0, rx_ok = 0;
    double rate_sum_us = 0, rate_max_us = 0, rate_win_us = 0;

    while (g_running.load()) {

        // Measure the real wall-clock time since the previous iteration.
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double dt_us = (t_now.tv_sec - t_prev.tv_sec) * 1e6 + (t_now.tv_nsec - t_prev.tv_nsec) / 1e3;
        t_prev = t_now;
        rate_count++; rate_sum_us += dt_us; rate_win_us += dt_us;
        if (dt_us > rate_max_us) rate_max_us = dt_us;
        if (dt_us > 2500.0) rate_over++;                       // missed the 2 ms budget
        if (rate_win_us >= 1e6) {                              // report ~once per second
            std::fprintf(stderr,
                "[loop] %.0f Hz  mean %.2f ms  max %.2f ms  overruns %ld/%ld  rx-replies %ld/s\n",
                rate_count / (rate_win_us / 1e6), rate_sum_us / rate_count / 1000.0,
                rate_max_us / 1000.0, rate_over, rate_count, rx_ok);
            rate_count = 0; rate_sum_us = 0; rate_max_us = 0; rate_over = 0; rate_win_us = 0; rx_ok = 0;
        }

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
        if (readFramedPacket(serial, rx_reader, rx_packet, 1)) rx_ok++;  // count valid round-trips/s

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

        // Publish state to the web UI over UDP at ~30 Hz.
        static int pub = 0;
        if (g_udp_fd >= 0 && (++pub % 16 == 0)) {
            bool have; sockaddr_in cli;
            { std::lock_guard<std::mutex> lk(g_addr_mtx); have = g_have_client; cli = g_client; }
            if (have) {
                double tg[DOFs], vm[DOFs], am[DOFs], jm[DOFs];
                { std::lock_guard<std::mutex> lk(g_mtx);
                  for (int i = 0; i < DOFs; i++) { tg[i]=g_target[i]; vm[i]=g_max_vel[i]; am[i]=g_max_acc[i]; jm[i]=g_max_jerk[i]; } }
                char sb[640];
                int len = snprintf(sb, sizeof(sb),
                    "{\"pos\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                    "\"tgt\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                    "\"vmax\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                    "\"amax\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                    "\"jmax\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}",
                    rx_packet.actual_position[0], rx_packet.actual_position[1], rx_packet.actual_position[2],
                    rx_packet.actual_position[3], rx_packet.actual_position[4], rx_packet.actual_position[5],
                    tg[0],tg[1],tg[2],tg[3],tg[4],tg[5],
                    vm[0],vm[1],vm[2],vm[3],vm[4],vm[5],
                    am[0],am[1],am[2],am[3],am[4],am[5],
                    jm[0],jm[1],jm[2],jm[3],jm[4],jm[5]);
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