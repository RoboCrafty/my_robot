#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

#pragma pack(push, 1) // Force exact byte alignment
struct EspToPiPacket {
    int32_t actual_position[6]; // 24 bytes
};

struct PiToEspPacket {
    float v_cmd[6];             // 24 bytes
};
#pragma pack(pop)
int main() {
    // 1. Open High-Speed Serial Port (e.g., /dev/ttyAMA0 or /dev/ttyUSB0)
    int serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B1000000); // 1 Mbps baud rate
    cfsetospeed(&options, B1000000);
    options.c_cflag = CS8 | CREAD | CLOCAL; // 8n1
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0; // Raw mode
    tcsetattr(serial_fd, TCSANOW, &options);

    PiToEspPacket tx_packet = {0};
    EspToPiPacket rx_packet = {0};

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    while (true) {
        // 2. Send Velocity Command
        write(serial_fd, &tx_packet, sizeof(tx_packet));

        // 3. Wait for ESP32 Reply (Blocking read or polling)
        int bytes_read = 0;
        while (bytes_read < sizeof(rx_packet)) {
            int result = read(serial_fd, ((uint8_t*)&rx_packet) + bytes_read, sizeof(rx_packet) - bytes_read);
            if (result > 0) bytes_read += result;
        }

        // 4. Process rx_packet here...
        // ...

        // 5. Sleep exactly until the next 2ms interval (500 Hz)
        next_tick.tv_nsec += 2000000; // Add 2ms
        if (next_tick.tv_nsec >= 1000000000) {
            next_tick.tv_nsec -= 1000000000;
            next_tick.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, NULL);
    }

    close(serial_fd);
    return 0;
}