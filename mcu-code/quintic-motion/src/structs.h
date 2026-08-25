#pragma once

#include <stdint.h>
#include <stddef.h>

// --- Pi <-> ESP32 packet layout (MUST match the Raspberry Pi protocol.h) ---
#pragma pack(push, 1) // exact byte alignment across both platforms
struct EspToPiPacket {
    float actual_position[6]; // 24 bytes
    uint8_t homing_sequence;  // Increments after each completed rehome
};

struct PiToEspPacket {
    float pos_cmd[6];         // Absolute target position, deg (24 bytes)
    float vel_cmd[6];         // Commanded velocity, deg/s (24 bytes)
    uint8_t motor_enable_mask;// Bits 0..5: J1..J6 enabled; bit 6: rehome request
};
#pragma pack(pop)

// CRC16-CCITT (poly 0x1021, init 0xFFFF). PacketSerial does the COBS framing;
// this verifies the payload inside a frame wasn't corrupted on the wire.
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

struct Joints {
    float q1, q2, q3, q4, q5, q6;
};
struct JointFrame {
    int32_t q_steps[6];   
};
struct Pose {
    float x, y, z, rx, ry, rz;
};

struct TrigValues {
    float s1, s2, s3, s4, s5, s6;
    float c1, c2, c3, c4, c5, c6;
};

// Toggle this: 1 to enable debug prints, 0 to disable completely
#define DEBUG_MODE 0

#if DEBUG_MODE
  #define DEBUG_PRINT(x)         Serial.print(x)
  #define DEBUG_PRINTLN(x)       Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  // If debug is off, these macros are replaced with absolutely nothing
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif