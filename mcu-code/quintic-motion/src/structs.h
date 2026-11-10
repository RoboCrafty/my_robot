#pragma once

#include <stdint.h>
#include <stddef.h>

// --- Pi <-> ESP32 packet layout (MUST match the Raspberry Pi protocol.h) ---
#pragma pack(push, 1) // exact byte alignment across both platforms
struct EspToPiPacket {
    float actual_position[6]; // 24 bytes
    uint8_t homing_sequence;  // Increments after each completed rehome
    uint8_t gripper_pos;      // Last value the ESP-NOW link actually delivered
    uint32_t underruns;       // Cumulative step-queue-empty events (motion roughness)
    uint8_t driver_ok_mask;   // Bit i: Ji's TMC driver answers on UART and is configured (motor power present)
    uint8_t homed_mask;       // Bit i: Ji homed since its driver last powered up; cleared by any power loss
    uint8_t stat_id;          // Which odometer counter stat_value carries (STAT_*); rotates every reply
    uint32_t stat_value;      // Value of counter stat_id (persisted in the ESP32's NVS flash)
    uint8_t odo_ack;          // odo_seq of the last odometer op applied (see ODO_*)
    uint8_t uart_fail_mask;   // Bit i: Ji's most recent driver poll got no valid UART reply
    uint8_t kept_power_mask;  // Bit i: Ji last came back WITHOUT a chip reset -> it never lost
                              // power; that outage was UART-only (wiring, address pins)
};

struct PiToEspPacket {
    float pos_cmd[6];         // Absolute target position, deg (24 bytes)
    float vel_cmd[6];         // Commanded velocity, deg/s (24 bytes)
    uint8_t motor_enable_mask;// Bits 0..5: J1..J6 enabled; bit 6: rehome; bit 7: sync
    uint8_t flags;            // FLAG_HOLD: report position, skip motion
    uint8_t gripper_pos;      // Servo angle 0..140; forwarded over ESP-NOW on change
    uint8_t odo_op;           // ODO_*: odometer restore step, applied once per odo_seq change
    uint8_t odo_seq;          // 1..255; the ESP echoes it in odo_ack once applied
    uint8_t odo_id;           // STAT_* counter for ODO_ADD
    uint32_t odo_value;       // amount for ODO_ADD; previous store id for ODO_COMMIT
};
#pragma pack(pop)

static const uint8_t GRIPPER_POS_MAX = 140;

static const uint8_t FLAG_HOLD   = 0x01;
static const uint8_t FLAG_RADIO_OFF = 0x02;  // stop the WiFi/ESP-NOW radio
// Bits 2-4 of `flags`: which joint a rehome targets (0 = all six, existing
// behaviour; 1..6 = that joint only). Packed into the spare flag bits instead
// of growing the packet, since both copies of this struct must stay identical.
static const uint8_t REHOME_JOINT_SHIFT = 2;
static const uint8_t REHOME_JOINT_MASK  = 0x1C;
static const uint8_t FLAG_REHOME = 0x40;
static const uint8_t FLAG_SYNC   = 0x80;

// Odometer counters multiplexed through EspToPiPacket::stat_id/stat_value, one
// per reply, so the whole set refreshes every STAT_COUNT packets (~30 ms).
enum : uint8_t {
    STAT_POWER_S = 0,      // seconds with motor power present (any driver answering)
    STAT_SERVO_S,          // seconds with power present AND at least one motor energized
    STAT_MOTION_S,         // seconds during which any joint was stepping
    STAT_POWER_CYCLES,     // motor-power on transitions seen while the ESP32 was running
    STAT_HOMINGS,          // completed homing runs (all-joint or single-joint)
    STAT_BOOTS,            // ESP32 boots
    STAT_DRIVER_FAULTS,    // TMC drv_err events (over-temperature / short shutdown)
    STAT_TRAVEL_J1,        // whole degrees travelled by J1 (J2..J6 follow consecutively)
    STAT_COUNTERS = STAT_TRAVEL_J1 + 6,  // the real counters end here
    STAT_STORE_ID = STAT_COUNTERS,       // random id made when the ESP's NVS was empty: a new
                                         // or erased board shows up with a different one
    STAT_RESTORED_FROM,                  // store id whose totals were added in by a restore
    STAT_COUNT
};

// Odometer restore (e.g. after replacing a dead ESP32): the host stages its
// last saved totals with ODO_BEGIN + ODO_ADDs, then ODO_COMMIT adds them all
// at once, records the old store id in STAT_RESTORED_FROM and saves -- so a
// restore cut short is simply dropped, and a finished one can't be repeated.
enum : uint8_t { ODO_NONE = 0, ODO_BEGIN, ODO_ADD, ODO_COMMIT };

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