#pragma once
// ---------------------------------------------------------------------------
// Shared Pi <-> ESP32 serial framing protocol.
//
// Frame on the wire:   [ payload bytes | crc16(2, little-endian) ]  --COBS-->  [ ...  0x00 ]
//
// COBS guarantees 0x00 only appears as the frame delimiter, so a receiver can
// always re-synchronise after a dropped/extra/corrupt byte simply by scanning
// to the next 0x00. CRC16-CCITT rejects any frame whose bytes were corrupted.
//
// IMPORTANT: this file MUST be byte-for-byte identical on both the ESP32 and
// the Raspberry Pi builds. Keep the two copies in sync.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#pragma pack(push, 1) // Force exact byte alignment across both platforms
struct EspToPiPacket {
    float actual_position[6]; // 24 bytes
};

struct PiToEspPacket {
    float pos_cmd[6];         // Absolute target position, deg (24 bytes)
    float vel_cmd[6];         // Commanded velocity, deg/s (24 bytes)
};
#pragma pack(pop)

// ---------------------------------------------------------------------------
// CRC16-CCITT (poly 0x1021, init 0xFFFF)
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// COBS encode/decode (Consistent Overhead Byte Stuffing).
// Reference implementation by Jacques Fortier (public domain).
// ---------------------------------------------------------------------------
inline size_t cobsEncode(const void* data, size_t length, uint8_t* buffer) {
    uint8_t* encode = buffer;   // Encoded byte pointer
    uint8_t* codep  = encode++; // Output code pointer
    uint8_t  code   = 1;        // Code value

    for (const uint8_t* byte = (const uint8_t*)data; length--; ++byte) {
        if (*byte) {            // Byte not zero, write it
            *encode++ = *byte;
            ++code;
        }
        if (!*byte || code == 0xFF) { // Zero byte or full block, restart
            *codep = code;
            code = 1;
            codep = encode;
            if (!*byte || length) ++encode;
        }
    }
    *codep = code; // Write final code value
    return (size_t)(encode - buffer);
}

// Returns number of decoded bytes.
inline size_t cobsDecode(const uint8_t* buffer, size_t length, void* data) {
    const uint8_t* byte   = buffer;         // Encoded input pointer
    uint8_t*       decode = (uint8_t*)data; // Decoded output pointer

    for (uint8_t code = 0xFF, block = 0; byte < buffer + length; --block) {
        if (block) {                        // Decode block byte
            *decode++ = *byte++;
        } else {
            block = *byte++;                // Fetch next block length
            if (block && (code != 0xFF)) {  // Encoded zero, write it
                *decode++ = 0;
            }
            code = block;
            if (!code) break;               // Delimiter code found
        }
    }
    return (size_t)(decode - (uint8_t*)data);
}

// Worst-case encoded size (payload + crc), including COBS overhead and delimiter.
template <typename T>
constexpr size_t frameMaxLen() {
    return (sizeof(T) + 2) + ((sizeof(T) + 2) / 254) + 2;
}

// ---------------------------------------------------------------------------
// Encode a packet into a full COBS frame terminated by 0x00.
// `out` must be at least frameMaxLen<T>() bytes. Returns bytes written.
// ---------------------------------------------------------------------------
template <typename T>
size_t frameEncode(const T& pkt, uint8_t* out) {
    uint8_t payload[sizeof(T) + 2];
    memcpy(payload, &pkt, sizeof(T));
    uint16_t crc = crc16_ccitt(payload, sizeof(T));
    payload[sizeof(T)]     = (uint8_t)(crc & 0xFF);
    payload[sizeof(T) + 1] = (uint8_t)(crc >> 8);

    size_t n = cobsEncode(payload, sizeof(T) + 2, out);
    out[n] = 0x00; // frame delimiter
    return n + 1;
}

// ---------------------------------------------------------------------------
// Stateful, byte-at-a-time frame receiver. Feed incoming bytes with push();
// it returns true and fills `out` exactly when a CRC-valid frame completes.
// Recovers automatically from desync (waits for the next 0x00 delimiter).
// ---------------------------------------------------------------------------
template <typename T>
class FrameReader {
public:
    bool push(uint8_t b, T& out) {
        if (b == 0x00) {                 // End of frame
            bool ok = decodeCurrent(out);
            idx_ = 0;
            return ok;
        }
        if (idx_ < sizeof(buf_)) {
            buf_[idx_++] = b;
        } else {
            idx_ = 0;                    // Overflow -> resync on next delimiter
        }
        return false;
    }

    void reset() { idx_ = 0; }

private:
    bool decodeCurrent(T& out) {
        if (idx_ == 0) return false;
        uint8_t decoded[sizeof(T) + 2];
        size_t n = cobsDecode(buf_, idx_, decoded);
        if (n != sizeof(T) + 2) return false;

        uint16_t crc = crc16_ccitt(decoded, sizeof(T));
        uint16_t rx  = (uint16_t)decoded[sizeof(T)] |
                       ((uint16_t)decoded[sizeof(T) + 1] << 8);
        if (crc != rx) return false;

        memcpy(&out, decoded, sizeof(T));
        return true;
    }

    // Capacity = worst-case COBS-encoded length of (payload + crc).
    static const size_t CAP = frameMaxLen<T>();
    uint8_t buf_[CAP];
    size_t  idx_ = 0;
};
