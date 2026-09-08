#pragma once

#include <esp_now.h>
#include <WiFi.h>
#include <string.h>
#include <stdint.h>

#include "structs.h"

// Parallel gripper lives on a second ESP32 reached over ESP-NOW.
//
// The Pi streams command packets at 500 Hz, but the gripper only ever needs a
// frame when its target actually changes -- transmitting every cycle would put
// the WiFi radio in contention with the step generator for no benefit. So the
// serial packet handler only records a target, and loop() does the (rare) send.
namespace Gripper {

// Slave's burned-in MAC. Read it from the slave with ESP.getEfuseMac().
static const uint8_t SLAVE_MAC[6] = {0x78, 0x1C, 0x3C, 0xE1, 0x08, 0x1C};

static const uint32_t RETRY_MS = 25;   // also rate-caps bursts of slider drags

struct Msg { uint8_t gripper_pos; };   // must match the slave's struct_message

inline uint8_t          applied = 0;          // last value the slave acked
inline uint8_t          pending = 0;
inline volatile bool    have_pending = false; // set from the WiFi task on failure
inline uint32_t         last_attempt_ms = 0;
inline bool             ready = false;

inline void onSent(const uint8_t*, esp_now_send_status_t status) {
    // Runs in the WiFi task. A failed frame just re-arms the pending flag so
    // service() sends it again; `pending` still holds the value.
    if (status != ESP_NOW_SEND_SUCCESS) have_pending = true;
}

inline bool begin() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, SLAVE_MAC, 6);
    peer.channel = 0;        // stay on the current channel, like the slave
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return false;

    ready = true;
    return true;
}

// Called from the serial packet handler -- records only, never transmits.
inline void setTarget(uint8_t pos) {
    if (pos > GRIPPER_POS_MAX) pos = GRIPPER_POS_MAX;
    if (pos == applied && !have_pending) return;
    pending = pos;
    have_pending = true;
}

// Called from loop(). Sends at most one frame per RETRY_MS.
inline void service() {
    if (!ready || !have_pending) return;
    uint32_t now = millis();
    if (now - last_attempt_ms < RETRY_MS) return;
    last_attempt_ms = now;

    Msg m{pending};
    have_pending = false;                       // onSent() re-arms this if it fails
    if (esp_now_send(SLAVE_MAC, (uint8_t*)&m, sizeof(m)) == ESP_OK) applied = pending;
    else have_pending = true;
}

} // namespace Gripper
