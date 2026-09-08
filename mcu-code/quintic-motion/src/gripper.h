#pragma once

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>
#include <stdint.h>

#include "structs.h"

// Parallel gripper lives on a second ESP32 reached over ESP-NOW.
//
// Everything WiFi-related runs on core 0, in its own low-priority task --
// including the driver init, because ESP-IDF allocates the WiFi interrupts on
// whichever core starts it. Core 1 is left to the step generator and the 500 Hz
// serial link; sharing a core with the radio shows up as audible stepper whine.
//
// The Pi streams command packets at 500 Hz, but the gripper only needs a frame
// when its target actually changes, so the packet handler records and this task
// transmits.
namespace Gripper {

// Slave's burned-in MAC. Read it from the slave with ESP.getEfuseMac().
static const uint8_t SLAVE_MAC[6] = {0x78, 0x1C, 0x3C, 0xE1, 0x08, 0x1C};

static const uint32_t RETRY_MS = 25;   // also rate-caps bursts of slider drags

struct Msg { uint8_t gripper_pos; };   // must match the slave's struct_message

inline uint8_t          applied = 0;          // last value the slave acked
inline uint8_t          pending = 0;
inline volatile bool    have_pending = false; // set from the WiFi task on failure
inline volatile bool    radio_wanted = true;  // driven by FLAG_RADIO_OFF
inline volatile bool    radio_on = false;
inline uint32_t         last_attempt_ms = 0;

inline void onSent(const uint8_t*, esp_now_send_status_t status) {
    // Runs in the WiFi task. A failed frame just re-arms the pending flag so
    // the next pass sends it again; `pending` still holds the value.
    if (status != ESP_NOW_SEND_SUCCESS) have_pending = true;
}

// ESP-NOW state is lost whenever the WiFi driver stops, so the peer is always
// (re-)registered here rather than once at boot.
inline bool radioUp() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, SLAVE_MAC, 6);
    peer.channel = 0;        // stay on the current channel, like the slave
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return false;

    radio_on = true;
    return true;
}

inline void radioDown() {
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
    radio_on = false;
}

// Called from the serial packet handler on core 1 -- records only, never
// touches the radio.
inline void setTarget(uint8_t pos) {
    if (pos > GRIPPER_POS_MAX) pos = GRIPPER_POS_MAX;
    if (pos == applied && !have_pending) return;
    pending = pos;
    have_pending = true;
}

inline void taskFn(void*) {
    for (;;) {
        if (radio_wanted != radio_on) {
            if (radio_wanted) radioUp(); else radioDown();
        }
        if (radio_on && have_pending && (millis() - last_attempt_ms >= RETRY_MS)) {
            last_attempt_ms = millis();
            Msg m{pending};
            have_pending = false;               // onSent() re-arms this on failure
            if (esp_now_send(SLAVE_MAC, (uint8_t*)&m, sizeof(m)) == ESP_OK) applied = pending;
            else have_pending = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

inline void start() {
    xTaskCreatePinnedToCore(taskFn, "gripper", 3072, nullptr, 2, nullptr, 0);
}

} // namespace Gripper
