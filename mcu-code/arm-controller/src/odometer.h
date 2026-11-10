#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "structs.h"
#include "motor_init.h"
#include "power_monitor.h"

// Lifetime counters (power-on hours, motion hours, per-joint travel...), kept
// in the ESP32's NVS flash so they stay with the robot, not with whichever
// computer happens to drive it. Runs entirely on the loop task (core 1).
//
// A flash write stalls both cores for a few ms (longer if NVS has to erase a
// page), which would show up as a hiccup in the step stream. So writes only
// happen once no joint has stepped for IDLE_BEFORE_SAVE_MS, at most every
// SAVE_PERIOD_MS -- or as soon as idle after motor power drops. Worst case an
// unexpected ESP32 reset loses the last couple of minutes. NVS wear-levels;
// at this rate the flash outlives the robot by decades.
namespace Odometer {

static const uint32_t SAVE_PERIOD_MS      = 120000;
static const uint32_t IDLE_BEFORE_SAVE_MS = 2000;
static const uint8_t  MAX_STATS           = 32;   // on-flash capacity; STAT_COUNT may grow up to this
static_assert(STAT_COUNT <= MAX_STATS, "grow MAX_STATS");

inline Preferences prefs;
inline uint32_t v[STAT_COUNT] = {0};
inline bool     dirty = false, save_soon = false;
inline uint32_t last_update_ms = 0, last_save_ms = 0, last_motion_ms = 0;
inline uint32_t power_ms = 0, servo_ms = 0, motion_ms = 0;   // sub-second remainders
inline float    travel_rem[6] = {0};                           // sub-degree remainders, steps
inline int32_t  last_pos[6] = {0};
inline float    steps_per_deg[6] = {1, 1, 1, 1, 1, 1};
inline uint8_t  prev_ready = 0;
inline uint32_t prev_faults = 0;
inline uint8_t  tx_id = 0;

// Restore staging (see ODO_*). RAM only: a restore cut short is just dropped.
inline uint32_t pending[STAT_COUNTERS] = {0};
inline bool     staging = false;
inline uint8_t  last_seq = 0;            // 0 = nothing applied since boot; the host uses 1..255

inline void save(uint32_t now) {
    prefs.putBytes("v", v, sizeof(v));
    dirty = false;
    save_soon = false;
    last_save_ms = now;
}

// Stored as a bare uint32 array: counters appended to the STAT_* enum later
// simply start at zero while the existing ones keep their values.
inline void begin(const float* spd) {
    for (int i = 0; i < 6; i++) steps_per_deg[i] = spd[i];
    prefs.begin("odometer", false);
    uint32_t buf[MAX_STATS] = {0};
    size_t len = prefs.getBytesLength("v");
    if (len > 0 && len <= sizeof(buf)) prefs.getBytes("v", buf, len);
    memcpy(v, buf, sizeof(v));

    v[STAT_BOOTS]++;
    // Identifies THIS copy of the data. Generated only when NVS had none, so a
    // replacement or erased board always reports a new id and the host knows to
    // restore its saved totals. 0 and 1 are reserved for the host's records.
    while (v[STAT_STORE_ID] < 2) v[STAT_STORE_ID] = esp_random();
    prev_ready  = Power::ready();
    prev_faults = Power::fault_events;
    last_update_ms = last_motion_ms = millis();
    save(last_update_ms);                 // nothing is moving yet during setup()
}

// Call after anything that rewrites the step counters (homing re-zeroes them),
// so the jump is not counted as travel.
inline void resyncPositions() {
    for (int i = 0; i < 6; i++) last_pos[i] = steppers[i]->getCurrentPosition();
}

inline void onHomed() { v[STAT_HOMINGS]++; dirty = true; }

inline void rollSeconds(uint32_t& ms, uint8_t id) {
    if (ms >= 1000) { v[id] += ms / 1000; ms %= 1000; dirty = true; }
}

// Called every ~100 ms from loop().
inline void update(uint32_t now) {
    uint32_t dt = now - last_update_ms;
    last_update_ms = now;

    const uint8_t ready = Power::ready();
    if (ready && !prev_ready) { v[STAT_POWER_CYCLES]++; dirty = true; }
    if (!ready && prev_ready) save_soon = true;
    prev_ready = ready;

    const uint32_t faults = Power::fault_events;
    if (faults != prev_faults) { v[STAT_DRIVER_FAULTS] += faults - prev_faults; prev_faults = faults; dirty = true; }

    bool moving = false;
    for (int i = 0; i < 6; i++) {
        int32_t p = steppers[i]->getCurrentPosition();
        int32_t d = p - last_pos[i];
        last_pos[i] = p;
        if (d == 0 || !(ready & (1u << i))) continue;   // steps into an unpowered driver go nowhere
        moving = true;
        travel_rem[i] += (float)(d < 0 ? -d : d);
        if (travel_rem[i] >= steps_per_deg[i]) {
            uint32_t whole = (uint32_t)(travel_rem[i] / steps_per_deg[i]);
            v[STAT_TRAVEL_J1 + i] += whole;
            travel_rem[i] -= whole * steps_per_deg[i];
            dirty = true;
        }
    }

    if (ready) power_ms += dt;
    if (ready & Power::applied_enable_mask) servo_ms += dt;
    if (moving) { motion_ms += dt; last_motion_ms = now; }
    rollSeconds(power_ms,  STAT_POWER_S);
    rollSeconds(servo_ms,  STAT_SERVO_S);
    rollSeconds(motion_ms, STAT_MOTION_S);

    const bool idle = (now - last_motion_ms) >= IDLE_BEFORE_SAVE_MS;
    if (dirty && idle && (save_soon || now - last_save_ms >= SAVE_PERIOD_MS)) save(now);
}

// One counter per reply; the host reassembles the full set.
inline void fillPacket(EspToPiPacket& p) {
    p.stat_id = tx_id;
    p.stat_value = v[tx_id];
    tx_id = (uint8_t)((tx_id + 1) % STAT_COUNT);
    p.odo_ack = last_seq;
}

// The host repeats each op until it sees odo_ack == odo_seq, so an op is
// applied only when the seq changes. Called from the packet handler (loop task).
inline void handleOp(const PiToEspPacket& p) {
    if (p.odo_op == ODO_NONE || p.odo_seq == last_seq) return;
    last_seq = p.odo_seq;
    switch (p.odo_op) {
        case ODO_BEGIN:
            memset(pending, 0, sizeof(pending));
            staging = true;
            break;
        case ODO_ADD:
            if (staging && p.odo_id < STAT_COUNTERS) pending[p.odo_id] += p.odo_value;
            break;
        case ODO_COMMIT:
            if (!staging) break;
            for (int i = 0; i < STAT_COUNTERS; i++) v[i] += pending[i];
            v[STAT_RESTORED_FROM] = p.odo_value;
            staging = false;
            // Saved right away despite the brief flash stall: this happens once,
            // while the host is just starting up and the arm is holding still.
            save(millis());
            break;
    }
}

} // namespace Odometer
