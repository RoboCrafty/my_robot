#pragma once

#include <Arduino.h>
#include "motor_init.h"

// Motor-power detection via the TMC2209 UART.
//
// The drivers' logic runs off VM (the motor supply), not the ESP32's 3.3 V, so
// when main power is unplugged they stop answering on UART -- the same thing
// test_connection() reports as "no power". When VM comes back the driver has
// reset to its OTP defaults (different microstepping, analog current), and the
// joints may have dropped while unpowered, so a returning driver is fully
// reconfigured and its joint is marked un-homed.
//
// A failed UART read blocks for ~18 ms (library retries + timeouts), so polling
// runs in its own low-priority task on core 0 -- never in the 500 Hz loop on
// core 1. One driver per POLL_MS, i.e. each driver every 6 * POLL_MS.
namespace Power {

static const uint32_t POLL_MS    = 50;
static const uint8_t  MISS_LIMIT = 2;     // consecutive failed reads before a driver counts as unpowered
// A returning driver must answer continuously this long before it is
// reconfigured: a USB-C PD supply starts at 5 V (enough for the TMC to answer)
// and needs ~2 s to negotiate up to 20 V.
static const uint32_t SETTLE_MS  = 2000;

static const uint8_t GSTAT_RESET   = 0x01;
static const uint8_t GSTAT_DRV_ERR = 0x02;

inline portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Bit i = joint i+1. Written from both cores, so always go through the helpers.
inline volatile uint8_t ready_mask = 0;          // answering + configured: steps may be generated
inline volatile uint8_t homed_mask = 0;          // homed since the driver last powered up
inline volatile uint8_t lost_mask  = 0;          // newly lost, not yet handled by the loop task
inline volatile uint8_t applied_enable_mask = 0x3f; // torque requested by the host; guarded by TmcLock
inline volatile uint32_t fault_events = 0;       // drv_err rising edges (over-temp / short)

inline uint8_t misses[6] = {0};                  // task-only
inline uint32_t answering_since[6] = {0};        // task-only; 0 = not in a settle window
inline uint8_t fault_latched = 0;                // task-only

inline uint8_t ready()  { return ready_mask; }
inline uint8_t homed()  { return homed_mask; }
inline bool isReady(uint8_t joint) { return joint >= 1 && joint <= 6 && (ready_mask & (1u << (joint - 1))); }

inline void setHomed(uint8_t bits) {
    portENTER_CRITICAL(&mux);
    homed_mask |= (bits & ready_mask);   // a joint that lost power mid-homing stays un-homed
    portEXIT_CRITICAL(&mux);
}

// Returns and clears the joints lost since the last call (loop task).
inline uint8_t takeLost() {
    portENTER_CRITICAL(&mux);
    uint8_t m = lost_mask;
    lost_mask = 0;
    portEXIT_CRITICAL(&mux);
    return m;
}

inline void markLost(uint8_t bit) {
    portENTER_CRITICAL(&mux);
    ready_mask &= (uint8_t)~bit;
    homed_mask &= (uint8_t)~bit;
    lost_mask  |= bit;
    portEXIT_CRITICAL(&mux);
}

inline void markReady(uint8_t bit) {
    portENTER_CRITICAL(&mux);
    ready_mask |= bit;
    portEXIT_CRITICAL(&mux);
}

inline void pollDriver(uint8_t i) {
    const uint8_t bit = (uint8_t)(1u << i);
    TmcLock lock;
    TMC2209Stepper* d = DRIVERS[i];
    uint8_t gstat = d->GSTAT();

    if (d->CRCerror) {                               // no (valid) reply: VM is gone
        answering_since[i] = 0;                      // restart the settle window
        if (misses[i] < 255) misses[i]++;
        if (misses[i] >= MISS_LIMIT && (ready_mask & bit)) markLost(bit);
        return;
    }
    misses[i] = 0;

    if (!(ready_mask & bit) || (gstat & GSTAT_RESET)) {
        // Power just returned, or blipped faster than MISS_LIMIT polls could
        // see. Either way the motor was de-energized, so position is suspect.
        if (ready_mask & bit) markLost(bit);
        // Wait for the supply to settle; the reset flag stays set until we
        // clear it below, so nothing is lost by waiting.
        const uint32_t now = millis();
        if (!answering_since[i]) { answering_since[i] = now ? now : 1; return; }
        if (now - answering_since[i] < SETTLE_MS) return;
        answering_since[i] = 0;
        configureDriver(i, applied_enable_mask & bit);
        d->GSTAT(0b111);
        fault_latched &= (uint8_t)~bit;
        markReady(bit);
        return;
    }

    if (gstat & GSTAT_DRV_ERR) {
        if (!(fault_latched & bit)) { fault_latched |= bit; fault_events++; }
        d->GSTAT(0b111);
    } else {
        fault_latched &= (uint8_t)~bit;
    }
}

inline void taskFn(void*) {
    uint8_t i = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        pollDriver(i);
        i = (uint8_t)((i + 1) % 6);
    }
}

// initial_ready: drivers that answered during initJoints() (already configured).
inline void start(uint8_t initial_ready) {
    ready_mask = initial_ready & 0x3f;
    xTaskCreatePinnedToCore(taskFn, "power", 3072, nullptr, 1, nullptr, 0);
}

} // namespace Power
