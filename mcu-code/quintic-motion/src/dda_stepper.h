#pragma once
//
// dda_stepper.h
// -----------------------------------------------------------------------------
// A tiny shared-timebase DDA (Bresenham-style phase accumulator) step generator
// for synchronised multi-axis stepping on the classic ESP32 (Arduino-ESP32 3.x).
//
// WHY THIS EXISTS
//   FastAccelStepper.moveTimed() quantises both the step count and the per-step
//   tick interval of each axis independently. Near the velocity ceiling that
//   integer rounding shows up as audible velocity ripple, and the axes are not
//   guaranteed to share a timebase. This module instead runs ONE hardware timer
//   ISR at a fixed high rate (e.g. 100 kHz) and, on every tick, advances a 32-bit
//   phase accumulator per axis. When an accumulator overflows, that axis emits a
//   single step. The fractional remainder is carried across ticks AND across
//   control cycles, so the commanded velocity is reproduced with sub-step
//   resolution and zero per-window rounding. All axes share the same tick, so
//   they stay time-synchronised by construction.
//
// CONTRACT
//   - init()   : reclaim the GPIO pins and pre-compute per-pin bank masks.
//   - start()  : begin the 100 kHz ISR.
//   - setVelocity(axis, steps_per_sec) : called from your control loop (~1 kHz).
//       Sets the DIR pin + the signed Q32 increment for that axis.
//   - getPosition(axis) : emitted step count (updated only by the ISR).
//   - setPosition(axis, steps) : sync the counter (e.g. to 0 after homing).
//
// All floating point lives in setVelocity() (loop context). The ISR is pure
// integer + register writes and is placed in IRAM.
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <soc/gpio_struct.h>

namespace dda {

constexpr uint8_t  NUM_AXES = 6;
constexpr uint32_t TICK_HZ  = 100000;            // ISR frequency (100 kHz -> 10 us)
constexpr uint32_t TIMER_RES_HZ = 1000000;       // 1 MHz timer base -> 1 us resolution
constexpr uint32_t TIMER_ALARM  = TIMER_RES_HZ / TICK_HZ;  // = 10 ticks -> 10 us

// 2^32 / TICK_HZ : steps/sec -> Q32 increment per tick.
// One full 32-bit wrap of the accumulator == one emitted step.
static const float Q32_PER_SPS = 4294967296.0f / (float)TICK_HZ;

// A single axis state. Hot fields are touched by the ISR.
struct Axis {
    // --- configuration (set once in init) ---
    uint8_t  step_pin = 0;
    uint8_t  dir_pin  = 0;
    bool     dir_high_is_positive = true;   // matches FastAccelStepper dir polarity
    uint32_t step_mask_lo = 0;              // bit for GPIO 0..31  (out_w1ts/out_w1tc)
    uint32_t step_mask_hi = 0;              // bit for GPIO 32..39 (out1_w1ts/out1_w1tc)

    // --- runtime (shared with ISR) ---
    volatile uint32_t accumulator = 0;      // phase accumulator (ISR only)
    volatile uint32_t increment   = 0;      // |steps/tick| in Q32 (loop writes, ISR reads)
    volatile int8_t   step_sign   = 0;      // -1 / 0 / +1            (loop writes, ISR reads)
    volatile int32_t  position    = 0;      // emitted step count    (ISR writes, loop reads)
};

inline Axis axes[NUM_AXES];
inline hw_timer_t* timer = nullptr;

// Masks of step pins driven HIGH on the previous tick, so the next tick can pull
// them LOW (ends the pulse). Split by GPIO bank.
inline volatile uint32_t prev_pulse_lo = 0;
inline volatile uint32_t prev_pulse_hi = 0;

// -----------------------------------------------------------------------------
// ISR: runs at TICK_HZ. Integer + register writes only.
// -----------------------------------------------------------------------------
inline void IRAM_ATTR onTick() {
    // 1. End the pulses started on the previous tick.
    if (prev_pulse_lo) GPIO.out_w1tc      = prev_pulse_lo;
    if (prev_pulse_hi) GPIO.out1_w1tc.val = prev_pulse_hi;

    // 2. Advance each accumulator; collect axes that step this tick.
    uint32_t set_lo = 0;
    uint32_t set_hi = 0;
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        Axis& a = axes[i];
        const int8_t sgn = a.step_sign;
        if (sgn == 0) continue;
        const uint32_t before = a.accumulator;
        const uint32_t after  = before + a.increment;
        a.accumulator = after;
        if (after < before) {            // 32-bit wrap == one whole step due
            a.position += sgn;
            set_lo |= a.step_mask_lo;
            set_hi |= a.step_mask_hi;
        }
    }

    // 3. Raise the new step pulses.
    if (set_lo) GPIO.out_w1ts      = set_lo;
    if (set_hi) GPIO.out1_w1ts.val = set_hi;
    prev_pulse_lo = set_lo;
    prev_pulse_hi = set_hi;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
// Reclaim raw GPIO control of the step/dir pins (call AFTER any FastAccelStepper
// homing + detachFromPin()) and pre-compute the per-pin bank masks.
inline void init(const uint8_t step_pins[NUM_AXES],
                 const uint8_t dir_pins[NUM_AXES],
                 const bool    dir_high_is_positive[NUM_AXES]) {
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        Axis& a = axes[i];
        a.step_pin = step_pins[i];
        a.dir_pin  = dir_pins[i];
        a.dir_high_is_positive = dir_high_is_positive[i];
        a.accumulator = 0;
        a.increment   = 0;
        a.step_sign   = 0;
        a.position    = 0;

        if (a.step_pin < 32) { a.step_mask_lo = (1u << a.step_pin);        a.step_mask_hi = 0; }
        else                 { a.step_mask_lo = 0; a.step_mask_hi = (1u << (a.step_pin - 32)); }

        pinMode(a.step_pin, OUTPUT);
        pinMode(a.dir_pin,  OUTPUT);
        digitalWrite(a.step_pin, LOW);
        digitalWrite(a.dir_pin,  a.dir_high_is_positive ? LOW : HIGH);  // default = negative-side rest
    }
    prev_pulse_lo = 0;
    prev_pulse_hi = 0;
}

// Start the 100 kHz step ISR.
inline void start() {
    if (timer) return;
    timer = timerBegin(TIMER_RES_HZ);            // 1 MHz base
    timerAttachInterrupt(timer, &onTick);
    timerAlarm(timer, TIMER_ALARM, true, 0);     // fire every TIMER_ALARM us, auto-reload
}

// Stop the ISR (and ensure no pin is left high).
inline void stop() {
    if (!timer) return;
    timerEnd(timer);
    timer = nullptr;
    if (prev_pulse_lo) GPIO.out_w1tc      = prev_pulse_lo;
    if (prev_pulse_hi) GPIO.out1_w1tc.val = prev_pulse_hi;
    prev_pulse_lo = 0;
    prev_pulse_hi = 0;
}

// -----------------------------------------------------------------------------
// Control-loop API (loop / control task context)
// -----------------------------------------------------------------------------
// Command a signed velocity in steps/second for one axis. Floating point math
// is done here, never in the ISR. |velocity| is clamped below TICK_HZ so the
// single-step-per-tick scheme stays valid.
inline void setVelocity(uint8_t axis, float steps_per_sec) {
    if (axis >= NUM_AXES) return;
    Axis& a = axes[axis];

    int8_t sgn;
    float mag;
    if (steps_per_sec > 0.0f)      { sgn = +1; mag = steps_per_sec; }
    else if (steps_per_sec < 0.0f) { sgn = -1; mag = -steps_per_sec; }
    else                           { a.step_sign = 0; a.increment = 0; return; }

    // Clamp to just under the single-step-per-tick ceiling.
    const float max_sps = (float)TICK_HZ * 0.98f;
    if (mag > max_sps) mag = max_sps;

    const uint32_t inc = (uint32_t)(mag * Q32_PER_SPS);

    // Set DIR before enabling the sign so the physical direction is settled
    // before the next emitted step. Direction only flips at ~zero velocity, so
    // there is no glitch in practice.
    digitalWrite(a.dir_pin, (sgn > 0) == a.dir_high_is_positive ? HIGH : LOW);
    a.increment = inc;
    a.step_sign = sgn;
}

// Stop every axis immediately (no further steps emitted).
inline void stopAll() {
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        axes[i].step_sign = 0;
        axes[i].increment = 0;
    }
}

// Emitted step count for an axis (updated by the ISR).
inline int32_t getPosition(uint8_t axis) {
    return (axis < NUM_AXES) ? axes[axis].position : 0;
}

// Force the tracked position (e.g. to 0 right after homing). Only call while the
// axis is at rest.
inline void setPosition(uint8_t axis, int32_t steps) {
    if (axis >= NUM_AXES) return;
    axes[axis].position    = steps;
    axes[axis].accumulator = 0;
}

} // namespace dda
