#pragma once
#include <Arduino.h>
#include <soc/gpio_struct.h>

// ---------------------------------------------------------
// ESP32 Direct Digital Synthesis (DDS) Stepper Driver
// ---------------------------------------------------------

class SixAxisDDS {
private:
    // --- Hardware Timer Configuration ---
    static const uint32_t TIMER_BASE_FREQ_HZ = 1000000;
    static const uint32_t TIMER_FREQ_HZ = 100000; // 100 kHz ISR frequency
    hw_timer_t * timer = NULL;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    // --- Volatile variables shared between Loop and ISR ---
    volatile uint32_t step_increments[6] = {0, 0, 0, 0, 0, 0};
    volatile uint32_t accumulators[6] = {0, 0, 0, 0, 0, 0};
    
    // Tracks the physical position for closed-loop correction
    volatile int32_t absolute_steps[6] = {0, 0, 0, 0, 0, 0};
    volatile bool moving_forward[6] = {true, true, true, true, true, true};

    // Fast GPIO Bitmasks (To avoid slow digitalWrite commands in the ISR)
    uint32_t step_masks_low[6] = {0};  // For pins 0-31
    uint32_t step_masks_high[6] = {0}; // For pins 32-39
    uint8_t dir_pins_array[6];

    // Singleton instance pointer for the static ISR
    static SixAxisDDS* instance;

    // ---------------------------------------------------------
    // THE INTERRUPT SERVICE ROUTINE (Executes every 10us)
    // ---------------------------------------------------------
    static void IRAM_ATTR onTimerISR() {
        // 1. CLEAR ALL STEP PINS FIRST
        // W1TC (Write 1 To Clear) instantly pulls the pins LOW.
        for(int i=0; i<6; i++) {
            if (instance->step_masks_low[i])  GPIO.out_w1tc = instance->step_masks_low[i];
            if (instance->step_masks_high[i]) GPIO.out1_w1tc.val = instance->step_masks_high[i];
        }

        // 2. RUN THE PHASE ACCUMULATOR MATH
        portENTER_CRITICAL_ISR(&instance->timerMux);
        for (int i = 0; i < 6; i++) {
            // Skip math if motor is stopped
            if (instance->step_increments[i] == 0) continue;

            uint32_t previous_acc = instance->accumulators[i];
            
            // Add the velocity increment
            instance->accumulators[i] += instance->step_increments[i];

            // If the 32-bit integer overflows, it wraps around back to 0.
            if (instance->accumulators[i] < previous_acc) {
                // FIRE THE STEP PULSE! 
                // W1TS (Write 1 To Set) instantly pulls the pin HIGH.
                if (instance->step_masks_low[i])  GPIO.out_w1ts = instance->step_masks_low[i];
                if (instance->step_masks_high[i]) GPIO.out1_w1ts.val = instance->step_masks_high[i];

                // Track absolute physical position
                if (instance->moving_forward[i]) {
                    instance->absolute_steps[i]++;
                } else {
                    instance->absolute_steps[i]--;
                }
            }
        }
        portEXIT_CRITICAL_ISR(&instance->timerMux);
    }

public:
    SixAxisDDS() {
        instance = this;
    }

    // ---------------------------------------------------------
    // INITIALIZATION (Core 3.x API)
    // ---------------------------------------------------------
    void init(const uint8_t step_pins[6], const uint8_t dir_pins[6]) {
        for (int i = 0; i < 6; i++) {
            dir_pins_array[i] = dir_pins[i];
            pinMode(step_pins[i], OUTPUT);
            pinMode(dir_pins_array[i], OUTPUT);
            digitalWrite(step_pins[i], LOW);

            // Pre-compute the fast GPIO bitmasks
            if (step_pins[i] < 32) {
                step_masks_low[i] = (1UL << step_pins[i]);
            } else {
                step_masks_high[i] = (1UL << (step_pins[i] - 32));
            }
        }

        // 1. Begin the timer with a 1 MHz base frequency (1,000,000 Hz)
        timer = timerBegin(TIMER_BASE_FREQ_HZ);
        
        // 2. Attach our ISR
        timerAttachInterrupt(timer, &onTimerISR);

        
        
        // 3. Set Alarm
        // Trigger at 10 ticks (1,000,000 / 10 = 100,000 Hz)
        // autoreload = true
        // reload_count = 0
        timerAlarm(timer, TIMER_BASE_FREQ_HZ/TIMER_FREQ_HZ, true, 0); 
    }

    // ---------------------------------------------------------
    // READ PHYSICAL STEPS (For Closed-Loop Tracking)
    // ---------------------------------------------------------
    int32_t getSteps(int axis) {
        int32_t current_steps = 0;
        portENTER_CRITICAL(&timerMux);
        current_steps = absolute_steps[axis];
        portEXIT_CRITICAL(&timerMux);
        return current_steps;
    }

    // ---------------------------------------------------------
    // UPDATE VELOCITIES (Call this from your Ruckig Loop)
    // ---------------------------------------------------------
    void updateVelocities(float target_velocities[6]) {
        // 1. Do the slow GPIO writes OUTSIDE the spinlock
        for (int i = 0; i < 6; i++) {
            if (target_velocities[i] < 0) {
                digitalWrite(dir_pins_array[i], LOW); 
            } else if (target_velocities[i] > 0) {
                digitalWrite(dir_pins_array[i], HIGH);
            }
        }

        // 2. Do the fast math INSIDE the spinlock
        portENTER_CRITICAL(&timerMux);
        for (int i = 0; i < 6; i++) {
            float v = target_velocities[i];
            
            if (v < 0) {
                moving_forward[i] = false;
                v = -v; 
            } else if (v > 0) {
                moving_forward[i] = true;
            }

            if (v == 0.0f) {
                step_increments[i] = 0;
            } else {
                // SAFETY: the increment is scaled by 2^32. At v = 100000 steps/s
                // it equals 2^32 and wraps to 0 (motor stops); above that it is
                // garbage. The ISR also can't emit >1 step per 10us tick = 100k
                // steps/s. Clamp so we degrade gracefully instead of overflowing.
                // if (v > 95000.0f) v = 95000.0f;
                step_increments[i] = (uint32_t)((v / (float)TIMER_FREQ_HZ) * 4294967296.0f);
            }
        }
        portEXIT_CRITICAL(&timerMux);
    }
};

// Define static instance pointer
SixAxisDDS* SixAxisDDS::instance = nullptr;