#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include "motor_init.h"


// ================= LIMIT SWITCHES =================
#define L1 5  // Base (J1) -- !! NOT INSTALLED YET
#define L2 36 // Shoulder (J2) -- Normally high
#define L3 39 // Elbow (J3) -- Normally high
#define L4 18 // Wrist1 (J4) -- Normally low
#define L5 34 // Wrist2 (J5) -- Normally high
#define L6 35 // Wrist3 (J6) -- Normally low





// ================= UART PINS =================
#define TMC_RX 15
#define TMC_TX 4
#define TMC2_RX 16
#define TMC2_TX 17

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2

// Current readings
// Just J5 (as uart broken) = 0.35A
// J5 + J2                  = 1.018A
// J5 + J2 + J3


#define TICKS_PER_S 16000000 
#define TIME_SLICE_S 0.005 // 5ms per command queue entry

void setup()
{   
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(1000);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 0, 1, 1, 0, 0, 1); 
    pinMode(L1, INPUT_PULLUP);
    pinMode(L2, INPUT_PULLUP);
    pinMode(L3, INPUT_PULLUP);
    pinMode(L4, INPUT_PULLUP);
    pinMode(L5, INPUT_PULLUP); // Pin 18 supports INPUT_PULLUP if needed, but keeping as INPUT to match the rest
    pinMode(L6, INPUT_PULLUP);


    engine.init();
    stepper = engine.stepperConnectToPin(J2_STEP_PIN);
  if (stepper) 
  {
        stepper->setDirectionPin(J2_DIR_PIN, false);
        // stepper->setEnablePin(255); // no enable pin
        stepper->setAutoEnable(true);
        stepper->setSpeedInHz(60000);      // steps/sec
        // stepper->setAcceleration(8000);  // steps/sec^2
  }
  // J2_driver.en_spreadCycle(true);
  // J2_driver.pwm_autoscale(false);
  // J2_driver.intpol(true);
  // J2_driver.TCOOLTHRS(0xFFFFF);
    
}

// ---------------------------------------------------------
// 1. The Math: Evaluates normalized quintic position (0.0 to 1.0)
// ---------------------------------------------------------
float getQuinticPosition(float t, float T) {
  if (t <= 0.0) return 0.0;
  if (t >= T) return 1.0;
  
  float tau = t / T;
  return 10.0 * pow(tau, 3) - 15.0 * pow(tau, 4) + 6.0 * pow(tau, 5);
}

// ---------------------------------------------------------
// 2. The Generator: Slices time and fills the command queue
// ---------------------------------------------------------
void executeQuinticMove(int32_t steps_to_move, float duration_sec) {
  if (steps_to_move == 0) return;

  bool dir_high = (steps_to_move > 0);
  uint32_t total_steps = abs(steps_to_move);
  
  int num_slices = duration_sec / TIME_SLICE_S;
  uint32_t steps_completed = 0;

  for (int i = 1; i <= num_slices; i++) {
    float t = i * TIME_SLICE_S;
    if (i == num_slices) t = duration_sec; // Force exact end time

    // Calculate target position at the end of this 4ms window
    float normalized_pos = getQuinticPosition(t, duration_sec);
    uint32_t target_step_count = round(normalized_pos * total_steps);
    uint32_t slice_steps = target_step_count - steps_completed;
    
    uint32_t steps_left_in_slice = slice_steps;

    // Handle the zero-step pause (velocity is nearly zero)
    if (steps_left_in_slice == 0) {
      struct stepper_command_s cmd;
      cmd.count_up = dir_high;
      cmd.steps = 0;
      cmd.ticks = TIME_SLICE_S * TICKS_PER_S; 

      AqeResultCode res = stepper->addQueueEntry(&cmd, true);
      // Explicitly cast enum to int to fix the compiler error
      while ((int)res > 0) { 
        // delay(1);
        res = stepper->addQueueEntry(&cmd, true);
      }
      if ((int)res < 0) break;
    } 
    // Handle movement (chunking into 255-step blocks)
    else {
      uint32_t base_ticks = (TIME_SLICE_S * TICKS_PER_S) / slice_steps;

      while (steps_left_in_slice > 0) {
        // FastAccelStepper strictly limits steps to 8-bit (max 255)
        uint8_t steps_to_send = (steps_left_in_slice > 255) ? 255 : steps_left_in_slice;

        struct stepper_command_s cmd;
        cmd.count_up = dir_high;
        cmd.steps = steps_to_send;
        cmd.ticks = base_ticks;

        AqeResultCode res = stepper->addQueueEntry(&cmd, true);
        
        while ((int)res > 0) {
          // delay(1);
          res = stepper->addQueueEntry(&cmd, true);
        }
        
        if ((int)res < 0) {
          Serial.printf("Fatal error: %d\n", (int)res);
          break;
        }
        steps_left_in_slice -= steps_to_send;
      }
    }
    
    steps_completed = target_step_count;
  }
}

// ---------------------------------------------------------
// 3. The Loop: 0 -> 1600 -> 0
// ---------------------------------------------------------
void loop() {
  if (stepper) {
    Serial.println("Moving forward 1600 steps...");
    executeQuinticMove(5000, 1.1); // Move +1600 steps over 1.5 seconds
    
    // Wait for the hardware queue to empty before starting the next move
    while (!stepper->isQueueEmpty()) {
      
    }
    // delay(50); // Brief pause at the end

    Serial.println("Moving backward 1600 steps...");
    executeQuinticMove(-5000, 1.1); // Move -1600 steps over 1.5 seconds
    
    while (!stepper->isQueueEmpty()) {
      // delay(10);
    }
    // delay(50); // Brief pause at home
  }
}

