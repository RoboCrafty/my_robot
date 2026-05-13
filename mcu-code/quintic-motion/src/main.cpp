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
#define TIME_SLICE_S 0.02 // 5ms per command queue entry

const float MOTOR_STEPS_PER_REV = 200.0; // Standard 1.8 deg stepper
const float MICROSTEPPING = 8.0;        // Check your stepper driver switches
const float GEAR_RATIO = 20.0;            // e.g., 5.18 for a planetary gearbox
const float STEPS_PER_DEGREE = (MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO) / 360.0;
int32_t current_absolute_steps = 0;

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
    stepper = engine.stepperConnectToPin(J3_STEP_PIN);
  if (stepper) 
  {
        // stepper->setDirectionPin(J2_DIR_PIN, false);
        stepper->setDirectionPin(J3_DIR_PIN, false);
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
// void loop() {
//   if (stepper) {
//     Serial.println("Moving forward 1600 steps...");
//     executeQuinticMove(40000, 2.7); // Move +1600 steps over 1.5 seconds
    
//     // Wait for the hardware queue to empty before starting the next move
//     while (!stepper->isQueueEmpty()) {
      
//     }
//     // delay(50); // Brief pause at the end

//     Serial.println("Moving backward 1600 steps...");
//     executeQuinticMove(-40000, 2.7); // Move -1600 steps over 1.5 seconds
    
//     while (!stepper->isQueueEmpty()) {
//       // delay(10);
//     }
//     // delay(50); // Brief pause at home
//   }
// }

// ---------------------------------------------------------
// 3. Go to commanded angle
// ---------------------------------------------------------
void loop() {
  // Check if user sent a command over Serial
  if (Serial.available() > 0) {
    // Read the incoming string until the user presses Enter
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any stray whitespace or carriage returns
    
    if (input.length() > 0) {
      float target_angle = input.toFloat();
      
      // 1. Convert Angle to Absolute Steps
      int32_t target_absolute_steps = round(target_angle * STEPS_PER_DEGREE);
      
      // 2. Calculate how far we actually need to move
      int32_t steps_to_move = target_absolute_steps - current_absolute_steps;
      
      if (steps_to_move != 0) {
        Serial.printf("\nTarget Angle: %.2f degrees\n", target_angle);
        Serial.printf("Moving %d steps...\n", steps_to_move);
        
        // PRO TIP: Calculate a dynamic duration!
        // Moving 10 degrees in 1.5s is slow, but 3600 degrees in 1.5s might stall the motor.
        // This ensures the motor averages ~1000 steps per second.
        float dynamic_duration = max(0.5f, abs(steps_to_move) / 4000.0f);
        
        // float duration = 1.5; // Keeping it fixed for now
        
        // 3. Execute the move
        executeQuinticMove(steps_to_move, dynamic_duration);
        
        // 4. Wait for the move to finish
        while (!stepper->isQueueEmpty()) { 
          delay(10); 
        }
        
        // 5. Update our internal tracker
        current_absolute_steps = target_absolute_steps;
        Serial.println("Arrived! Enter new angle:");
        
      } else {
        Serial.println("Already at target angle!");
      }
    }
  }
}













// ####################################################################### code 2 ##############################################################
// ######################################################################################################################################################


// #include <Arduino.h>
// #include "FastAccelStepper.h"
// #include "TMCStepper.h"
// #include "motor_init.h"


// // ================= LIMIT SWITCHES =================
// #define L1 5  // Base (J1) -- !! NOT INSTALLED YET
// #define L2 36 // Shoulder (J2) -- Normally high
// #define L3 39 // Elbow (J3) -- Normally high
// #define L4 18 // Wrist1 (J4) -- Normally low
// #define L5 34 // Wrist2 (J5) -- Normally high
// #define L6 35 // Wrist3 (J6) -- Normally low





// // ================= UART PINS =================
// #define TMC_RX 15
// #define TMC_TX 4
// #define TMC2_RX 16
// #define TMC2_TX 17

// #define SERIAL_PORT1 Serial1
// #define SERIAL_PORT2 Serial2

// // Current readings
// // Just J5 (as uart broken) = 0.35A
// // J5 + J2                  = 1.018A
// // J5 + J2 + J3


// void setup()
// {   
//     Serial.begin(115200);
//     Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
//     Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

//     // Initialise
//     delay(1000);
//     Serial.println("\n--- Initializing System ---");
//     delay(100);

//     initJoints(1, 0, 1, 1, 0, 0, 1); 
//     pinMode(L1, INPUT_PULLUP);
//     pinMode(L2, INPUT_PULLUP);
//     pinMode(L3, INPUT_PULLUP);
//     pinMode(L4, INPUT_PULLUP);
//     pinMode(L5, INPUT_PULLUP); // Pin 18 supports INPUT_PULLUP if needed, but keeping as INPUT to match the rest
//     pinMode(L6, INPUT_PULLUP);


//     engine.init();
//   stepper = engine.stepperConnectToPin(J2_STEP_PIN);
//   if (stepper) 
//   {
//         stepper->setDirectionPin(J2_DIR_PIN, false);
//         // stepperJ1->setEnablePin(255); // no enable pin
//         stepper->setAutoEnable(true);
//         stepper->setSpeedInHz(8000);      // steps/sec
//         stepper->setAcceleration(32000);  // steps/sec^2
//   }
  
    



// }

// // Variables to track our raw movement
// uint32_t steps_remaining = 000;
// bool movement_done = false;
// uint64_t itr = 0;
// uint32_t start_time = 0;
// int target_position = 0;
// String inputString = "";
// uint16_t ticks = 8000;  // default speed
// bool printed;
// static bool goingForward = true;
// void loop()
// {
//   // // Read serial input
//   // if (Serial.available()) {
//   //   char c = Serial.read();

//   //   if (c == '\n') {
//   //     int steps = atoi(inputString.c_str());

//   //     target_position = steps * 1000;  // relative move

//   //     Serial.print("Moving at vactual: ");
//   //     Serial.println(target_position);
//   //     J3_driver.VACTUAL(target_position);
//   //     stepperJ1->mo
      
     

//   //     inputString = "";
//   //   } else {
//   //     inputString += c;
//   //   }
//   // }
//   if (!stepper->isRunning()) {

//     if (goingForward) {
//         stepper->moveTo(5000);
//     } else {
//         stepper->moveTo(0);
//     }

//     goingForward = !goingForward;
// }

//   // int stateL1 = digitalRead(L1);
//   // int stateL2 = digitalRead(L2);
//   // int stateL3 = digitalRead(L3);
//   // int stateL4 = digitalRead(L4);
//   // int stateL5 = digitalRead(L5);
//   // int stateL6 = digitalRead(L6);

//   // // Print the states to the Serial Monitor
//   // Serial.print(" | J1: "); Serial.print(stateL1);
//   // Serial.print(" | J2: "); Serial.print(stateL2);
//   // Serial.print(" | J3: "); Serial.print(stateL3);
//   // Serial.print(" | J4 "); Serial.print(stateL4);
//   // Serial.print(" | J5 "); Serial.print(stateL5);
//   // Serial.print(" | J6 "); Serial.println(stateL6);
  

//   // Serial.println(J6_driver.c);

//   // // Optional: print when motion done
//   // if (stepperJ1->isRunning() == false) {
//   //   static bool printed = false;
//   //   if (!printed) {
//   //     Serial.println("✅ Move complete");
//   //     printed = true;
//   //   }
//   // } else {
//   //   printed = false;
//   // }

// //   Serial.print("Current Scale Actual: ");
// //     Serial.println(J3_driver.cs_actual());
// }
