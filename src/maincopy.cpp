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


void setup()
{   
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(1000);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 0, 0, 1, 0, 0, 0); 
    pinMode(L1, INPUT_PULLUP);
    pinMode(L2, INPUT_PULLUP);
    pinMode(L3, INPUT_PULLUP);
    pinMode(L4, INPUT_PULLUP);
    pinMode(L5, INPUT_PULLUP); // Pin 18 supports INPUT_PULLUP if needed, but keeping as INPUT to match the rest
    pinMode(L6, INPUT_PULLUP);


    engine.init();
  stepperJ1 = engine.stepperConnectToPin(J3_STEP_PIN);
  if (stepperJ1) 
  {
        stepperJ1->setDirectionPin(J3_DIR_PIN);
        // stepperJ1->setEnablePin(255); // no enable pin
        stepperJ1->setAutoEnable(true);
        stepperJ1->setSpeedInHz(6000);      // steps/sec
        stepperJ1->setAcceleration(8000);  // steps/sec^2
  }
  
    



}

// Variables to track our raw movement
uint32_t steps_remaining = 000;
bool movement_done = false;
uint64_t itr = 0;
uint32_t start_time = 0;
int target_position = 0;
String inputString = "";
uint16_t ticks = 8000;  // default speed
bool printed;
static bool goingForward = true;
void loop()
{
  // Read serial input
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      int steps = atoi(inputString.c_str());

      target_position = steps * 1000;  // relative move

      Serial.print("Moving at vactual: ");
      Serial.println(target_position);
      J3_driver.VACTUAL(target_position);
      stepperJ1->mo
      
     

      inputString = "";
    } else {
      inputString += c;
    }
  }
//   if (!stepperJ1->isRunning()) {

//     if (goingForward) {
//         stepperJ1->moveTo(12000);
//     } else {
//         stepperJ1->moveTo(0);
//     }

//     goingForward = !goingForward;
// }

  // int stateL1 = digitalRead(L1);
  // int stateL2 = digitalRead(L2);
  // int stateL3 = digitalRead(L3);
  // int stateL4 = digitalRead(L4);
  // int stateL5 = digitalRead(L5);
  // int stateL6 = digitalRead(L6);

  // // Print the states to the Serial Monitor
  // Serial.print(" | J1: "); Serial.print(stateL1);
  // Serial.print(" | J2: "); Serial.print(stateL2);
  // Serial.print(" | J3: "); Serial.print(stateL3);
  // Serial.print(" | J4 "); Serial.print(stateL4);
  // Serial.print(" | J5 "); Serial.print(stateL5);
  // Serial.print(" | J6 "); Serial.println(stateL6);
  

  // Serial.println(J6_driver.c);

  // // Optional: print when motion done
  // if (stepperJ1->isRunning() == false) {
  //   static bool printed = false;
  //   if (!printed) {
  //     Serial.println("✅ Move complete");
  //     printed = true;
  //   }
  // } else {
  //   printed = false;
  // }

//   Serial.print("Current Scale Actual: ");
//     Serial.println(J3_driver.cs_actual());
}

// struct JointConfig
// {
//     int MS            = 16;
//     int gear_ratio    = 10;
//     int steps_per_rev = 200* MS * gear_ratio;

// };

// void populateJointConfig(JointConfig* JC){
//     JC->steps_per_rev = 200 * JC->MS * JC->gear_ratio;

// }

// double steps_to_move(int& steps_per_rev, double& angle_to_move)
// {
//     return steps_per_rev * (angle_to_move/2*M_PI);
// }




// void setup() {
//   Serial.begin(115200);
//   Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
//   Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);
//   // Connect Joint 6 stepper
//   engine.init();
//   stepperJ1 = engine.stepperConnectToPin(J1_STEP_PIN);
//   if (stepperJ1) 
//   {
//         stepperJ1->setDirectionPin(J1_DIR_PIN);
//         // stepperJ1->setEnablePin(255); // no enable pin
//         stepperJ1->setAutoEnable(true);
//         stepperJ1->setSpeedInHz(10000);      // steps/sec
//         stepperJ1->setAcceleration(4000);  // steps/sec^2
//   }
// //   driver.TCOOLTHRS(0);
  
//   Serial.println("Waiting 1s");
//   delay(500);
//   Serial.println("Waiting 1s");
//   delay(500);

//   J1_driver.begin();
//   J1_driver.toff(2);
//   J1_driver.blank_time(24);
//   J1_driver.en_spreadCycle();
//   J1_driver.TCOOLTHRS(0);
//   J1_driver.rms_current(1800,1.0); 
//   J1_driver.microsteps(8);
// //   J1_driver.pdn_disable(true);       // <--- ADD THIS: Disables the PDN/UART pin's secondary function
// //   J1_driver.I_scale_analog(false);   // <--- ADD THIS: Use internal voltage reference
//   Serial.print(F("\nTesting driver 1 connection... 0"));
//   uint8_t result = J1_driver.test_connection();
//    delay(500);
//   if (result) {
//     Serial.println(F("failed!"));
//     Serial.print(F("Likely cause: "));
//     switch (result) {
//       case 1: Serial.println(F("loose connection")); break;
//       case 2: Serial.println(F("Likely cause: no power")); break;
//     }
//     Serial.println(F("Fix the problem and reset board."));
//     // abort();
//   }
//   else{
//       Serial.println("driver 1 Initialized!");
//   }

  






//   J2_driver.begin();
//   J2_driver.en_spreadCycle();
//   J2_driver.toff(0);
//   J2_driver.blank_time(24);
//   J2_driver.TCOOLTHRS(0);
//   J2_driver.rms_current(1800,1.0); 
//   J2_driver.microsteps(8);
// //   driver.pdn_disable(true);       // <--- ADD THIS: Disables the PDN/UART pin's secondary function
// //   driver.I_scale_analog(false);   // <--- ADD THIS: Use internal voltage reference
//   Serial.print(F("\nTesting driver 2 connection... 0"));
//   result = J2_driver.test_connection();
//    delay(500);
//   if (result) {
//     Serial.println(F("failed!"));
//     Serial.print(F("Likely cause: "));
//     switch (result) {
//       case 1: Serial.println(F("loose connection")); break;
//       case 2: Serial.println(F("Likely cause: no power")); break;
//     }
//     Serial.println(F("Fix the problem and reset board."));
//     // abort();
//   }
//   else{
//       Serial.println("driver 2 Initialized!");
//   }




//   J3_driver.begin();
//   J3_driver.toff(0);
//   J3_driver.blank_time(24);
//   J3_driver.TCOOLTHRS(0);
  
//   J3_driver.rms_current(300); 
//   J3_driver.ihold(31);
//   J3_driver.irun(31);
//   J3_driver.microsteps(16);
// //   driver.pdn_disable(true);       // <--- ADD THIS: Disables the PDN/UART pin's secondary function
// //   driver.I_scale_analog(false);   // <--- ADD THIS: Use internal voltage reference
//   Serial.print(F("\nTesting driver 3 connection... 0"));
//   result = J3_driver.test_connection();
//    delay(500);
//   if (result) {
//     Serial.println(F("failed!"));
//     Serial.print(F("Likely cause: "));
//     switch (result) {
//       case 1: Serial.println(F("loose connection")); break;
//       case 2: Serial.println(F("Likely cause: no power")); break;
//     }
//     Serial.println(F("Fix the problem and reset board."));
//     // abort();
//   }
//   else{
//       Serial.println("driver 3 Initialized!");
//   }



//   J4_driver.begin();
//   J4_driver.toff(0);
//   J4_driver.blank_time(24);
  
//   J4_driver.rms_current(1500); 
//   J4_driver.microsteps(16);
// //   driver.pdn_disable(true);       // <--- ADD THIS: Disables the PDN/UART pin's secondary function
// //   driver.I_scale_analog(false);   // <--- ADD THIS: Use internal voltage reference
//   Serial.print(F("\nTesting driver 4 connection... 0"));
//   result = J4_driver.test_connection();
//    delay(500);
//   if (result) {
//     Serial.println(F("failed!"));
//     Serial.print(F("Likely cause: "));
//     switch (result) {
//       case 1: Serial.println(F("loose connection")); break;
//       case 2: Serial.println(F("Likely cause: no power")); break;
//     }
//     Serial.println(F("Fix the problem and reset board."));
//     // abort();
//   }
//   else{
//       Serial.println("driver 4 Initialized!");
//   }


//   J5_driver.begin();
//   J5_driver.toff(0);
//   J5_driver.blank_time(24);
  
//   J5_driver.rms_current(1500); 
//   J5_driver.microsteps(16);
// //   driver.pdn_disable(true);       // <--- ADD THIS: Disables the PDN/UART pin's secondary function
// //   driver.I_scale_analog(false);   // <--- ADD THIS: Use internal voltage reference
//   Serial.print(F("\nTesting driver 5 connection... 0"));
//   result = J5_driver.test_connection();
//    delay(500);
//   if (result) {
//     Serial.println(F("failed!"));
//     Serial.print(F("Likely cause: "));
//     switch (result) {
//       case 1: Serial.println(F("loose connection")); break;
//       case 2: Serial.println(F("Likely cause: no power")); break;
//     }
//     Serial.println(F("Fix the problem and reset board."));
//     // abort();
//   }
//   else{
//       Serial.println("driver 5 Initialized!");
//   }



//   J6_driver.begin();
//   J6_driver.toff(0);
//   J6_driver.blank_time(24);
  
//   J6_driver.rms_current(900); 
//   J6_driver.microsteps(8);
// //   driver.pdn_disable(true);       // <--- ADD THIS: Disables the PDN/UART pin's secondary function
// //   driver.I_scale_analog(false);   // <--- ADD THIS: Use internal voltage reference
//   Serial.print(F("\nTesting driver 6 connection... 0"));
//   result = J6_driver.test_connection();
//    delay(500);
//   if (result) {
//     Serial.println(F("failed!"));
//     Serial.print(F("Likely cause: "));
//     switch (result) {
//       case 1: Serial.println(F("loose connection")); break;
//       case 2: Serial.println(F("Likely cause: no power")); break;
//     }
//     Serial.println(F("Fix the problem and reset board."));
//     // abort();
//   }
//   else{
//       Serial.println("driver 6 Initialized!");
//   }





// }


// Variables to track our raw movement
uint32_t steps_remaining = 000;
bool movement_done = false;
uint64_t itr = 0;
uint32_t start_time = 0;
String inputString = "";
uint16_t ticks = 8000;  // default speed
void loop()
{
    if (itr == 0) {
        start_time = micros();  // start timing
    }

    // ✅ Read user input
   while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
        int steps = 0;
        int new_ticks = 0;

        // Parse two integers from the string
        int parsed = sscanf(inputString.c_str(), "%d %d", &new_ticks, &steps);

        if (parsed >= 1 && steps > 0) {
            steps_remaining = steps;
            Serial.print("✅ Steps: ");
            Serial.println(steps_remaining);
        }

        if (parsed == 2 && new_ticks > 0) {
            ticks = new_ticks;
            Serial.print("⚡ Ticks: ");
            Serial.println(ticks);
        }

        inputString = "";
    } else {
        inputString += c;
    }
}
    if (!stepperJ1) return;
    if(steps_remaining > 0)
    {
        // Grab up to 255 steps for this command chunk
        uint8_t steps_to_send = (steps_remaining > 255) ? 255 : steps_remaining;

        struct stepper_command_s cmd = 
        {
            .ticks = ticks,
            .steps = steps_to_send,
            .count_up = true
        };
        auto result = stepperJ1->addQueueEntry(&cmd, true);
        if (result == AQE_OK) {
            steps_remaining -= steps_to_send;
            
            if (steps_remaining == 0) {
                Serial.println("All steps have been loaded into the queue!");
            }
        }
        
        
    }
    // if(itr % 200000 == 0){
    //     Serial.print("Interation no:");
    //     Serial.println(itr); 
    // }
    
    itr++;
    if (itr == 200000) {
        uint32_t elapsed = micros() - start_time;

        Serial.print("Time for 10000 iterations (us): ");
        Serial.println(elapsed);

        Serial.print("Avg per iteration (us): ");
        Serial.println(elapsed / 200000.0);

        itr = 0;  // restart measurement
    }


}


// // int32_t target_position = 0;
// // bool printed;

// // void loop() {
// //   // Read serial input
// //   while (Serial.available()) {
// //     char c = Serial.read();

// //     if (c == '\n') {
// //       int steps = atoi(inputString.c_str());

// //       target_position += steps;  // relative move

// //       Serial.print("Moving to: ");
// //       Serial.println(target_position);

// //       stepperJ1->moveTo(target_position);

// //       inputString = "";
// //     } else {
// //       inputString += c;
// //     }
// //   }

// //   // Optional: print when motion done
// //   if (stepperJ1->isRunning() == false) {
// //     static bool printed = false;
// //     if (!printed) {
// //       Serial.println("✅ Move complete");
// //       printed = true;
// //     }
// //   } else {
// //     printed = false;
// //   }

// // //   Serial.print("Current Scale Actual: ");
// // //     Serial.println(J3_driver.cs_actual());
// // }


































// // ================= VARIABLES =================
// String inputString = "";
// int32_t target_position = 0;
// bool move_printed = true;

// void setup() {
  

  

//   // 1. Initialize FastAccelStepper for Joint 1
//   engine.init();
//   stepperJ1 = engine.stepperConnectToPin(J2_STEP_PIN);
//   if (stepperJ1) {
//     stepperJ1->setDirectionPin(J2_DIR_PIN);
//     stepperJ1->setAutoEnable(false); // Assumes EN pin is hardwired to GND
//     stepperJ1->setSpeedInHz(15000);   // Safe testing speed
//     stepperJ1->setAcceleration(5000); // Safe testing acceleration
//   }

//   // 2. Initialize Joint 1 (ACTIVE)
//   J1_driver.begin();
//   J1_driver.en_spreadCycle();   // Force high-torque mode
//   J1_driver.rms_current(1800);   // SAFE 800mA limit to prevent instant overheating
//   J1_driver.microsteps(16);
  
//   if (J1_driver.test_connection() == 0) {
//     Serial.println("✅ Driver J1: UART Connection OK!");
//   } else {
//     Serial.println("❌ Driver J1: UART FAILED! Check wiring/power.");
//   }

//   // 3. Initialize Joints 2-6 and explicitly DISABLE them
//   J2_driver.begin(); J2_driver.toff(2); // toff(0) cuts power to the motor coils
//   J3_driver.begin(); J3_driver.toff(2);
//   J4_driver.begin(); J4_driver.toff(2);
//   J5_driver.begin(); J5_driver.toff(2);
//   J6_driver.begin(); J6_driver.toff(2);

//   Serial.println("Drivers 2-6 disabled for safety.");
//   Serial.println("\nSetup Complete. Enter steps to move J1 (e.g., '1600'):");
// }

// void loop() {
//   // Read Serial Input
//   while (Serial.available()) {
//     char c = Serial.read();

//     if (c == '\n') {
//       int steps = atoi(inputString.c_str());
//       target_position += steps;

//       Serial.print("Moving J1 to position: ");
//       Serial.println(target_position);

//       stepperJ1->moveTo(target_position);
      
//       inputString = "";
//       move_printed = false; // Reset completion flag
//     } else {
//       inputString += c;
//     }
//   }

//   // Check if movement is finished
//   if (stepperJ1 && stepperJ1->isRunning() == false) {
//     if (!move_printed) {
//       Serial.println("✅ Move complete");
//       move_printed = true;
//     }
//   }
// }