#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include "motor_init.h"
#include <math.h>
#include <ArduinoEigen.h>

// ================= LIMIT SWITCHES =================
#define L1 5  // Base     (J1)      -- Normally low
#define L2 36 // Shoulder (J2)      -- Normally high
#define L3 39 // Elbow    (J3)      -- Normally high
#define L4 18 // Wrist1   (J4)      -- Normally low
#define L5 34 // Wrist2   (J5)      -- Normally high
#define L6 35 // Wrist3   (J6)      -- Normally low


// ================= UART PINS =================
#define TMC_RX 15
#define TMC_TX 4
#define TMC2_RX 16
#define TMC2_TX 17

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2


void setup()
{   
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(1000);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 1, 1, 1, 1, 1, 1); 
    pinMode(L1, INPUT_PULLUP);
    pinMode(L2, INPUT_PULLUP);
    pinMode(L3, INPUT_PULLUP);
    pinMode(L4, INPUT_PULLUP);
    pinMode(L5, INPUT_PULLUP); // Pin 18 supports INPUT_PULLUP if needed, but keeping as INPUT to match the rest
    pinMode(L6, INPUT_PULLUP);

    engine.init();
    stepper1 = engine.stepperConnectToPin(J1_STEP_PIN);
    stepper1->setDirectionPin(J1_DIR_PIN, true);
    stepper1->setAutoEnable(true);
    stepper1->setSpeedInHz(10000);      // steps/sec
    stepper1->setAcceleration(6000);     // steps/sec^2
    // stepper1->moveTo(15*J4_STEPS_PER_DEG); // Move to 15 degrees to start


    stepper2 = engine.stepperConnectToPin(J2_STEP_PIN);
    stepper2->setDirectionPin(J2_DIR_PIN, true);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(10000);      // steps/sec
    stepper2->setAcceleration(6000);     // steps/sec^2

    stepper3 = engine.stepperConnectToPin(J3_STEP_PIN);
    stepper3->setDirectionPin(J3_DIR_PIN, true);
    stepper3->setAutoEnable(true);
    stepper3->setSpeedInHz(10000);      // steps/sec
    stepper3->setAcceleration(6000);     // steps/sec^2

    stepper4 = engine.stepperConnectToPin(J4_STEP_PIN);
    stepper4->setDirectionPin(J4_DIR_PIN, true);
    stepper4->setAutoEnable(true);
    stepper4->setSpeedInHz(10000);      // steps/sec
    stepper4->setAcceleration(6000);     // steps/sec^2

    stepper5 = engine.stepperConnectToPin(J5_STEP_PIN);
    stepper5->setDirectionPin(J5_DIR_PIN, true);
    stepper5->setAutoEnable(true);
    stepper5->setSpeedInHz(10000);      // steps/sec
    stepper5->setAcceleration(6000);     // steps/sec^2  

    stepper6 = engine.stepperConnectToPin(J6_STEP_PIN);
    stepper6->setDirectionPin(J6_DIR_PIN, true);
    stepper6->setAutoEnable(true);
    stepper6->setSpeedInHz(10000);      // steps/sec
    stepper6->setAcceleration(6000);     // steps/sec^2  

    
}


String inputString = "";

float j1_angle = 0;
float j2_angle = 0;
float j3_angle = 0;
float j4_angle = 0;
float j5_angle = 0;
float j6_angle = 0;

// Joint limits
const float J1_MIN = 0;
const float J1_MAX = 360;

const float J2_MIN = 0;
const float J2_MAX = 360;
void loop() {
while (Serial.available()) {
  char c = Serial.read();

  if (c == '\n') {
    // We will track the start and end positions of each value between commas
    int lastCommaIndex = 0;
    int nextCommaIndex = 0;
    
    // Array to hold our 6 parsed angles
    float angles[6] = {0, 0, 0, 0, 0, 0};
    bool parseSuccess = true;

    for (int i = 0; i < 6; i++) {
      // For the last element, there is no trailing comma, look for the end of the string
      if (i == 5) {
        nextCommaIndex = inputString.length();
      } else {
        nextCommaIndex = inputString.indexOf(',', lastCommaIndex);
      }

      // If we can't find a comma before the 6th element, the data is malformed
      if (nextCommaIndex == -1 && i < 5) {
        Serial.println("Error: Incomplete data received.");
        parseSuccess = false;
        break;
      }

      // Extract the substring and convert to float
      String valueStr = inputString.substring(lastCommaIndex, nextCommaIndex);
      angles[i] = valueStr.toFloat();

      // Move the pointer past the current comma for the next iteration
      lastCommaIndex = nextCommaIndex + 1;
    }

    // Only move the motors if we successfully parsed all 6 values
    if (parseSuccess) {
      j1_angle = angles[0];
      j2_angle = angles[1];
      j3_angle = angles[2];
      j4_angle = angles[3];
      j5_angle = angles[4];
      j6_angle = angles[5];

      // Debug Print
      Serial.print("Parsed Angles -> J1: "); Serial.print(j1_angle);
      Serial.print(" | J2: "); Serial.print(j2_angle);
      Serial.print(" | J3: "); Serial.print(j3_angle);
      Serial.print(" | J4: "); Serial.print(j4_angle);
      Serial.print(" | J5: "); Serial.print(j5_angle);
      Serial.print(" | J6: "); Serial.println(j6_angle);

      // Apply your motor movements
      stepper1->moveTo(j1_angle * J1_STEPS_PER_DEG);
      stepper2->moveTo(j2_angle * J2_STEPS_PER_DEG);
      stepper3->moveTo(j3_angle * J3_STEPS_PER_DEG);
      stepper4->moveTo(j4_angle * J4_STEPS_PER_DEG);
      stepper5->moveTo(j5_angle * J5_STEPS_PER_DEG);
      stepper6->moveTo(j6_angle * J6_STEPS_PER_DEG);
    }

    // Clear the string for the next command
    inputString = "";
  } 
  else if (c != '\r') { // Ignore carriage returns if sent by some serial monitors
    inputString += c;
  }
}

}




