#include <Arduino.h>
#include "FastAccelStepper.h"


// ================= LIMIT SWITCHES =================
#define L1 36 // Shoulder (J2) -- Normally high
#define L2 39 // Elbow (J3) -- Normally high
#define L3 34 // Wrist2 (J5) -- Normally high
#define L4 35 // Wrist3 (J6) -- Normally low
#define L5 18 // Wrist1 (J4) -- Normally low
#define L6 5  // Base (J1) -- !! NOT INSTALLED YET


// ================= JOINT DEFINITIONS =================
// Joint 1 - Base
#define J1_STEP_PIN 13
#define J1_DIR_PIN  12

// Joint 2 - Shoulder
#define J2_STEP_PIN 22
#define J2_DIR_PIN  23

// Joint 3 - Elbow
#define J3_STEP_PIN 14
#define J3_DIR_PIN  27

// Joint 4 - Wrist 1
#define J4_STEP_PIN 26
#define J4_DIR_PIN  25

// Joint 5 - Wrist 2
#define J5_STEP_PIN 33
#define J5_DIR_PIN  32

// Joint 6 - Wrist 3 (SAFEST TEST)
#define J6_STEP_PIN 19
#define J6_DIR_PIN  21



// ================= STEPPER ENGINE =================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperJ6 = NULL;


void setup() {
  Serial.begin(115200);

  // Initialize limit switch pins
  pinMode(L1, INPUT);  // pulled up with 10k resistor to 3.3v
  pinMode(L2, INPUT);  // pulled up with 10k resistor to 3.3v
  pinMode(L3, INPUT);  // pulled up with 10k resistor to 3.3v
  pinMode(L4, INPUT);  // pulled up with 10k resistor to 3.3v
  pinMode(L5, INPUT_PULLUP);  // important
  pinMode(L6, INPUT_PULLUP);  // important

  // Initialize stepper engine
  engine.init();

  // Connect Joint 6 stepper
  stepperJ6 = engine.stepperConnectToPin(J6_STEP_PIN);

  if (stepperJ6) {
    stepperJ6->setDirectionPin(J6_DIR_PIN);
    // stepperJ6->setEnablePin(255); // no enable pin
    stepperJ6->setAutoEnable(true);

    // ================= KEY SETTINGS =================
    stepperJ6->setAcceleration(50000);     // steps/sec^2
    // stepperJ6->set  setMaxSpeed(2000);         // steps/sec
    stepperJ6->setSpeedInHz(20000);
    Serial.println("J6 Stepper initialized");
    // stepperJ6
    // Move test: 2000 steps forward with accel
    // stepperJ6->move(2000);
  } else {
    Serial.println("Stepper not connected!");
  }
}

void loop() {
  // Example: back and forth motion
  static bool forward = true;
  
  int s1 = digitalRead(L1);
  int s2 = digitalRead(L2);
  int s3 = digitalRead(L3);
  int s4 = digitalRead(L4);
  int s5 = digitalRead(L5);
  int s6 = digitalRead(L6);

  Serial.print("L1=");
  Serial.print(s1);
  Serial.print("  L2=");
  Serial.print(s2);
  Serial.print("  L3=");
  Serial.print(s3);
  Serial.print("  L4=");
  Serial.print(s4);
  Serial.print("  L5=");
  Serial.print(s5);
  Serial.print("  L6=");
  Serial.println(s6);  // newline at the end

  if (stepperJ6 && !stepperJ6->isRunning()) {
    // Serial.println("Stepper moving!");
    if (forward) {
      stepperJ6->move(1000);
    } else {
      stepperJ6->move(-1000);
    }
    forward = !forward;

    delay(50); // small pause
  }
}