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

    initJoints(1, 0, 0, 0, 0, 0, 0); 
    pinMode(L1, INPUT_PULLUP);
    pinMode(L2, INPUT_PULLUP);
    pinMode(L3, INPUT_PULLUP);
    pinMode(L4, INPUT_PULLUP);
    pinMode(L5, INPUT_PULLUP); // Pin 18 supports INPUT_PULLUP if needed, but keeping as INPUT to match the rest
    pinMode(L6, INPUT_PULLUP);

    engine.init();
    
}


void loop() {

  int stateL1 = digitalRead(L1);
  int stateL2 = digitalRead(L2);
  int stateL3 = digitalRead(L3);
  int stateL4 = digitalRead(L4);
  int stateL5 = digitalRead(L5);
  int stateL6 = digitalRead(L6);

  // Print the states to the Serial Monitor
  Serial.print(" | J1: "); Serial.print(stateL1);
  Serial.print(" | J2: "); Serial.print(stateL2);
  Serial.print(" | J3: "); Serial.print(stateL3);
  Serial.print(" | J4 "); Serial.print(stateL4);
  Serial.print(" | J5 "); Serial.print(stateL5);
  Serial.print(" | J6 "); Serial.println(stateL6);

}




