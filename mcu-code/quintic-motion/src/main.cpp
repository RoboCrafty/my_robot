#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <motor_init.h>
#include <math.h>
#include <ArduinoEigen.h>
#include <homing_sequence.h>
#include <kinematics.h>
#include <BasicLinearAlgebra.h>
#include <helper_functions.h>
#include <structs.h>
// ================= UART PINS =================
#define TMC_RX 15
#define TMC_TX 4
#define TMC2_RX 16
#define TMC2_TX 17

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2

BLA::Matrix<6, 1, float> joints;
Pose FK_result_container;
Pose targetPose;
BLA::Matrix<6, 1, float> IK_result_container;
BLA::Matrix<4, 4, float> T0_ee_result;
void test();
void test2();
void test3();
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
    initLimitSwitches();
    

    engine.init();
    stepper1 = engine.stepperConnectToPin(Constants::Pins::J1_STEP_PIN);
    stepper1->setDirectionPin(Constants::Pins::J1_DIR_PIN, false);
    stepper1->setAutoEnable(true);
    stepper1->setSpeedInHz(10000);      // steps/sec
    stepper1->setAcceleration(6000);     // steps/sec^2
    // stepper1->moveTo(15*J4_STEPS_PER_DEG); // Move to 15 degrees to start


    stepper2 = engine.stepperConnectToPin(Constants::Pins::J2_STEP_PIN);
    stepper2->setDirectionPin(Constants::Pins::J2_DIR_PIN, true);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(10000);      // steps/sec
    stepper2->setAcceleration(6000);     // steps/sec^2

    stepper3 = engine.stepperConnectToPin(Constants::Pins::J3_STEP_PIN);
    stepper3->setDirectionPin(Constants::Pins::J3_DIR_PIN, true);
    stepper3->setAutoEnable(true);
    stepper3->setSpeedInHz(10000);      // steps/sec
    stepper3->setAcceleration(6000);     // steps/sec^2

    stepper4 = engine.stepperConnectToPin(Constants::Pins::J4_STEP_PIN);
    stepper4->setDirectionPin(Constants::Pins::J4_DIR_PIN, false);
    stepper4->setAutoEnable(true);
    stepper4->setSpeedInHz(10000);      // steps/sec
    stepper4->setAcceleration(6000);     // steps/sec^2

    stepper5 = engine.stepperConnectToPin(Constants::Pins::J5_STEP_PIN);
    stepper5->setDirectionPin(Constants::Pins::J5_DIR_PIN, false);
    stepper5->setAutoEnable(true);
    stepper5->setSpeedInHz(10000);      // steps/sec
    stepper5->setAcceleration(6000);     // steps/sec^2  

    stepper6 = engine.stepperConnectToPin(Constants::Pins::J6_STEP_PIN);
    stepper6->setDirectionPin(Constants::Pins::J6_DIR_PIN, true);
    stepper6->setAutoEnable(true);
    stepper6->setSpeedInHz(10000);      // steps/sec
    stepper6->setAcceleration(6000);     // steps/sec^2  



    homeAxis(1);
    homeAxis(2);
    homeAxis(3);
    homeAxis(4);
    homeAxis(5);
    delay(2000); // Wait for homing to complete
    stepper1->setCurrentPosition(0);
    stepper2->setCurrentPosition(0);
    stepper3->setCurrentPosition(0);
    stepper4->setCurrentPosition(0);
    stepper5->setCurrentPosition(0);
    stepper6->setCurrentPosition(0);
    Serial.println("--- System Initialized ---\n");

    
}


String inputString = "";

float j1_angle = 0;
float j2_angle = 0;
float j3_angle = 0;
float j4_angle = 0;
float j5_angle = 0;
float j6_angle = 0;


void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      int lastCommaIndex = 0;
      int nextCommaIndex = 0;
      
      // Array to hold 12 values: 6 joints + 6 Cartesian pose values
      float payload[12] = {0};
      bool parseSuccess = true;

      for (int i = 0; i < 12; i++) {
        // For the last element, look for the end of the string
        if (i == 11) {
          nextCommaIndex = inputString.length();
        } else {
          nextCommaIndex = inputString.indexOf(',', lastCommaIndex);
        }

        // If we can't find a comma before the 12th element, data is malformed
        if (nextCommaIndex == -1 && i < 11) {
          Serial.println("Error: Incomplete data received. Expected 12 values.");
          parseSuccess = false;
          break;
        }

        // Extract substring and convert to float
        String valueStr = inputString.substring(lastCommaIndex, nextCommaIndex);
        payload[i] = valueStr.toFloat();

        lastCommaIndex = nextCommaIndex + 1;
      }

      // Only move motors and update pose if we successfully parsed all 12 values
      if (parseSuccess) {
        // --- 1. Map Joint Angles ---
        j1_angle = payload[0];
        j2_angle = payload[1];
        j3_angle = payload[2];
        j4_angle = payload[3];
        j5_angle = payload[4];
        j6_angle = payload[5];

        // --- 2. Map Cartesian Pose ---
        targetPose.x  = payload[6]/1000.0f; 
        targetPose.y  = payload[7]/1000.0f; 
        targetPose.z  = payload[8]/1000.0f; 
        targetPose.rx = degToRad(payload[9]);   // Assuming degrees from Python
        targetPose.ry = degToRad(payload[10]); 
        targetPose.rz = degToRad(payload[11]);

        Serial.println(" ----------------------------------- ");
        // Debug Print
        Serial.print("Angles -> J1: "); Serial.print(j1_angle);
        Serial.print(" | J2: "); Serial.print(j2_angle);
        Serial.print(" | J3: "); Serial.print(j3_angle);
        Serial.print(" | J4: "); Serial.print(j4_angle);
        Serial.print(" | J5: "); Serial.print(j5_angle);
        Serial.print(" | J6: "); Serial.println(j6_angle);
        
        Serial.print("Pose   -> X: "); Serial.print(targetPose.x);
        Serial.print(" | Y: "); Serial.print(targetPose.y);
        Serial.print(" | Z: "); Serial.print(targetPose.z);
        Serial.print(" | Rx: "); Serial.print(targetPose.rx);
        Serial.print(" | Ry: "); Serial.print(targetPose.ry);
        Serial.print(" | Rz: "); Serial.println(targetPose.rz);
        Serial.println(" ----------------------------------- ");

        // Update Matrix
        joints(0, 0) = j1_angle;
        joints(1, 0) = j2_angle;
        joints(2, 0) = j3_angle;
        joints(3, 0) = j4_angle;
        joints(4, 0) = j5_angle;
        joints(5, 0) = j6_angle;

        // test();
        // test2();
        test3();

        // Apply motor movements (Assuming test() dictates changes, or keep manual tracking)
        // stepper1->moveTo(j1_angle * Constants::Config::J1_STEPS_PER_DEG);
        // stepper2->moveTo(j2_angle * Constants::Config::J2_STEPS_PER_DEG);
        // stepper3->moveTo(j3_angle * Constants::Config::J3_STEPS_PER_DEG);
        // stepper4->moveTo(j4_angle * Constants::Config::J4_STEPS_PER_DEG);
        // stepper5->moveTo(j5_angle * Constants::Config::J5_STEPS_PER_DEG);
        // stepper6->moveTo(j6_angle * Constants::Config::J6_STEPS_PER_DEG);
      }

      // Clear string for next command
      inputString = "";
    } 
    else if (c != '\r') { 
      inputString += c;
    }
  }
}




void test(){
Serial.println(" ----------------------------------- ");

  degToRad(joints);


  // 2. BENCHMARK
  auto start_time = micros();
  getFK(joints, FK_result_container, T0_ee_result);
  auto end_time = micros();
  

  float avg_time = (float)(end_time - start_time);
  
  Serial.printf("Real avg execution time = %.3f us\n", avg_time);

  // Print results to ensure 'res' is actually used
  Serial.println("FK result:");
  printPose(FK_result_container);
  Serial.println(" ");

Serial.println(" ----------------------------------- ");
}





void test2()
{

  Serial.println(" ----------------------------------- ");
  Serial.println(" ");
  Serial.println(" ");

  BLA::Matrix<6,6> J;
  BLA::Matrix<6,6> J_copy;
  degToRad(joints);

  auto start_time = micros();
  fillJacobian(joints, J);
  J_copy = J; // Make a copy of J to ensure it's used in subsequent operations
  auto end_time = micros();
  float avg_time = (float)(end_time - start_time);

  Serial.printf("Jacobian filling execution time = %.3f us\n", avg_time);

  Serial.println("Jacobian Matrix:");
  J.printTo(Serial);
  Serial.println(" ");
  Serial.println(" ");


  start_time = micros();
  bool is_non_singular = BLA::Invert(J);
  end_time = micros();
  avg_time = (float)(end_time - start_time);
  Serial.printf("Jacobian inversion execution time = %.3f us\n", avg_time);

  if(!is_non_singular) {
    Serial.println("Warning: Jacobian is singular and cannot be inverted.");
  } else {
    Serial.println("Jacobian Matrix successfully inverted.");
    J.printTo(Serial);
  }

  Serial.println(" ");
  Serial.println(" ");

  start_time = micros();
  auto res2 = BLA::MatrixTranspose(J_copy);
  end_time = micros();
  avg_time = (float)(end_time - start_time);
  Serial.printf("Jacobian transpose execution time = %.3f us\n", avg_time);


  res2.printTo(Serial);


  Serial.println(" ");
  Serial.println(" ");
  // Serial.println(J);
  Serial.println(" ----------------------------------- ");
}

void test3(){
Serial.println(" ----------------------------------- ");
Serial.println(" ");

  int i;
  auto start_time = micros();
  IK_result_container = getIK(joints, &targetPose, i);
  auto end_time = micros();
  auto avg_time = (float)(end_time - start_time);
  Serial.printf("IK execution time = %.3f us in %d iterations", avg_time, i);

  Serial.println("IK Result (radians):");
  IK_result_container.printTo(Serial);
  Serial.println(" ");

  stepper1->moveTo(IK_result_container(0, 0) * Constants::Config::J1_STEPS_PER_DEG);
  stepper2->moveTo(IK_result_container(1, 0) * Constants::Config::J2_STEPS_PER_DEG);
  stepper3->moveTo(IK_result_container(2, 0) * Constants::Config::J3_STEPS_PER_DEG);
  stepper4->moveTo(IK_result_container(3, 0) * Constants::Config::J4_STEPS_PER_DEG);
  stepper5->moveTo(IK_result_container(4, 0) * Constants::Config::J5_STEPS_PER_DEG);
  stepper6->moveTo(IK_result_container(5, 0) * Constants::Config::J6_STEPS_PER_DEG);
  

Serial.println(" ");
Serial.println(" ----------------------------------- ");

}