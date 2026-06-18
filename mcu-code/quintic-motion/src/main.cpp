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
#include <motion_planner.h>
#include <dda_stepper.h>


// ================= UART PINS =================
#define TMC_RX 15
#define TMC_TX 4
#define TMC2_RX 16
#define TMC2_TX 17

#define SERIAL_PORT1 Serial1
#define SERIAL_PORT2 Serial2

BLA::Matrix<6, 1, float> joints;
BLA::Matrix<6, 1, float> joints2;
BLA::Matrix<6, 1, float> FK_result_container;
BLA::Matrix<6, 1, float> targetPose;
BLA::Matrix<6, 1, float> IK_result_container;
BLA::Matrix<4, 4, float> T0_ee_result;

auto stepperQueue = xQueueCreate(10, sizeof(JointFrame));
void feeder(void *pvParameters);


unsigned long previous_loop_time = 0;
const unsigned long LOOP_INTERVAL_US = 1000; // 1000 microseconds = 1000 Hz
void test();
void test2();
void test3();
void test4();
void handleSerial();

// ================= DDA STEPPER CONFIG =================
// Pin order J1..J6, matching Constants::Pins. Direction polarity mirrors the
// FastAccelStepper setDirectionPin() flags used for homing so "positive" means
// the same physical direction in both modes.
static const uint8_t DDA_STEP_PINS[6] = {
    Constants::Pins::J1_STEP_PIN, Constants::Pins::J2_STEP_PIN, Constants::Pins::J3_STEP_PIN,
    Constants::Pins::J4_STEP_PIN, Constants::Pins::J5_STEP_PIN, Constants::Pins::J6_STEP_PIN};
static const uint8_t DDA_DIR_PINS[6] = {
    Constants::Pins::J1_DIR_PIN, Constants::Pins::J2_DIR_PIN, Constants::Pins::J3_DIR_PIN,
    Constants::Pins::J4_DIR_PIN, Constants::Pins::J5_DIR_PIN, Constants::Pins::J6_DIR_PIN};
// dir_high_is_positive per axis (2nd arg passed to setDirectionPin during setup)
static const bool DDA_DIR_POS[6] = {false, true, true, false, false, false};
// steps-per-degree per axis, indexed J1..J6, for the control loop conversions.
static const float DDA_SPD[6] = {
    Constants::Config::J1_STEPS_PER_DEG, Constants::Config::J2_STEPS_PER_DEG,
    Constants::Config::J3_STEPS_PER_DEG, Constants::Config::J4_STEPS_PER_DEG,
    Constants::Config::J5_STEPS_PER_DEG, Constants::Config::J6_STEPS_PER_DEG};
// Control-loop cadence: feed new velocities to the DDA at this rate. Must match
// the Ruckig timestep (ruck(0.001) in motion_planner.h).
constexpr float    CONTROL_DT_S  = 0.001f;
constexpr uint32_t CONTROL_DT_US = 1000;
static uint32_t    last_control_us = 0;

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
    steppers[0] = engine.stepperConnectToPin(Constants::Pins::J1_STEP_PIN);
    steppers[0]->setDirectionPin(Constants::Pins::J1_DIR_PIN, false);
    steppers[0]->setAutoEnable(true);
    steppers[0]->setSpeedInHz(10000);      // steps/sec
    steppers[0]->setAcceleration(6000);     // steps/sec^2
    // steppers[0]->moveTo(15*J4_STEPS_PER_DEG); // Move to 15 degrees to start


    steppers[1] = engine.stepperConnectToPin(Constants::Pins::J2_STEP_PIN);
    steppers[1]->setDirectionPin(Constants::Pins::J2_DIR_PIN, true);
    steppers[1]->setAutoEnable(true);
    steppers[1]->setSpeedInHz(10000);      // steps/sec
    steppers[1]->setAcceleration(6000);     // steps/sec^2

    steppers[2] = engine.stepperConnectToPin(Constants::Pins::J3_STEP_PIN);
    steppers[2]->setDirectionPin(Constants::Pins::J3_DIR_PIN, true);
    steppers[2]->setAutoEnable(true);
    steppers[2]->setSpeedInHz(10000);      // steps/sec
    steppers[2]->setAcceleration(6000);     // steps/sec^2

    steppers[3] = engine.stepperConnectToPin(Constants::Pins::J4_STEP_PIN);
    steppers[3]->setDirectionPin(Constants::Pins::J4_DIR_PIN, false);
    steppers[3]->setAutoEnable(true);
    steppers[3]->setSpeedInHz(10000);      // steps/sec
    steppers[3]->setAcceleration(6000);     // steps/sec^2

    steppers[4] = engine.stepperConnectToPin(Constants::Pins::J5_STEP_PIN);
    steppers[4]->setDirectionPin(Constants::Pins::J5_DIR_PIN, false);
    steppers[4]->setAutoEnable(true);
    steppers[4]->setSpeedInHz(10000);      // steps/sec
    steppers[4]->setAcceleration(6000);     // steps/sec^2  

    steppers[5] = engine.stepperConnectToPin(Constants::Pins::J6_STEP_PIN);
    steppers[5]->setDirectionPin(Constants::Pins::J6_DIR_PIN, false);
    steppers[5]->setAutoEnable(true);
    steppers[5]->setSpeedInHz(10000);      // steps/sec
    steppers[5]->setAcceleration(6000);     // steps/sec^2  



    // homeAxis(1);
    // homeAxis(2);
    // homeAxis(3);
    // homeAxis(4);
    homeAxis(6);
    // homeAxis(5);
    delay(3000); // Wait for homing to complete
    steppers[0]->setCurrentPosition(0);
    steppers[1]->setCurrentPosition(0);
    steppers[2]->setCurrentPosition(0);
    steppers[3]->setCurrentPosition(0);
    steppers[4]->setCurrentPosition(0);
    steppers[5]->setCurrentPosition(0);
    // steppers[4]->moveTo(90*Constants::Config::J5_STEPS_PER_DEG);
    // delay(1000);
    Serial.println("--- System Initialized ---\n");
    Serial.println("t, q, qdot, q_act");



    setupRuckig();

    // ---- Hand the step/dir pins from FastAccelStepper to the DDA generator ----
    // Homing is done; release the MCPWM/RMT routing so we can drive the pins as
    // raw GPIO from the 100 kHz DDA ISR, then start the timer.
    for (int i = 0; i < 6; ++i) {
        steppers[i]->forceStop();
        steppers[i]->detachFromPin();
    }
    dda::init(DDA_STEP_PINS, DDA_DIR_PINS, DDA_DIR_POS);
    for (int i = 0; i < 6; ++i) dda::setPosition(i, 0); // homed == 0
    dda::start();
    last_control_us = micros();
    Serial.println("--- DDA stepper engine started (100 kHz) ---");
}


String inputString = "";

float j1_angle = 0;
float j2_angle = 0;
float j3_angle = 0;
float j4_angle = 0;
float j5_angle = 0;
float j6_angle = 0;




void test(){
DEBUG_PRINTLN(" ----------------------------------- ");

  degToRad(joints);


  // 2. BENCHMARK
  auto start_time = micros();
  // getFK(joints, FK_result_container, T0_ee_result);
  auto end_time = micros();
  

  float avg_time = (float)(end_time - start_time);
  
  DEBUG_PRINTF("Real avg execution time = %.3f us\n", avg_time);

  // Print results to ensure 'res' is actually used
  DEBUG_PRINTLN("FK result:");
  printPose(FK_result_container);
  DEBUG_PRINTLN(" ");

DEBUG_PRINTLN(" ----------------------------------- ");
}




// Test jacobians
void test2()
{

  DEBUG_PRINTLN(" ----------------------------------- ");
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");

  BLA::Matrix<6,6> J;
  BLA::Matrix<6,6> J_copy;
  degToRad(joints);

  auto start_time = micros();
  // fillJacobian(joints, J);
  J_copy = J; // Make a copy of J to ensure it's used in subsequent operations
  auto end_time = micros();
  float avg_time = (float)(end_time - start_time);

  DEBUG_PRINTF("Jacobian filling execution time = %.3f us\n", avg_time);

  DEBUG_PRINTLN("Jacobian Matrix:");
  J.printTo(Serial);
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");


  start_time = micros();
  bool is_non_singular = BLA::Invert(J);
  end_time = micros();
  avg_time = (float)(end_time - start_time);
  DEBUG_PRINTF("Jacobian inversion execution time = %.3f us\n", avg_time);

  if(!is_non_singular) {
    DEBUG_PRINTLN("Warning: Jacobian is singular and cannot be inverted.");
  } else {
    DEBUG_PRINTLN("Jacobian Matrix successfully inverted.");
    J.printTo(Serial);
  }

  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");

  start_time = micros();
  auto res2 = BLA::MatrixTranspose(J_copy);
  end_time = micros();
  avg_time = (float)(end_time - start_time);
  DEBUG_PRINTF("Jacobian transpose execution time = %.3f us\n", avg_time);


  res2.printTo(Serial);


  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");
  // DEBUG_PRINTLN(J);
  DEBUG_PRINTLN(" ----------------------------------- ");
}

// Pos level IK test
void test3(){
DEBUG_PRINTLN(" ----------------------------------- ");
DEBUG_PRINTLN(" ");

  int i;
  
  joints2(0, 0) = steppers[0]->getCurrentPosition() / Constants::Config::J1_STEPS_PER_DEG;
  joints2(1, 0) = steppers[1]->getCurrentPosition() / Constants::Config::J2_STEPS_PER_DEG;
  joints2(2, 0) = steppers[2]->getCurrentPosition() / Constants::Config::J3_STEPS_PER_DEG;
  joints2(3, 0) = steppers[3]->getCurrentPosition() / Constants::Config::J4_STEPS_PER_DEG;
  joints2(4, 0) = steppers[4]->getCurrentPosition() / Constants::Config::J5_STEPS_PER_DEG;
  joints2(5, 0) = steppers[5]->getCurrentPosition() / Constants::Config::J6_STEPS_PER_DEG;


  degToRad(joints2);
  auto start_time = micros();
  IK_result_container = getIK_Pos(joints2, targetPose, i);
  auto end_time = micros();
  auto avg_time = (float)(end_time - start_time);
  DEBUG_PRINTF("IK execution time = %.3f us in %d iterations", avg_time, i);

  DEBUG_PRINTLN("IK Result (radians):");
  IK_result_container.printTo(Serial);
  DEBUG_PRINTLN(" ");

  steppers[0]->moveTo(IK_result_container(0, 0) * Constants::Config::J1_STEPS_PER_DEG);
  steppers[1]->moveTo(IK_result_container(1, 0) * Constants::Config::J2_STEPS_PER_DEG);
  steppers[2]->moveTo(IK_result_container(2, 0) * Constants::Config::J3_STEPS_PER_DEG);
  steppers[3]->moveTo(IK_result_container(3, 0) * Constants::Config::J4_STEPS_PER_DEG);
  steppers[4]->moveTo(IK_result_container(4, 0) * Constants::Config::J5_STEPS_PER_DEG);
  steppers[5]->moveTo(IK_result_container(5, 0) * Constants::Config::J6_STEPS_PER_DEG);
  

DEBUG_PRINTLN(" ");
DEBUG_PRINTLN(" ----------------------------------- ");

}

// RRMC test
Result res;
// home config {0, 334, 288.88, 0, 0, 0}
BLA::Matrix<6,1> T_home = {0.0, 0.245, 0.199, 1.57, 0, 0};
BLA::Matrix<6,1> W1 = {0.15, 0.245, 0.199, 1.57, 0, 0};
BLA::Matrix<6,1> W2 = {-0.050, 0.334, 0.28889, 0, 0, 0};
BLA::Matrix<6,1> diff = (W1 - T_home); 
float dist = BLA::Norm(diff.Submatrix<3,1>(0,0));
BLA::Matrix<6, 1, float> ideal_joint_config;
float t;


bool initialised = false;
int ctr=0;
void test4(){
  DEBUG_PRINTLN(" ----------------------------------- ");
  ctr++;
  if(!initialised){
    handleRuckigTargetUpdate(dist);
    initialised = true;
    ideal_joint_config.Fill(0.0f);


    handleRuckigTargetUpdate(dist);
    initialised = true;
  }



  // 1. update ruckig
  // 2. get twist and target cartesian pose from ruckig by its time parameterised s
  // 3. send those to getIK_RRMC
  // 4. execute move timed to steppers

  res = ruck.update(input, output);  


  float s = output.new_position[0];
  float s_dot = output.new_velocity[0];

  // construct twist
  BLA::Matrix<6,1> target_twist = (diff/dist) * s_dot;
  BLA::Matrix<6,1> target_pose  = T_home + (diff/dist) * s;
  
  joints2(0, 0) = steppers[0]->getCurrentPosition() / Constants::Config::J1_STEPS_PER_DEG;
  joints2(1, 0) = steppers[1]->getCurrentPosition() / Constants::Config::J2_STEPS_PER_DEG;
  joints2(2, 0) = steppers[2]->getCurrentPosition() / Constants::Config::J3_STEPS_PER_DEG;
  joints2(3, 0) = steppers[3]->getCurrentPosition() / Constants::Config::J4_STEPS_PER_DEG;
  joints2(4, 0) = steppers[4]->getCurrentPosition() / Constants::Config::J5_STEPS_PER_DEG;
  joints2(5, 0) = steppers[5]->getCurrentPosition() / Constants::Config::J6_STEPS_PER_DEG;
  
  
  
  
  auto q_delta = getIK_RRMC(ideal_joint_config, target_twist, target_pose);
  if (ctr >= 10)
  {
    Serial.printf("%f, ", t);
    printVecCSV(q_delta);
    ctr = 0;
  }
  

  // steppers[0]->moveTo(q(0, 0) * Constants::Config::J1_STEPS_PER_DEG);
  // steppers[1]->moveTo(q(1, 0) * Constants::Config::J2_STEPS_PER_DEG);
  // steppers[2]->moveTo(q(2, 0) * Constants::Config::J3_STEPS_PER_DEG);
  // steppers[3]->moveTo(q(3, 0) * Constants::Config::J4_STEPS_PER_DEG);
  // steppers[4]->moveTo(q(4, 0) * Constants::Config::J5_STEPS_PER_DEG);
  // steppers[5]->moveTo(q(5, 0) * Constants::Config::J6_STEPS_PER_DEG);
  // 1. Calculate the absolute target step positions
  int32_t target_steps_j1 = q_delta(0, 0) * Constants::Config::J1_STEPS_PER_DEG;
  int32_t target_steps_j2 = q_delta(1, 0) * Constants::Config::J2_STEPS_PER_DEG;
  int32_t target_steps_j3 = q_delta(2, 0) * Constants::Config::J3_STEPS_PER_DEG;
  int32_t target_steps_j4 = q_delta(3, 0) * Constants::Config::J4_STEPS_PER_DEG;
  int32_t target_steps_j5 = q_delta(4, 0) * Constants::Config::J5_STEPS_PER_DEG;
  int32_t target_steps_j6 = q_delta(5, 0) * Constants::Config::J6_STEPS_PER_DEG;



  // 3. Send ONLY the delta to moveTimed
  MoveTimedResultCode r1 = steppers[0]->moveTimed(target_steps_j1, TICKS_PER_S / 1000, NULL, false);
  MoveTimedResultCode r2 = steppers[1]->moveTimed(target_steps_j2, TICKS_PER_S / 1000, NULL, false);
  MoveTimedResultCode r3 = steppers[2]->moveTimed(target_steps_j3, TICKS_PER_S / 1000, NULL, false);
  MoveTimedResultCode r4 = steppers[3]->moveTimed(target_steps_j4, TICKS_PER_S / 1000, NULL, false);
  MoveTimedResultCode r5 = steppers[4]->moveTimed(target_steps_j5, TICKS_PER_S / 1000, NULL, false);
  MoveTimedResultCode r6 = steppers[5]->moveTimed(target_steps_j6, TICKS_PER_S / 1000, NULL, false);


  

  if (r1 <= MOVE_TIMED_OK && r2 <= MOVE_TIMED_OK && r3 <= MOVE_TIMED_OK && r4 <= MOVE_TIMED_OK && r5 <= MOVE_TIMED_OK && r6 <= MOVE_TIMED_OK) {
    // Serial.println("Move timed command accepted for all joints.");
    output.pass_to_input(input);
    noInterrupts();
    steppers[0]->moveTimed(0,0, NULL, true);
    steppers[1]->moveTimed(0,0, NULL, true);
    steppers[2]->moveTimed(0,0, NULL, true);
    steppers[3]->moveTimed(0,0, NULL, true);
    steppers[4]->moveTimed(0,0, NULL, true);
    steppers[5]->moveTimed(0,0, NULL, true);
    interrupts();
  } 

t+=0.001;

}

float q, qdot, prevq;
float idealj5;
bool first_call = true;
// move j5 from 0 to -90 with jerk limited profile to test movetimed function

enum class RobotState {
    Idle,
    Executing,
    Error,
    Homing
};
RobotState currentState;
void loop()
{
  // case 0 --> handle serial --> if serial available --> parse serial and set switch-case to executing
  // case 1 --> 


  switch (currentState)
  {
  case RobotState::Idle:
    handleSerial();
    
    
    break;
  case RobotState::Executing:
    {
      // Hold the control cadence so Ruckig steps at its design dt and the
      // velocity math uses a consistent CONTROL_DT_S.
      uint32_t now = micros();
      if ((uint32_t)(now - last_control_us) < CONTROL_DT_US) break;
      last_control_us += CONTROL_DT_US;

      res = ruck.update(input, output);
      output.pass_to_input(input); // advance trajectory state

      // Feedforward the jerk-limited Ruckig VELOCITY (this is what carries the
      // acceleration profile), plus a SMALL position trim to cancel drift.
      // Do NOT use (target-emitted)/dt: that deadbeat gain multiplies the +-1
      // step quantisation of `emitted` by 1/dt (=1000), which is what produced
      // the huge step-interval jitter and masked the acceleration profile.
      float   ff_sps       = radToDeg(output.new_velocity[0]) * DDA_SPD[5];
      int32_t target_steps = (int32_t)(radToDeg(output.new_position[0]) * DDA_SPD[5]);
      int32_t err          = target_steps - dda::getPosition(5);

      // Stop cleanly once the trajectory is finished AND the DDA has caught up.
      if (res == Result::Finished && err > -2 && err < 2) {
        dda::setVelocity(5, 0.0f);
        currentState = RobotState::Idle;
        break;
      }

      // Gentle drift correction only: a 1-step error adds just POS_TRIM steps/s,
      // not 1000. Closes accumulated error over ~tens of ms without adding noise.
      constexpr float POS_TRIM = 25.0f; // [1/s]
      dda::setVelocity(5, ff_sps + POS_TRIM * (float)err);
    }
    break;

  
  }
}




void handleSerial()
{
  if (Serial.available()) {
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
        targetPose(0, 0) = payload[6]/1000.0f; 
        targetPose(1, 0) = payload[7]/1000.0f; 
        targetPose(2, 0) = payload[8]/1000.0f; 
        targetPose(3, 0) = degToRad(payload[9]);   // Assuming degrees from Python
        targetPose(4, 0) = degToRad(payload[10]); 
        targetPose(5, 0) = degToRad(payload[11]);

        Serial.println(" ----------------------------------- ");
        // Debug Print
        Serial.print("Angles -> J1: "); Serial.print(j1_angle, 5);
        Serial.print(" | J2: "); Serial.print(j2_angle, 5);
        Serial.print(" | J3: "); Serial.print(j3_angle, 5);
        Serial.print(" | J4: "); Serial.print(j4_angle, 5);
        Serial.print(" | J5: "); Serial.print(j5_angle, 5);
        Serial.print(" | J6: "); Serial.println(j6_angle, 5);
        
        Serial.print("Pose   -> X: "); Serial.print(targetPose(0, 0), 5);
        Serial.print(" | Y: "); Serial.print(targetPose(1, 0), 5);
        Serial.print(" | Z: "); Serial.print(targetPose(2, 0), 5);
        Serial.print(" | Rx: "); Serial.print(targetPose(3, 0), 5);
        Serial.print(" | Ry: "); Serial.print(targetPose(4, 0), 5);
        Serial.print(" | Rz: "); Serial.println(targetPose(5, 0), 5);
        Serial.println(" ----------------------------------- ");

        // Update Matrix
        joints(0, 0) = j1_angle;
        joints(1, 0) = j2_angle;
        joints(2, 0) = j3_angle;
        joints(3, 0) = j4_angle;
        joints(4, 0) = j5_angle;
        joints(5, 0) = j6_angle;

        currentState = RobotState::Executing;

        // Do NOT seed current_position from the stepper readback: this is an
        // open-loop planner and Ruckig already holds its own last position
        // (kept up to date via output.pass_to_input). Reading the hardware here
        // couples the planner to any physical drift/runaway and produces a huge
        // first delta -> ErrorTicksTooLow. Only set the new target.
        auto trp = degToRad(j6_angle);
        Serial.printf("Current J6: %f | Target J6: %f\n", input.current_position[0], trp);
        input.target_position[0] = trp;

        // test();
        // test2();
        // test3();

        // Apply motor movements (Assuming test() dictates changes, or keep manual tracking)
        // steppers[0]->moveTo(j1_angle * Constants::Config::J1_STEPS_PER_DEG);
        // steppers[1]->moveTo(j2_angle * Constants::Config::J2_STEPS_PER_DEG);
        // steppers[2]->moveTo(j3_angle * Constants::Config::J3_STEPS_PER_DEG);
        // steppers[3]->moveTo(j4_angle * Constants::Config::J4_STEPS_PER_DEG);
        // steppers[4]->moveTo(j5_angle * Constants::Config::J5_STEPS_PER_DEG);
        // steppers[5]->moveTo(j6_angle * Constants::Config::J6_STEPS_PER_DEG);
      }

      // Clear string for next command
      inputString = "";
    } 
    else if (c != '\r') { 
      inputString += c;
    }
  }
}


