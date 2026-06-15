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

auto stepperQueue = xQueueCreate(20, sizeof(JointFrame));
void feeder(void *pvParameters);
// Set true while a trajectory is being produced. The feeder uses it to know
// the TRUE end of a motion (vs a transient queue hiccup) so it only flushes a
// final partial batch window when the motion has actually finished.
volatile bool g_motionActive = false;


unsigned long previous_loop_time = 0;
const unsigned long LOOP_INTERVAL_US = 1000; // 1000 microseconds = 1000 Hz
void test();
void test2();
void test3();
void test4();
void handleSerial();

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



    homeAxis(1);
    homeAxis(2);
    homeAxis(3);
    homeAxis(4);
    homeAxis(6);
    homeAxis(5);
    delay(3000); // Wait for homing to complete
    steppers[0]->setCurrentPosition(0);
    steppers[1]->setCurrentPosition(0);
    steppers[2]->setCurrentPosition(0);
    steppers[3]->setCurrentPosition(0);
    steppers[4]->setCurrentPosition(0);
    steppers[5]->setCurrentPosition(0);

    // CRITICAL: homeAxis() drives each stepper with the RAMP generator
    // (runForward / forceStopAndNewPosition / moveTo). After forceStop the
    // stepper IGNORES every subsequent moveTimed() call (see FastAccelStepper
    // issue #299, reported by Matoseb / confirmed by gin66). That makes the
    // motor never move AND makes our feeder's retry loop spin forever on the
    // ignored (busy) return -> the queue fills -> loop() is stuck in Executing
    // -> no more serial -> "frozen". We must hand each stepper from ramp mode
    // into the moveTimed state machine by priming an empty (start=false) timed
    // window, exactly as the official moveTimed example does.
    for (int i = 0; i < 6; i++) {
        steppers[i]->stopMove();                 // ensure ramp generator is idle
        steppers[i]->moveTimed(0, TICKS_PER_S / 240, NULL, false); // prime moveTimed
    }
    // steppers[4]->moveTo(90*Constants::Config::J5_STEPS_PER_DEG);
    // delay(1000);
    Serial.println("--- System Initialized ---\n");
    Serial.println("t, q, qdot, q_act");



    setupRuckig();

    xTaskCreatePinnedToCore(feeder, "Queue Feeder", 4096, NULL, 5, NULL, 0);

    
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
// Continuously integrated joint configuration (radians). Initialised to the
// homed configuration (all zero). NOT reset between moves so chained segments
// start from the true current configuration.
BLA::Matrix<6, 1, float> ideal_joint_config = {0, 0, 0, 0, 0, 0};
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

// ===================== RRMC PRODUCER =====================
// T_home holds the segment START pose, which is also the robot's persistent
// current Cartesian pose: it begins at the homed pose and is advanced to each
// goal as moves complete. g_goalPose is the active segment's target.
static BLA::Matrix<6, 1, float> g_goalPose = {0, 0, 0, 0, 0, 0};

// Start a resolved-rate straight-line segment from the current Cartesian pose
// (T_home) to 'goalPose'. Sets up the single-DOF Ruckig path parameter s:
// 0 -> dist for THIS segment. ideal_joint_config is intentionally NOT reset --
// it holds the continuously integrated joint configuration so chained moves
// start from the true current configuration (no jump back to home).
static void initRRMC(const BLA::Matrix<6, 1, float>& goalPose)
{
  g_goalPose = goalPose;
  diff = goalPose - T_home;                 // full 6D pose delta (orientation incl.)

  // Path length drives the time scaling. Use translation distance; for a pure
  // reorientation (no translation) fall back to rotation magnitude so dist is
  // never zero. Endpoint is exact either way because (diff/dist)*dist == diff.
  float transDist = BLA::Norm(diff.Submatrix<3, 1>(0, 0));
  float rotDist   = BLA::Norm(diff.Submatrix<3, 1>(3, 0));
  dist = (transDist > 1e-4f) ? transDist : rotDist;

  input.current_position     = {0.0};       // path parameter s starts at 0
  input.current_velocity     = {0.0};
  input.current_acceleration = {0.0};
  input.target_position      = {dist};      // s travels 0 -> path length
  input.target_velocity      = {0.0};
  input.target_acceleration  = {0.0};
  t = 0.0f;
}

// Produce ONE RRMC frame: advance Ruckig's scalar path parameter s, map it to a
// full 6D Cartesian pose + twist (orientation included via diff(3..5)), run
// resolved-rate IK to integrate the joint configuration, and write the
// ABSOLUTE joint step setpoints into 'frame'. The feeder turns these absolute
// setpoints into per-window deltas. Returns true once the trajectory finished.
static bool produceRRMCFrame(JointFrame& frame)
{
  res = ruck.update(input, output);

  float s     = output.new_position[0];     // time-parameterised path length
  float s_dot = output.new_velocity[0];     // path speed

  // Scalar path parameter -> full 6D Cartesian pose & twist.
  BLA::Matrix<6, 1, float> target_twist = (diff / dist) * s_dot;
  BLA::Matrix<6, 1, float> target_pose  = T_home + (diff / dist) * s;

  // Resolved-rate IK. Integrates ideal_joint_config (radians) in place; the
  // returned per-step delta (degrees) is not needed here -- we publish the
  // integrated ABSOLUTE config instead.
  getIK_RRMC(ideal_joint_config, target_twist, target_pose);

  frame.q_steps[0] = radToDeg(ideal_joint_config(0, 0)) * Constants::Config::J1_STEPS_PER_DEG;
  frame.q_steps[1] = radToDeg(ideal_joint_config(1, 0)) * Constants::Config::J2_STEPS_PER_DEG;
  frame.q_steps[2] = radToDeg(ideal_joint_config(2, 0)) * Constants::Config::J3_STEPS_PER_DEG;
  frame.q_steps[3] = radToDeg(ideal_joint_config(3, 0)) * Constants::Config::J4_STEPS_PER_DEG;
  frame.q_steps[4] = radToDeg(ideal_joint_config(4, 0)) * Constants::Config::J5_STEPS_PER_DEG;
  frame.q_steps[5] = radToDeg(ideal_joint_config(5, 0)) * Constants::Config::J6_STEPS_PER_DEG;

  if (res == Result::Finished) {
    T_home = g_goalPose;   // segment complete: current pose becomes the goal
    return true;
  }
  return false;
}

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
    if (uxQueueSpacesAvailable(stepperQueue) > 0)
      {
        // Run one resolved-rate step: Ruckig advances the scalar path
        // parameter, RRMC maps it to absolute joint setpoints for all 6 axes.
        JointFrame j_send;
        bool finished = produceRRMCFrame(j_send);

        // Producer is the SOLE sender and we just confirmed space, so this send
        // never blocks. Advance Ruckig ONLY after the frame is safely queued.
        // Do NOT re-run produceRRMCFrame on a failed send: getIK_RRMC
        // integrates ideal_joint_config in place, so a re-run would
        // double-integrate and corrupt the trajectory.
        if (xQueueSend(stepperQueue, &j_send, 0) == pdPASS)
        {
          output.pass_to_input(input); // advance trajectory state safely
        }
        else
        {
          Serial.println("Queue full");
        }

        if (finished)
        {
          g_motionActive = false;      // tell feeder to flush the final partial window
          currentState = RobotState::Idle;
        }
      }
    // If no space, do nothing. We will catch up on the next loop iteration.
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

        // Keep this SHORT. handleSerial runs on the producer core; a long
        // Serial.print block here stalls the producer for tens of ms (TX buffer
        // drain at 115200 baud), starving the pipeline right as a move starts.
        Serial.printf("CMD pose X:%.3f Y:%.3f Z:%.3f Rx:%.3f Ry:%.3f Rz:%.3f\n",
                      targetPose(0, 0), targetPose(1, 0), targetPose(2, 0),
                      targetPose(3, 0), targetPose(4, 0), targetPose(5, 0));

        // Update Matrix
        joints(0, 0) = j1_angle;
        joints(1, 0) = j2_angle;
        joints(2, 0) = j3_angle;
        joints(3, 0) = j4_angle;
        joints(4, 0) = j5_angle;
        joints(5, 0) = j6_angle;

        // Start a resolved-rate Cartesian straight-line move from the current
        // pose (T_home) to the parsed target pose (already meters / radians).
        // Guard against a zero-length command so we never divide by zero or
        // get stuck in Executing with nothing to do.
        BLA::Matrix<6, 1, float> goalPose = targetPose;
        BLA::Matrix<6, 1, float> preview  = goalPose - T_home;
        float previewTrans = BLA::Norm(preview.Submatrix<3, 1>(0, 0));
        float previewRot   = BLA::Norm(preview.Submatrix<3, 1>(3, 0));
        if (previewTrans < 1e-4f && previewRot < 1e-4f) {
          Serial.println("Target pose ~= current pose; nothing to do.");
          g_motionActive = false;
          currentState = RobotState::Idle;
        } else {
          initRRMC(goalPose);
          g_motionActive = true;
          currentState = RobotState::Executing;
          Serial.printf("RRMC start dist %.3f\n", dist);
        }

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




JointFrame emitted = {{0,0,0,0,0,0}};          // last absolute position absorbed (delta baseline)
// Per-joint leftover ticks. moveTimed uses integer rate = duration/steps, so
// actual_duration = rate*steps <= duration. The unspent (duration - actual)
// ticks are carried into the next window so the cumulative timeline stays
// locked to real time (same trick as the official moveTimed example).
uint32_t feederDrift[6] = {0, 0, 0, 0, 0, 0};

// ---- Batching state ------------------------------------------------------
// At low step rates (~1 step/ms) emitting one moveTimed window per 1ms frame
// forces the step count to integers (1,2,1,2..), which makes the per-window
// velocity swing +-50% every ms -> audible buzz. Instead we ACCUMULATE the
// per-joint step deltas (and the elapsed time) across several producer frames
// and emit a single window only once enough steps have piled up. The library
// then spaces those many steps evenly at one constant interval inside the
// window -- exactly like the ramp generator -- so the motion is smooth.
int32_t  accSteps[6] = {0, 0, 0, 0, 0, 0};      // steps accumulated since last flush
uint32_t accTicks = 0;                          // window duration accumulated since last flush
static const int32_t  BATCH_MIN_STEPS = 16;     // flush once the busiest joint has this many steps
static const uint32_t BATCH_MAX_TICKS = TICKS_PER_S / 40; // ...or after ~25ms (latency cap / slow-motion flush)

// ---- Keep-alive ----------------------------------------------------------
// CRITICAL: the HW queue must never run fully empty. If it does, the driver
// calls init_stop(), which halts the MCPWM timer but leaves its counter value
// frozen. isReadyForCommands() then latches false (timer_value > 1) and EVERY
// subsequent moveTimed() returns DeviceNotReady (code 4) -- a state the public
// API cannot recover from (addQueueEntry bails before it can restart the
// timer). The motor stops and the feeder/loop freeze. To prevent this we top
// up each idle stepper with short 0-step PAUSE windows so isRunning() stays
// true and the timer never latches. A 0-step pause holds position exactly (no
// motion, no lost steps) -- safe both between commands and during a transient
// producer underrun.
static const uint32_t KEEPALIVE_TICKS     = TICKS_PER_S / 250; // ~4ms pause per top-up (>= MIN_CMD_TICKS)
static const uint32_t KEEPALIVE_MIN_TICKS = TICKS_PER_S / 100; // top up when < ~10ms remain in the HW queue

// Emit the accumulated batch as ONE timed window across all 6 joints, then
// clear the accumulator. Blocks (with backpressure) until every joint's window
// is appended to its hardware queue. Each joint appends exactly one window
// (its steps, or a pure pause if it didn't move) sharing the same duration, so
// the axes stay time-synchronized.
static void flushBatchWindow()
{
  uint32_t winTicks = accTicks;
  if (winTicks == 0) return;
  bool flushed[6] = {false, false, false, false, false, false};
  uint32_t spins = 0;                       // guard against an endless busy loop
  while (true)
  {
    bool busy = false;
    bool fault = false;
    for (int i = 0; i < 6; i++)
    {
      if (flushed[i]) continue;
      uint32_t dur = winTicks + feederDrift[i];
      uint32_t actual = 0;
      MoveTimedResultCode r = steppers[i]->moveTimed(accSteps[i], dur, &actual, true);
      if (r == MoveTimedResultCode::OK || r == MoveTimedResultCode::MoveEmpty) {
        feederDrift[i] = dur - actual; // carry leftover ticks forward
        flushed[i] = true;
      } else if (static_cast<int8_t>(r) > 0) {
        busy = true;                   // HW queue full -> retry just this joint
      } else {
        fault = true;
        Serial.printf("FAULT! moveTimed error on Joint %d. Error Code: %d\n", i, static_cast<int>(r));
      }
    }
    if (fault) {
      Serial.println("Executing Emergency Stop across all axes...");
      for (int j = 0; j < 6; j++) steppers[j]->stopMove();
      xQueueReset(stepperQueue);
      for (int j = 0; j < 6; j++) { feederDrift[j] = 0; accSteps[j] = 0; }
      accTicks = 0;
      return;
    }
    if (!busy) break;                  // all 6 windows appended
    // HW queues full: wait ~1ms for the silicon timer to drain a slot, then
    // retry the not-yet-appended joints. This is the real-time backpressure.
    // A genuine HW-queue-full state clears within a few ms as the timer drains.
    // If a joint stays "busy" for ~250ms it is NOT backpressure -- it is a
    // stepper that is ignoring moveTimed (e.g. still owned by the ramp
    // generator after a forceStop). Report it (with the joint + code) and bail
    // instead of freezing the whole ESP forever.
    if (++spins > 250) {
      for (int i = 0; i < 6; i++) {
        if (flushed[i]) continue;
        uint32_t actual = 0;
        MoveTimedResultCode r = steppers[i]->moveTimed(accSteps[i], winTicks, &actual, true);
        Serial.printf("STUCK: Joint %d moveTimed not accepted (code %d). "
                      "Likely ignoring moveTimed after ramp/forceStop.\n",
                      i, static_cast<int>(r));
      }
      for (int j = 0; j < 6; j++) { accSteps[j] = 0; }
      accTicks = 0;
      return;
    }
    vTaskDelay(1);
  }
  for (int j = 0; j < 6; j++) accSteps[j] = 0;
  accTicks = 0;
}

void feeder(void *pvParameters)
{
  JointFrame j_recv = {{0, 0, 0, 0, 0, 0}};
  static uint32_t underrun_count = 0;
  static uint32_t last_empty_print_ms = 0;
  while (true)
  {
    bool got = (xQueueReceive(stepperQueue, &j_recv, pdMS_TO_TICKS(1)) == pdPASS);
    if (got)
    {
      // Absorb this frame into the batch accumulator (RAM only, never fails).
      for (int i = 0; i < 6; i++)
      {
        int32_t delta = j_recv.q_steps[i] - emitted.q_steps[i];
        accSteps[i] += delta;
        emitted.q_steps[i] = j_recv.q_steps[i];
      }
      // Each frame represents TRAJ_DT_S of motion (must match the producer's
      // Ruckig/IK timestep). Advance the HW-queue window time accordingly.
      accTicks += (uint32_t)(TICKS_PER_S * TRAJ_DT_S);
    }
    else if (g_motionActive)
    {
      // Mid-motion gap: no frame arrived within 1ms while a motion is active
      // (producer momentarily behind, or stalled on a Serial print). The
      // unconditional keep-alive below bridges it; just record for visibility.
      underrun_count++;
      if (millis() - last_empty_print_ms >= 1000) {
        last_empty_print_ms = millis();
        if (underrun_count > 0) {
          Serial.printf("UNDERRUN: queue starved %u times in last 1s\n", underrun_count);
          underrun_count = 0;
        }
      }
    }

    // ---- Maintain HW-queue lookahead EVERY iteration (drain protection) ----
    // Is any stepper's HW queue about to run dry?
    bool hwLow = false;
    for (int i = 0; i < 6; i++) {
      if (!steppers[i]->hasTicksInQueue(KEEPALIVE_MIN_TICKS)) { hwLow = true; break; }
    }
    // Busiest joint's accumulated step count -> decides a step-rich flush.
    int32_t mx = 0;
    for (int i = 0; i < 6; i++) {
      int32_t a = accSteps[i] < 0 ? -accSteps[i] : accSteps[i];
      if (a > mx) mx = a;
    }
    // Flush the accumulated window when it is step-rich (smooth), a latency cap
    // is hit, the motion just finished, OR a HW queue is starving. The hwLow
    // term is the critical fix: during active streaming the feeder used to stay
    // in the receive branch and only flush on step/latency thresholds, so the
    // HW queue could drain to empty between flushes and latch the timer into
    // DeviceNotReady. Now any impending starvation forces a flush.
    bool finishing = (!g_motionActive && accTicks > 0);
    if (accTicks > 0 && (mx >= BATCH_MIN_STEPS || accTicks >= BATCH_MAX_TICKS || hwLow || finishing)) {
      flushBatchWindow();
    }

    // After any flush, pad any STILL-idle joint whose queue is low with a short
    // 0-step pause so its MCPWM timer never empties and latches. Runs every
    // iteration (both branches) -- confining this to the FreeRTOS-empty branch
    // was the bug that let the queue drain and latch mid-motion.
    for (int i = 0; i < 6; i++) {
      if (!steppers[i]->hasTicksInQueue(KEEPALIVE_MIN_TICKS)) {
        uint32_t dur = KEEPALIVE_TICKS + feederDrift[i];
        uint32_t actual = 0;
        MoveTimedResultCode r = steppers[i]->moveTimed(0, dur, &actual, true);
        if (r == MoveTimedResultCode::OK || r == MoveTimedResultCode::MoveEmpty) {
          feederDrift[i] = dur - actual; // carry leftover ticks forward
        }
      }
    }
  }
}