// Print an Eigen matrix to the Serial console with a given name
void printMatrix(const Eigen::MatrixXd& mat, const char* name) {
    Serial.printf("--- Matrix: %s (%dx%d) ---\n", name, (int)mat.rows(), (int)mat.cols());
    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            // Using %.2f format assumes your matrix holds floats/doubles
            Serial.printf("%.2f\t", mat(i, j)); 
        }
        Serial.println();
    }
    Serial.println();
}




// Quintic polynomial evaluation functions
// --- REQUIRES THESE DEFINES ---
// #define TICKS_PER_S 16000000 
// #define TIME_SLICE_S 0.02

// 1. The Math: Evaluates normalized quintic position (0.0 to 1.0)
float getQuinticPosition(float t, float T) {
  if (t <= 0.0) return 0.0;
  if (t >= T) return 1.0;
  
  float tau = t / T;
  return 10.0 * pow(tau, 3) - 15.0 * pow(tau, 4) + 6.0 * pow(tau, 5);
}

// 2. The Generator: Slices time and fills the command queue
void executeQuinticMove(int32_t steps_to_move, float duration_sec) {
  if (steps_to_move == 0) return;

  bool dir_high = (steps_to_move > 0);
  uint32_t total_steps = abs(steps_to_move);
  
  int num_slices = duration_sec / TIME_SLICE_S;
  uint32_t steps_completed = 0;

  for (int i = 1; i <= num_slices; i++) {
    float t = i * TIME_SLICE_S;
    if (i == num_slices) t = duration_sec; // Force exact end time

    // Calculate target position at the end of this window
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
      while ((int)res > 0) { 
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

// 3. Execution Loop Test
void test_sync_move_loop() {
  if (stepper && stepper2) {
    Serial.println("Starting Sync Move...");

    // Start Trapezoidal motor FIRST (Non-blocking)
    stepper2->moveTo(10000);
    // Queue Quintic move (Blocking-generation)
    executeQuinticMove(10000, 0.7); 

    while (!stepper->isQueueEmpty() || stepper2->isRunning()) { yield(); }
    
    // Reverse
    stepper2->moveTo(0);
    executeQuinticMove(-10000, 1.2);
    
    while (!stepper->isQueueEmpty() || stepper2->isRunning()) { yield(); }
  }
}