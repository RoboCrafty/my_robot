#pragma once

// Trajectory control timestep (seconds). ONE source of truth used by THREE
// coupled sites that MUST agree or the joint integration desyncs from real
// time: (1) Ruckig control cycle -> Ruckig ruck(TRAJ_DT_S) in motion_planner.h;
// (2) RRMC integration step -> dt in getIK_RRMC (kinematics.h); (3) the feeder's
// per-frame HW-queue duration -> accTicks += TICKS_PER_S*TRAJ_DT_S (main.cpp).
// 4ms = 250 Hz: matches the official moveTimed example and gives the producer
// (~1ms per RRMC+IK frame) a ~4x real-time margin so the HW queue never starves.
#define TRAJ_DT_S 0.001f

struct Joints {
    float q1, q2, q3, q4, q5, q6;
};
struct JointFrame {
    int32_t q_steps[6];   
};
struct Pose {
    float x, y, z, rx, ry, rz;
};

struct TrigValues {
    float s1, s2, s3, s4, s5, s6;
    float c1, c2, c3, c4, c5, c6;
};


// Toggle this: 1 to enable debug prints, 0 to disable completely
#define DEBUG_MODE 0

#if DEBUG_MODE
  #define DEBUG_PRINT(x)         Serial.print(x)
  #define DEBUG_PRINTLN(x)       Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  // If debug is off, these macros are replaced with absolutely nothing
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif