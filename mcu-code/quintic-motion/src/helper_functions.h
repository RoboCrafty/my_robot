#pragma once
#include <BasicLinearAlgebra.h>
#include <structs.h>


inline void degToRad(Joints& j) 
{
    j.q1 = j.q1 * (M_PI / 180.0f);
    j.q2 = j.q2 * (M_PI / 180.0f);
    j.q3 = j.q3 * (M_PI / 180.0f);
    j.q4 = j.q4 * (M_PI / 180.0f);
    j.q5 = j.q5 * (M_PI / 180.0f);
    j.q6 = j.q6 * (M_PI / 180.0f);
}
inline void degToRad(BLA::Matrix<6, 1, float>& j) 
{
    j(0, 0) = j(0, 0) * (M_PI / 180.0f);
    j(1, 0) = j(1, 0) * (M_PI / 180.0f);
    j(2, 0) = j(2, 0) * (M_PI / 180.0f);
    j(3, 0) = j(3, 0) * (M_PI / 180.0f);
    j(4, 0) = j(4, 0) * (M_PI / 180.0f);
    j(5, 0) = j(5, 0) * (M_PI / 180.0f);
}
inline float degToRad(float deg) 
{
    float result = deg * (M_PI / 180.0f);
    return result;
}
inline void radToDeg(BLA::Matrix<6, 1, float>& j) 
{
    j(0, 0) = j(0, 0) * (180.0f / M_PI);
    j(1, 0) = j(1, 0) * (180.0f / M_PI);
    j(2, 0) = j(2, 0) * (180.0f / M_PI);
    j(3, 0) = j(3, 0) * (180.0f / M_PI);
    j(4, 0) = j(4, 0) * (180.0f / M_PI);
    j(5, 0) = j(5, 0) * (180.0f / M_PI);
}
inline float radToDeg(float rad_angle) 
{
    float result = rad_angle * (180.0f / M_PI);
    return result;
}

inline Pose IRAM_ATTR extract_pose_from_transformation_matrix(const BLA::Matrix<4,4, float>& T0_ee)
{
    Pose result;
    result.x   = T0_ee(0,3);
    result.y   = T0_ee(1,3);
    result.z   = T0_ee(2,3);
    float r11 = T0_ee(0,0), r12 = T0_ee(0,1), r13 = T0_ee(0,2);
    float r21 = T0_ee(1,0), r22 = T0_ee(1,1), r23 = T0_ee(1,2);
    float r31 = T0_ee(2,0), r32 = T0_ee(2,1), r33 = T0_ee(2,2);

    // float pitch = asin(-r31);
    float roll = atan2(r32, r33);
    float pitch = atan2(-r31, sqrt(r32*r32 + r33*r33)); // More numerically stable than asin(-r31)
    float yaw  = atan2(r21, r11);

    result.rx = roll;
    result.ry = pitch;
    result.rz = yaw;
    // TODO: Figure out gimbal lock stuff. Maybe quaternions??
    // else
    // {
    //     // gimbal lock case
    //     float yaw = atan2(-r12, r22);

    //     result.rx = 0.0f;
    //     result.ry = pitch;
    //     result.rz = yaw;
    // }

    return result;
}   

inline BLA::Matrix<3, 3, float> getRotationMatrix(float rx, float ry, float rz)
{  
    // Assemble the target Rotation Matrix (Yaw * Pitch * Roll) 
    BLA::Matrix<3, 3, float> R = {
        cos(rz)*cos(ry),  (cos(rz)*sin(ry)*sin(rx)) - (sin(rz)*cos(rx)),  (cos(rz)*sin(ry)*cos(rx)) + (sin(rz)*sin(rx)),
        sin(rz)*cos(ry),  (sin(rz)*sin(ry)*sin(rx)) + (cos(rz)*cos(rx)),  (sin(rz)*sin(ry)*cos(rx)) - (cos(rz)*sin(rx)),
        -sin(ry),         cos(ry)*sin(rx),                                cos(ry)*cos(rx)
    };

    return R;
}


inline void IRAM_ATTR printPose(const Pose& p)
{
    Serial.print("x: ");    Serial.print(p.x, 5 );
    Serial.print(" | y: "); Serial.print(p.y, 5);
    Serial.print(" | z: "); Serial.print(p.z, 5);

    Serial.print(" | rx: "); Serial.print(radToDeg(p.rx), 5);
    Serial.print(" | ry: "); Serial.print(radToDeg(p.ry), 5);
    Serial.print(" | rz: "); Serial.println(radToDeg(p.rz), 5);
}

inline BLA::Matrix<3, 1, float> getAxisAngleError(BLA::Matrix<3, 3, float> R)
{
    BLA::Matrix<3, 1, float> rot_error;

    // 1. Calculate the trace value safely
    float trace_val = (BLA::Trace(R) - 1.0f) / 2.0f;

    // 2. CRITICAL FIX 1: Clamp the value to prevent acosf() from throwing NaN
    if (trace_val > 1.0f) trace_val = 1.0f;
    if (trace_val < -1.0f) trace_val = -1.0f;

    float theta = acosf(trace_val);

    // 3. Handle the near-zero rotation singularity
    if (theta < 1e-5f) {
        rot_error(0) = 0.0f;
        rot_error(1) = 0.0f;
        rot_error(2) = 0.0f;
        return rot_error;
    }

    // 4. CRITICAL FIX 2: Prevent divide-by-zero if angle is exactly 180 degrees (PI)
    float s = sinf(theta);
    if (s < 1e-5f) {
        s = 1e-5f; // Provide a tiny floor to prevent infinity
    }

    float k = theta / (2.0f * s);

    rot_error(0) = R(2,1) - R(1,2);
    rot_error(1) = R(0,2) - R(2,0);
    rot_error(2) = R(1,0) - R(0,1);

    rot_error = k * rot_error;
    return rot_error;
}