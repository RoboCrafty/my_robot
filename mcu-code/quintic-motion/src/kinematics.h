#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

struct Joints {
    float q1, q2, q3, q4, q5, q6;
};

struct Pose {
    float x, y, z, rx, ry, rz;
};

inline Pose extract_pose_from_transformation_matrix(const BLA::Matrix<4,4, float>& T0_ee);
/**
 * Calculates the forward kinematics transformation matrix.
 * @param q Array of 6 joint angles (q1 to q6 in radians) passed via the Joints struct
 * @return 4x4 Homogeneous Transformation Matrix
 */
inline void IRAM_ATTR calculateForwardKinematics(const Joints& q, Pose& result) {
    // 1. Unpack q array for readability (using 0-based indexing)
    float q1 = q.q1, q2 = q.q2, q3 = q.q3, q4 = q.q4, q5 = q.q5, q6 = q.q6;


    BLA::Matrix<4,4, float> T0_ee_result;
    T0_ee_result.Fill(0.0f);
    T0_ee_result(0, 0) = cos(q6)*(cos(q1)*cos(q4)+sin(q4)*(cos(q2)*sin(q1)*sin(q3)+cos(q3)*sin(q1)*sin(q2)))-sin(q6)*(cos(q5)*(cos(q1)*sin(q4)-cos(q4)*(cos(q2)*sin(q1)*sin(q3)+cos(q3)*sin(q1)*sin(q2)))+sin(q5)*(sin(q1)*sin(q2)*sin(q3)-cos(q2)*cos(q3)*sin(q1)));
    T0_ee_result(0, 1) = -sin(q6)*(cos(q1)*cos(q4)+sin(q4)*(cos(q2)*sin(q1)*sin(q3)+cos(q3)*sin(q1)*sin(q2)))-cos(q6)*(cos(q5)*(cos(q1)*sin(q4)-cos(q4)*(cos(q2)*sin(q1)*sin(q3)+cos(q3)*sin(q1)*sin(q2)))+sin(q5)*(sin(q1)*sin(q2)*sin(q3)-cos(q2)*cos(q3)*sin(q1)));
    T0_ee_result(0, 2) = sin(q5)*(cos(q1)*sin(q4)-cos(q4)*(cos(q2)*sin(q1)*sin(q3)+cos(q3)*sin(q1)*sin(q2)))-cos(q5)*(sin(q1)*sin(q2)*sin(q3)-cos(q2)*cos(q3)*sin(q1));
    T0_ee_result(0, 3) = sin(q1)*2.342E-2+sin(q1)*sin(q2)*(9.0/5.0E+1)-sin(q1)*sin(q2)*sin(q3)*1.7635E-1+(cos(q2+q3)*cos(q5)*sin(q1))/1.0E+1+cos(q2)*cos(q3)*sin(q1)*1.7635E-1+cos(q2)*sin(q1)*sin(q3)*4.35E-2+cos(q3)*sin(q1)*sin(q2)*4.35E-2+(cos(q1)*sin(q4)*sin(q5))/1.0E+1-(cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5))/1.0E+1-(cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5))/1.0E+1;
    T0_ee_result(1, 0) = -sin(q6)*(sin(q2+q3)*sin(q5)-cos(q2+q3)*cos(q4)*cos(q5))+cos(q2+q3)*cos(q6)*sin(q4);
    T0_ee_result(1, 1) = -cos(q6)*(sin(q2+q3)*sin(q5)-cos(q2+q3)*cos(q4)*cos(q5))-cos(q2+q3)*sin(q4)*sin(q6);
    T0_ee_result(1, 2) = -sin(q2+q3)*cos(q5)-cos(q2+q3)*cos(q4)*sin(q5);
    T0_ee_result(1, 3) = cos(q2+q3)*4.35E-2-sin(q2+q3)*1.7635E-1+cos(q2)*(9.0/5.0E+1)+(sin(q4-q5)*cos(q2+q3))/2.0E+1-(cos(q2+q3)*sin(q4+q5))/2.0E+1-(sin(q2+q3)*cos(q5))/1.0E+1+1.105E-1;
    T0_ee_result(2, 0) = -cos(q6)*(cos(q4)*sin(q1)-sin(q4)*(cos(q1)*cos(q2)*sin(q3)+cos(q1)*cos(q3)*sin(q2)))+sin(q6)*(cos(q5)*(sin(q1)*sin(q4)+cos(q4)*(cos(q1)*cos(q2)*sin(q3)+cos(q1)*cos(q3)*sin(q2)))+sin(q5)*(cos(q1)*cos(q2)*cos(q3)-cos(q1)*sin(q2)*sin(q3)));
    T0_ee_result(2, 1) = sin(q6)*(cos(q4)*sin(q1)-sin(q4)*(cos(q1)*cos(q2)*sin(q3)+cos(q1)*cos(q3)*sin(q2)))+cos(q6)*(cos(q5)*(sin(q1)*sin(q4)+cos(q4)*(cos(q1)*cos(q2)*sin(q3)+cos(q1)*cos(q3)*sin(q2)))+sin(q5)*(cos(q1)*cos(q2)*cos(q3)-cos(q1)*sin(q2)*sin(q3)));
    T0_ee_result(2, 2) = -sin(q5)*(sin(q1)*sin(q4)+cos(q4)*(cos(q1)*cos(q2)*sin(q3)+cos(q1)*cos(q3)*sin(q2)))+cos(q5)*(cos(q1)*cos(q2)*cos(q3)-cos(q1)*sin(q2)*sin(q3));
    T0_ee_result(2, 3) = cos(q1)*2.342E-2+cos(q1)*sin(q2)*(9.0/5.0E+1)-(sin(q1)*sin(q4)*sin(q5))/1.0E+1+(cos(q2+q3)*cos(q1)*cos(q5))/1.0E+1+cos(q1)*cos(q2)*cos(q3)*1.7635E-1+cos(q1)*cos(q2)*sin(q3)*4.35E-2+cos(q1)*cos(q3)*sin(q2)*4.35E-2-cos(q1)*sin(q2)*sin(q3)*1.7635E-1-(cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5))/1.0E+1-(cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5))/1.0E+1;
    T0_ee_result(3, 3) = 1.0;

    result = extract_pose_from_transformation_matrix(T0_ee_result);
}


/**
 * Calculates the Jacobian matrix for the robotic arm.
 * @param q Array of 6 joint angles (q1 to q6 in radians) passed via the Joints struct
 * @return Doesn't return anything, but modifies the Jacobian_result matrix
 */
inline void IRAM_ATTR fillJacobian(const Joints& joints, BLA::Matrix<6,6, float>& Jacobian_result)
{
    float q1 = joints.q1, q2 = joints.q2, q3 = joints.q3, q4 = joints.q4, q5 = joints.q5, q6 = joints.q6;

    Jacobian_result.Fill(0.0f);
    Jacobian_result(0, 0) = cosf(q2+q3)*(-4.35E-2f)+sinf(q2+q3)*1.7635E-1f-cosf(q2)*(9.0f/5.0E+1f)-(sinf(q4-q5)*cosf(q2+q3))/2.0E+1f+(cosf(q2+q3)*sinf(q4+q5))/2.0E+1f+(sinf(q2+q3)*cosf(q5))/1.0E+1f-1.105E-1f;
    Jacobian_result(0, 1) = cosf(q1)*2.342E-2f+cosf(q1)*sinf(q2)*(9.0f/5.0E+1f)-(sinf(q1)*sinf(q4)*sinf(q5))/1.0E+1f+(cosf(q2+q3)*cosf(q1)*cosf(q5))/1.0E+1f+cosf(q1)*cosf(q2)*cosf(q3)*1.7635E-1f+cosf(q1)*cosf(q2)*sinf(q3)*4.35E-2f+cosf(q1)*cosf(q3)*sinf(q2)*4.35E-2f-cosf(q1)*sinf(q2)*sinf(q3)*1.7635E-1f-(cosf(q1)*cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5))/1.0E+1f-(cosf(q1)*cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5))/1.0E+1f;
    Jacobian_result(0, 2) = sinf(q1)*(cosf(q2+q3)*4.35E-2f-sinf(q2+q3)*1.7635E-1f+cosf(q2)*(9.0f/5.0E+1f)+(sinf(q4-q5)*cosf(q2+q3))/2.0E+1f-(cosf(q2+q3)*sinf(q4+q5))/2.0E+1f-(sinf(q2+q3)*cosf(q5))/1.0E+1f);
    Jacobian_result(0, 3) = sinf(q1)*(cosf(q2+q3)*-8.7E+2f+sinf(q2+q3)*3.527E+3f+sinf(q2+q3)*cosf(q5)*2.0E+3f+cosf(q2+q3)*cosf(q4)*sinf(q5)*2.0E+3f)*(-5.0E-5f);
    Jacobian_result(0, 4) = (sinf(q5)*(cosf(q1)*cosf(q4)+cosf(q2)*sinf(q1)*sinf(q3)*sinf(q4)+cosf(q3)*sinf(q1)*sinf(q2)*sinf(q4)))/1.0E+1f;
    Jacobian_result(0, 5) = (cosf(q1)*cosf(q5)*sinf(q4))/1.0E+1f-(cosf(q2)*cosf(q3)*sinf(q1)*sinf(q5))/1.0E+1f+(sinf(q1)*sinf(q2)*sinf(q3)*sinf(q5))/1.0E+1f-(cosf(q2)*cosf(q4)*cosf(q5)*sinf(q1)*sinf(q3))/1.0E+1f-(cosf(q3)*cosf(q4)*cosf(q5)*sinf(q1)*sinf(q2))/1.0E+1f;
    Jacobian_result(1, 0) = sinf(q1)*2.342E-2f+sinf(q1)*sinf(q2)*(9.0f/5.0E+1f)-sinf(q1)*sinf(q2)*sinf(q3)*1.7635E-1f+(cosf(q2+q3)*cosf(q5)*sinf(q1))/1.0E+1f+cosf(q2)*cosf(q3)*sinf(q1)*1.7635E-1f+cosf(q2)*sinf(q1)*sinf(q3)*4.35E-2f+cosf(q3)*sinf(q1)*sinf(q2)*4.35E-2f+(cosf(q1)*sinf(q4)*sinf(q5))/1.0E+1f-(cosf(q2)*cosf(q4)*sinf(q1)*sinf(q3)*sinf(q5))/1.0E+1f-(cosf(q3)*cosf(q4)*sinf(q1)*sinf(q2)*sinf(q5))/1.0E+1f;
    Jacobian_result(1, 2) = sinf(q2)*(-9.0f/5.0E+1f)-cosf(q2)*cosf(q3)*1.7635E-1f-cosf(q2)*sinf(q3)*4.35E-2f-cosf(q3)*sinf(q2)*4.35E-2f+sinf(q2)*sinf(q3)*1.7635E-1f-(cosf(q2)*cosf(q3)*cosf(q5))/1.0E+1f+(cosf(q5)*sinf(q2)*sinf(q3))/1.0E+1f+(cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5))/1.0E+1f+(cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5))/1.0E+1f;
    Jacobian_result(1, 3) = cosf(q2)*cosf(q3)*(-1.7635E-1f)-cosf(q2)*sinf(q3)*4.35E-2f-cosf(q3)*sinf(q2)*4.35E-2f+sinf(q2)*sinf(q3)*1.7635E-1f-(cosf(q2)*cosf(q3)*cosf(q5))/1.0E+1f+(cosf(q5)*sinf(q2)*sinf(q3))/1.0E+1f+(cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5))/1.0E+1f+(cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5))/1.0E+1f;
    Jacobian_result(1, 4) = (cosf(q2+q3)*sinf(q4)*sinf(q5))/1.0E+1f;
    Jacobian_result(1, 5) = (cosf(q2)*sinf(q3)*sinf(q5))/1.0E+1f+(cosf(q3)*sinf(q2)*sinf(q5))/1.0E+1f-(cosf(q2)*cosf(q3)*cosf(q4)*cosf(q5))/1.0E+1f+(cosf(q4)*cosf(q5)*sinf(q2)*sinf(q3))/1.0E+1f;
    Jacobian_result(2, 1) = sinf(q1)*(-2.342E-2f)-sinf(q1)*sinf(q2)*(9.0f/5.0E+1f)+sinf(q1)*sinf(q2)*sinf(q3)*1.7635E-1f-(cosf(q2+q3)*cosf(q5)*sinf(q1))/1.0E+1f-cosf(q2)*cosf(q3)*sinf(q1)*1.7635E-1f-cosf(q2)*sinf(q1)*sinf(q3)*4.35E-2f-cosf(q3)*sinf(q1)*sinf(q2)*4.35E-2f-(cosf(q1)*sinf(q4)*sinf(q5))/1.0E+1f+(cosf(q2)*cosf(q4)*sinf(q1)*sinf(q3)*sinf(q5))/1.0E+1f+(cosf(q3)*cosf(q4)*sinf(q1)*sinf(q2)*sinf(q5))/1.0E+1f;
    Jacobian_result(2, 2) = cosf(q1)*(cosf(q2+q3)*4.35E-2f-sinf(q2+q3)*1.7635E-1f+cosf(q2)*(9.0f/5.0E+1f)+(sinf(q4-q5)*cosf(q2+q3))/2.0E+1f-(cosf(q2+q3)*sinf(q4+q5))/2.0E+1f-(sinf(q2+q3)*cosf(q5))/1.0E+1f);
    Jacobian_result(2, 3) = cosf(q1)*(cosf(q2+q3)*-8.7E+2f+sinf(q2+q3)*3.527E+3f+sinf(q2+q3)*cosf(q5)*2.0E+3f+cosf(q2+q3)*cosf(q4)*sinf(q5)*2.0E+3f)*(-5.0E-5f);
    Jacobian_result(2, 4) = (sinf(q5)*(-cosf(q4)*sinf(q1)+cosf(q1)*cosf(q2)*sinf(q3)*sinf(q4)+cosf(q1)*cosf(q3)*sinf(q2)*sinf(q4)))/1.0E+1f;
    Jacobian_result(2, 5) = cosf(q5)*sinf(q1)*sinf(q4)*(-1.0f/1.0E+1f)-(cosf(q1)*cosf(q2)*cosf(q3)*sinf(q5))/1.0E+1f+(cosf(q1)*sinf(q2)*sinf(q3)*sinf(q5))/1.0E+1f-(cosf(q1)*cosf(q2)*cosf(q4)*cosf(q5)*sinf(q3))/1.0E+1f-(cosf(q1)*cosf(q3)*cosf(q4)*cosf(q5)*sinf(q2))/1.0E+1f;
    Jacobian_result(3, 2) = cosf(q1);
    Jacobian_result(3, 3) = cosf(q1);
    Jacobian_result(3, 4) = cosf(q2+q3)*sinf(q1);
    Jacobian_result(3, 5) = cosf(q1)*cosf(q4)+sinf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2));
    Jacobian_result(4, 1) = 1.0f;
    Jacobian_result(4, 4) = -sinf(q2+q3);
    Jacobian_result(4, 5) = cosf(q2+q3)*sinf(q4);
    Jacobian_result(5, 0) = 1.0f;
    Jacobian_result(5, 2) = -sinf(q1);
    Jacobian_result(5, 3) = -sinf(q1);
    Jacobian_result(5, 4) = cosf(q2+q3)*cosf(q1);
    Jacobian_result(5, 5) = -cosf(q4)*sinf(q1)+sinf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2));
}


inline void degToRad(Joints& j) 
{
    j.q1 = j.q1 * (M_PI / 180.0f);
    j.q2 = j.q2 * (M_PI / 180.0f);
    j.q3 = j.q3 * (M_PI / 180.0f);
    j.q4 = j.q4 * (M_PI / 180.0f);
    j.q5 = j.q5 * (M_PI / 180.0f);
    j.q6 = j.q6 * (M_PI / 180.0f);
}
inline float radToDeg(float rad_angle) 
{
    float result = rad_angle * (180.0f / M_PI);
    return result;
}

inline Joints getIK(const Joints current_joint_config, const Pose* target_pose, int& itr_counter)
{
    
    Pose current_pose;
    Joints result;
    calculateForwardKinematics(current_joint_config, current_pose);

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


inline void IRAM_ATTR printPose(const Pose& p)
{
    Serial.print("x: ");    Serial.print(p.x);
    Serial.print(" | y: "); Serial.print(p.y);
    Serial.print(" | z: "); Serial.print(p.z);

    Serial.print(" | rx: "); Serial.print(radToDeg(p.rx));
    Serial.print(" | ry: "); Serial.print(radToDeg(p.ry));
    Serial.print(" | rz: "); Serial.println(radToDeg(p.rz));
}