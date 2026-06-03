#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <helper_functions.h>
#include <structs.h>


/**
 * Calculates the forward kinematics transformation matrix.
 * @param q Array of 6 joint angles (q1 to q6 in radians) passed via the Joints struct
 * @return 4x4 Homogeneous Transformation Matrix
 */
inline void IRAM_ATTR getFK(BLA::Matrix<6, 1, float>& q, Pose& result, BLA::Matrix<4,4, float>& T0_ee_result) {
    // 1. Unpack q array for readability (using 0-based indexing)
    float q1 = q(0, 0), q2 = q(1, 0), q3 = q(2, 0), q4 = q(3, 0), q5 = q(4, 0), q6 = q(5, 0);

    T0_ee_result.Fill(0.0f);
    T0_ee_result(0, 0) = cosf(q6)*(cosf(q1)*cosf(q4)+sinf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2)))-sinf(q6)*(cosf(q5)*(cosf(q1)*sinf(q4)-cosf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2)))+sinf(q5)*(sinf(q1)*sinf(q2)*sinf(q3)-cosf(q2)*cosf(q3)*sinf(q1)));
    T0_ee_result(0, 1) = -sinf(q6)*(cosf(q1)*cosf(q4)+sinf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2)))-cosf(q6)*(cosf(q5)*(cosf(q1)*sinf(q4)-cosf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2)))+sinf(q5)*(sinf(q1)*sinf(q2)*sinf(q3)-cosf(q2)*cosf(q3)*sinf(q1)));
    T0_ee_result(0, 2) = sinf(q5)*(cosf(q1)*sinf(q4)-cosf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2)))-cosf(q5)*(sinf(q1)*sinf(q2)*sinf(q3)-cosf(q2)*cosf(q3)*sinf(q1));
    T0_ee_result(0, 3) = sinf(q1)*2.342E-2+sinf(q1)*sinf(q2)*(9.0/5.0E+1)-sinf(q1)*sinf(q2)*sinf(q3)*1.7635E-1+cosf(q2+q3)*cosf(q5)*sinf(q1)*(8.9E+1/1.0E+3)+cosf(q2)*cosf(q3)*sinf(q1)*1.7635E-1+cosf(q2)*sinf(q1)*sinf(q3)*4.35E-2+cosf(q3)*sinf(q1)*sinf(q2)*4.35E-2+cosf(q1)*sinf(q4)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q2)*cosf(q4)*sinf(q1)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q3)*cosf(q4)*sinf(q1)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3);
    T0_ee_result(1, 0) = -sinf(q6)*(sinf(q2+q3)*sinf(q5)-cosf(q2+q3)*cosf(q4)*cosf(q5))+cosf(q2+q3)*cosf(q6)*sinf(q4);
    T0_ee_result(1, 1) = -cosf(q6)*(sinf(q2+q3)*sinf(q5)-cosf(q2+q3)*cosf(q4)*cosf(q5))-cosf(q2+q3)*sinf(q4)*sinf(q6);
    T0_ee_result(1, 2) = -sinf(q2+q3)*cosf(q5)-cosf(q2+q3)*cosf(q4)*sinf(q5);
    T0_ee_result(1, 3) = cosf(q2+q3)*4.35E-2-sinf(q2+q3)*1.7635E-1+cosf(q2)*(9.0/5.0E+1)+sinf(q4-q5)*cosf(q2+q3)*4.45E-2-cosf(q2+q3)*sinf(q4+q5)*4.45E-2-sinf(q2+q3)*cosf(q5)*(8.9E+1/1.0E+3)+1.105E-1;
    T0_ee_result(2, 0) = -cosf(q6)*(cosf(q4)*sinf(q1)-sinf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2)))+sinf(q6)*(cosf(q5)*(sinf(q1)*sinf(q4)+cosf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2)))+sinf(q5)*(cosf(q1)*cosf(q2)*cosf(q3)-cosf(q1)*sinf(q2)*sinf(q3)));
    T0_ee_result(2, 1) = sinf(q6)*(cosf(q4)*sinf(q1)-sinf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2)))+cosf(q6)*(cosf(q5)*(sinf(q1)*sinf(q4)+cosf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2)))+sinf(q5)*(cosf(q1)*cosf(q2)*cosf(q3)-cosf(q1)*sinf(q2)*sinf(q3)));
    T0_ee_result(2, 2) = -sinf(q5)*(sinf(q1)*sinf(q4)+cosf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2)))+cosf(q5)*(cosf(q1)*cosf(q2)*cosf(q3)-cosf(q1)*sinf(q2)*sinf(q3));
    T0_ee_result(2, 3) = cosf(q1)*2.342E-2+cosf(q1)*sinf(q2)*(9.0/5.0E+1)-sinf(q1)*sinf(q4)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q2+q3)*cosf(q1)*cosf(q5)*(8.9E+1/1.0E+3)+cosf(q1)*cosf(q2)*cosf(q3)*1.7635E-1+cosf(q1)*cosf(q2)*sinf(q3)*4.35E-2+cosf(q1)*cosf(q3)*sinf(q2)*4.35E-2-cosf(q1)*sinf(q2)*sinf(q3)*1.7635E-1-cosf(q1)*cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q1)*cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3);
    T0_ee_result(3, 3) = 1.0;
    result = extract_pose_from_transformation_matrix(T0_ee_result);
}


/**
 * Calculates the Jacobian matrix for the robotic arm.
 * @param q Array of 6 joint angles (q1 to q6 in radians) passed via the Joints struct
 * @return Doesn't return anything, but modifies the Jacobian_result matrix
 */
inline void IRAM_ATTR fillJacobian(BLA::Matrix<6, 1, float>& joints, BLA::Matrix<6,6, float>& Jacobian_result)
{
    float q1 = joints(0, 0), q2 = joints(1, 0), q3 = joints(2, 0), q4 = joints(3, 0), q5 = joints(4, 0), q6 = joints(5, 0);

    Jacobian_result.Fill(0.0f);
    Jacobian_result(0, 0) = cosf(q1)*2.342E-2+cosf(q1)*sinf(q2)*(9.0/5.0E+1)-sinf(q1)*sinf(q4)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q2+q3)*cosf(q1)*cosf(q5)*(8.9E+1/1.0E+3)+cosf(q1)*cosf(q2)*cosf(q3)*1.7635E-1+cosf(q1)*cosf(q2)*sinf(q3)*4.35E-2+cosf(q1)*cosf(q3)*sinf(q2)*4.35E-2-cosf(q1)*sinf(q2)*sinf(q3)*1.7635E-1-cosf(q1)*cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q1)*cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3);
    Jacobian_result(0, 1) = sinf(q1)*(cosf(q2+q3)*4.35E-2-sinf(q2+q3)*1.7635E-1+cosf(q2)*(9.0/5.0E+1)+sinf(q4-q5)*cosf(q2+q3)*4.45E-2-cosf(q2+q3)*sinf(q4+q5)*4.45E-2-sinf(q2+q3)*cosf(q5)*(8.9E+1/1.0E+3));
    Jacobian_result(0, 2) = sinf(q1)*(cosf(q2+q3)*-8.7E+2+sinf(q2+q3)*3.527E+3+sinf(q2+q3)*cosf(q5)*1.78E+3+cosf(q2+q3)*cosf(q4)*sinf(q5)*1.78E+3)*(-5.0E-5);
    Jacobian_result(0, 3) = sinf(q5)*(cosf(q1)*cosf(q4)+cosf(q2)*sinf(q1)*sinf(q3)*sinf(q4)+cosf(q3)*sinf(q1)*sinf(q2)*sinf(q4))*(8.9E+1/1.0E+3);
    Jacobian_result(0, 4) = cosf(q1)*cosf(q5)*sinf(q4)*(8.9E+1/1.0E+3)-cosf(q2)*cosf(q3)*sinf(q1)*sinf(q5)*(8.9E+1/1.0E+3)+sinf(q1)*sinf(q2)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q2)*cosf(q4)*cosf(q5)*sinf(q1)*sinf(q3)*(8.9E+1/1.0E+3)-cosf(q3)*cosf(q4)*cosf(q5)*sinf(q1)*sinf(q2)*(8.9E+1/1.0E+3);
    Jacobian_result(1, 1) = sinf(q2)*(-9.0/5.0E+1)-cosf(q2)*cosf(q3)*1.7635E-1-cosf(q2)*sinf(q3)*4.35E-2-cosf(q3)*sinf(q2)*4.35E-2+sinf(q2)*sinf(q3)*1.7635E-1-cosf(q2)*cosf(q3)*cosf(q5)*(8.9E+1/1.0E+3)+cosf(q5)*sinf(q2)*sinf(q3)*(8.9E+1/1.0E+3)+cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3);
    Jacobian_result(1, 2) = cosf(q2)*cosf(q3)*(-1.7635E-1)-cosf(q2)*sinf(q3)*4.35E-2-cosf(q3)*sinf(q2)*4.35E-2+sinf(q2)*sinf(q3)*1.7635E-1-cosf(q2)*cosf(q3)*cosf(q5)*(8.9E+1/1.0E+3)+cosf(q5)*sinf(q2)*sinf(q3)*(8.9E+1/1.0E+3)+cosf(q2)*cosf(q4)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q3)*cosf(q4)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3);
    Jacobian_result(1, 3) = cosf(q2+q3)*sinf(q4)*sinf(q5)*(8.9E+1/1.0E+3);
    Jacobian_result(1, 4) = cosf(q2)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q3)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q2)*cosf(q3)*cosf(q4)*cosf(q5)*(8.9E+1/1.0E+3)+cosf(q4)*cosf(q5)*sinf(q2)*sinf(q3)*(8.9E+1/1.0E+3);
    Jacobian_result(2, 0) = sinf(q1)*(-2.342E-2)-sinf(q1)*sinf(q2)*(9.0/5.0E+1)+sinf(q1)*sinf(q2)*sinf(q3)*1.7635E-1-cosf(q2+q3)*cosf(q5)*sinf(q1)*(8.9E+1/1.0E+3)-cosf(q2)*cosf(q3)*sinf(q1)*1.7635E-1-cosf(q2)*sinf(q1)*sinf(q3)*4.35E-2-cosf(q3)*sinf(q1)*sinf(q2)*4.35E-2-cosf(q1)*sinf(q4)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q2)*cosf(q4)*sinf(q1)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q3)*cosf(q4)*sinf(q1)*sinf(q2)*sinf(q5)*(8.9E+1/1.0E+3);
    Jacobian_result(2, 1) = cosf(q1)*(cosf(q2+q3)*4.35E-2-sinf(q2+q3)*1.7635E-1+cosf(q2)*(9.0/5.0E+1)+sinf(q4-q5)*cosf(q2+q3)*4.45E-2-cosf(q2+q3)*sinf(q4+q5)*4.45E-2-sinf(q2+q3)*cosf(q5)*(8.9E+1/1.0E+3));
    Jacobian_result(2, 2) = cosf(q1)*(cosf(q2+q3)*-8.7E+2+sinf(q2+q3)*3.527E+3+sinf(q2+q3)*cosf(q5)*1.78E+3+cosf(q2+q3)*cosf(q4)*sinf(q5)*1.78E+3)*(-5.0E-5);
    Jacobian_result(2, 3) = sinf(q5)*(-cosf(q4)*sinf(q1)+cosf(q1)*cosf(q2)*sinf(q3)*sinf(q4)+cosf(q1)*cosf(q3)*sinf(q2)*sinf(q4))*(8.9E+1/1.0E+3);
    Jacobian_result(2, 4) = cosf(q5)*sinf(q1)*sinf(q4)*(-8.9E+1/1.0E+3)-cosf(q1)*cosf(q2)*cosf(q3)*sinf(q5)*(8.9E+1/1.0E+3)+cosf(q1)*sinf(q2)*sinf(q3)*sinf(q5)*(8.9E+1/1.0E+3)-cosf(q1)*cosf(q2)*cosf(q4)*cosf(q5)*sinf(q3)*(8.9E+1/1.0E+3)-cosf(q1)*cosf(q3)*cosf(q4)*cosf(q5)*sinf(q2)*(8.9E+1/1.0E+3);
    Jacobian_result(3, 1) = cosf(q1);
    Jacobian_result(3, 2) = cosf(q1);
    Jacobian_result(3, 3) = cosf(q2+q3)*sinf(q1);
    Jacobian_result(3, 4) = cosf(q1)*cosf(q4)+sinf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2));
    Jacobian_result(3, 5) = sinf(q5)*(cosf(q1)*sinf(q4)-cosf(q4)*(cosf(q2)*sinf(q1)*sinf(q3)+cosf(q3)*sinf(q1)*sinf(q2)))-cosf(q5)*(sinf(q1)*sinf(q2)*sinf(q3)-cosf(q2)*cosf(q3)*sinf(q1));
    Jacobian_result(4, 0) = 1.0;
    Jacobian_result(4, 3) = -sinf(q2+q3);
    Jacobian_result(4, 4) = cosf(q2+q3)*sinf(q4);
    Jacobian_result(4, 5) = -sinf(q2+q3)*cosf(q5)-cosf(q2+q3)*cosf(q4)*sinf(q5);
    Jacobian_result(5, 1) = -sinf(q1);
    Jacobian_result(5, 2) = -sinf(q1);
    Jacobian_result(5, 3) = cosf(q2+q3)*cosf(q1);
    Jacobian_result(5, 4) = -cosf(q4)*sinf(q1)+sinf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2));
    Jacobian_result(5, 5) = -sinf(q5)*(sinf(q1)*sinf(q4)+cosf(q4)*(cosf(q1)*cosf(q2)*sinf(q3)+cosf(q1)*cosf(q3)*sinf(q2)))+cosf(q5)*(cosf(q1)*cosf(q2)*cosf(q3)-cosf(q1)*sinf(q2)*sinf(q3));
}

inline BLA::Matrix<6, 1, float> IRAM_ATTR getIK(BLA::Matrix<6, 1, float>& current_joint_config, const Pose* target_pose, int& itr_counter)
{
    float dampening_factor = 0.01f; // Adjust as needed for stability
    float step_size = 0.05f; // Adjust as needed for convergence speed
    Pose current_pose;
    Pose FK_result_container;
    Joints result;
    BLA::Matrix<6, 1, float> error;
    BLA::Matrix<6, 1, float> delta_q;
    BLA::Matrix<4, 4, float> T0_ee;
    BLA::Matrix<3, 3, float> current_rot_matrix;
    BLA::Matrix<3, 3, float> target_rot_matrix;
    BLA::Matrix<3, 3, float> rotation_err;
    BLA::Matrix<3, 1, float> angular_error;
    BLA::Matrix<6, 6, float> J_dampened_least_squares;
    BLA::Matrix<6, 6, float> I;
    I.Fill(0);
    I(0,0) = 1.0f; I(1,1) = 1.0f; I(2,2) = 1.0f; I(3,3) = 1.0f; I(4,4) = 1.0f; I(5,5) = 1.0f;
    itr_counter = 0;
    error.Fill(1.0f);


    while (BLA::Norm(error) > 1e-4f && itr_counter < 100)
    {
        // Serial.print("Iteration: ");Serial.println(itr_counter);
        itr_counter++;

        getFK(current_joint_config, current_pose, T0_ee);

        error(0) = target_pose->x - current_pose.x;
        error(1) = target_pose->y - current_pose.y;
        error(2) = target_pose->z - current_pose.z;
        
        current_rot_matrix = T0_ee.Submatrix<3,3>(0,0);
        target_rot_matrix = getRotationMatrix(target_pose->rx, target_pose->ry, target_pose->rz);
        rotation_err = target_rot_matrix * BLA::MatrixTranspose(current_rot_matrix);

        // Extract the [dRx, dRy, dRz] vector
        angular_error = getAxisAngleError(rotation_err);
        error(3) = angular_error(0);
        error(4) = angular_error(1);
        error(5) = angular_error(2);


        BLA::Matrix<6, 6, float> Jacobian;
        fillJacobian(current_joint_config, Jacobian);
        auto tmp = (Jacobian * BLA::MatrixTranspose(Jacobian) + dampening_factor*dampening_factor * I);
        J_dampened_least_squares = BLA::MatrixTranspose(Jacobian) * BLA::Inverse(tmp);

        delta_q = J_dampened_least_squares * error;

        float max_step = 0.1f; // Max ~2.8 degrees per step
        for (int j = 0; j < 6; j++) {
            if (delta_q(j, 0) > max_step) delta_q(j, 0) = max_step;
            if (delta_q(j, 0) < -max_step) delta_q(j, 0) = -max_step;
        }
        current_joint_config += delta_q;
        // radToDeg(current_joint_config);
        // Serial.print("[");
        // for(int j=0; j<6; j++){
        //     Serial.print(current_joint_config(j, 0) * (180.0f / PI)); 
        //     if(j < 5) Serial.print(", ");
        // }
        // Serial.println("]");
        // Serial.println(" ");
    }
        // getFK(current_joint_config, FK_result_container, T0_ee);
        // Serial.println("FK result:");
        // printPose(FK_result_container);
        // Serial.println(" ");
        radToDeg(current_joint_config);
    return current_joint_config;

}

