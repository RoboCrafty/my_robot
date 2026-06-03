#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <helper_functions.h>
#include <structs.h>

float ez = 89.0f; // End-effector length in mm
/**
 * Calculates the forward kinematics transformation matrix.
 * @param q Array of 6 joint angles (q1 to q6 in radians) passed via the Joints struct
 * @return 4x4 Homogeneous Transformation Matrix
 */
inline void IRAM_ATTR getFK(BLA::Matrix<6, 1, float>& q,const TrigValues& tc, Pose& result, BLA::Matrix<4,4, float>& T0_ee_result) {
    
    float q1 = q(0, 0), q2 = q(1, 0), q3 = q(2, 0), q4 = q(3, 0), q5 = q(4, 0), q6 = q(5, 0);

    T0_ee_result.Fill(0.0f);
    T0_ee_result(0, 0) = tc.c6*(tc.c1*tc.c4+tc.s4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))-tc.s6*(tc.c5*(tc.c1*tc.s4-tc.c4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))+tc.s5*(tc.s1*tc.s2*tc.s3-tc.c2*tc.c3*tc.s1));
    T0_ee_result(0, 1) = -tc.s6*(tc.c1*tc.c4+tc.s4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))-tc.c6*(tc.c5*(tc.c1*tc.s4-tc.c4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))+tc.s5*(tc.s1*tc.s2*tc.s3-tc.c2*tc.c3*tc.s1));
    T0_ee_result(0, 2) = tc.s5*(tc.c1*tc.s4-tc.c4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))-tc.c5*(tc.s1*tc.s2*tc.s3-tc.c2*tc.c3*tc.s1);
    T0_ee_result(0, 3) = tc.s1*2.342E-2+tc.s1*tc.s2*(9.0/5.0E+1)+(ez*(tc.s5*(tc.c1*tc.s4-tc.c4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))-tc.c5*(tc.s1*tc.s2*tc.s3-tc.c2*tc.c3*tc.s1)))/1.0E+3-tc.s1*tc.s2*tc.s3*1.7635E-1+tc.c2*tc.c3*tc.s1*1.7635E-1+tc.c2*tc.s1*tc.s3*4.35E-2+tc.c3*tc.s1*tc.s2*4.35E-2;
    T0_ee_result(1, 0) = -tc.s6*(sinf(q2+q3)*tc.s5-cosf(q2+q3)*tc.c4*tc.c5)+cosf(q2+q3)*tc.c6*tc.s4;
    T0_ee_result(1, 1) = -tc.c6*(sinf(q2+q3)*tc.s5-cosf(q2+q3)*tc.c4*tc.c5)-cosf(q2+q3)*tc.s4*tc.s6;
    T0_ee_result(1, 2) = -sinf(q2+q3)*tc.c5-cosf(q2+q3)*tc.c4*tc.s5;
    T0_ee_result(1, 3) = cosf(q2+q3)*4.35E-2-sinf(q2+q3)*1.7635E-1+tc.c2*(9.0/5.0E+1)-(ez*cosf(q2+q3)*sinf(q4+q5))/2.0E+3-(ez*sinf(q2+q3)*tc.c5)/1.0E+3+(ez*sinf(q4-q5)*cosf(q2+q3))/2.0E+3+1.105E-1;
    T0_ee_result(2, 0) = -tc.c6*(tc.c4*tc.s1-tc.s4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))+tc.s6*(tc.c5*(tc.s1*tc.s4+tc.c4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))+tc.s5*(tc.c1*tc.c2*tc.c3-tc.c1*tc.s2*tc.s3));
    T0_ee_result(2, 1) = tc.s6*(tc.c4*tc.s1-tc.s4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))+tc.c6*(tc.c5*(tc.s1*tc.s4+tc.c4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))+tc.s5*(tc.c1*tc.c2*tc.c3-tc.c1*tc.s2*tc.s3));
    T0_ee_result(2, 2) = -tc.s5*(tc.s1*tc.s4+tc.c4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))+tc.c5*(tc.c1*tc.c2*tc.c3-tc.c1*tc.s2*tc.s3);
    T0_ee_result(2, 3) = tc.c1*2.342E-2+tc.c1*tc.s2*(9.0/5.0E+1)-(ez*(tc.s5*(tc.s1*tc.s4+tc.c4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))-tc.c5*(tc.c1*tc.c2*tc.c3-tc.c1*tc.s2*tc.s3)))/1.0E+3+tc.c1*tc.c2*tc.c3*1.7635E-1+tc.c1*tc.c2*tc.s3*4.35E-2+tc.c1*tc.c3*tc.s2*4.35E-2-tc.c1*tc.s2*tc.s3*1.7635E-1;
    T0_ee_result(3, 3) = 1.0;
    result = extract_pose_from_transformation_matrix(T0_ee_result);
}

inline TrigValues IRAM_ATTR computeTrigValues(const BLA::Matrix<6, 1, float>& joints)
{
    TrigValues trig;

    float q1 = joints(0, 0), q2 = joints(1, 0), q3 = joints(2, 0), q4 = joints(3, 0), q5 = joints(4, 0), q6 = joints(5, 0);

    trig.s1 = sinf(q1); trig.c1 = cosf(q1);
    trig.s2 = sinf(q2); trig.c2 = cosf(q2);
    trig.s3 = sinf(q3); trig.c3 = cosf(q3);
    trig.s4 = sinf(q4); trig.c4 = cosf(q4);
    trig.s5 = sinf(q5); trig.c5 = cosf(q5);
    trig.s6 = sinf(q6); trig.c6 = cosf(q6);

    return trig;
}

/**
 * Calculates the Jacobian matrix for the robotic arm.
 * @param q Array of 6 joint angles (q1 to q6 in radians) passed via the Joints struct
 * @return Doesn't return anything, but modifies the Jacobian_result matrix
 */
inline void IRAM_ATTR fillJacobian(BLA::Matrix<6, 1, float>& q,const TrigValues& tc, BLA::Matrix<6,6, float>& Jacobian_result)
{
    float q1 = q(0, 0), q2 = q(1, 0), q3 = q(2, 0), q4 = q(3, 0), q5 = q(4, 0), q6 = q(5, 0);
    float sigma = (87*cosf(q2 + q3))/2000 - (3527*sinf(q2 + q3))/20000 + (9*tc.c2)/50 - (ez*cosf(q2 + q3)*sinf(q4 + q5))/2000 - (ez*sinf(q2 + q3)*tc.c5)/1000 + (ez*sinf(q4 - q5)*cosf(q2 + q3))/2000;

    Jacobian_result.Fill(0.0f);
    Jacobian_result(0, 0) = tc.c1*2.342E-2+tc.c1*tc.s2*(9.0/5.0E+1)-(ez*(tc.s5*(tc.s1*tc.s4+tc.c4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))-tc.c5*(tc.c1*tc.c2*tc.c3-tc.c1*tc.s2*tc.s3)))/1.0E+3+tc.c1*tc.c2*tc.c3*1.7635E-1+tc.c1*tc.c2*tc.s3*4.35E-2+tc.c1*tc.c3*tc.s2*4.35E-2-tc.c1*tc.s2*tc.s3*1.7635E-1;
    Jacobian_result(0, 1) = sigma*tc.s1;
    Jacobian_result(0, 2) = tc.s1*(cosf(q2+q3)*-8.7E+2+sinf(q2+q3)*3.527E+3+ez*sinf(q2+q3)*tc.c5*2.0E+1+ez*cosf(q2+q3)*tc.c4*tc.s5*2.0E+1)*(-5.0E-5);
    Jacobian_result(0, 3) = (ez*tc.s5*(tc.c1*tc.c4+tc.c2*tc.s1*tc.s3*tc.s4+tc.c3*tc.s1*tc.s2*tc.s4))/1.0E+3;
    Jacobian_result(0, 4) = (ez*tc.c1*tc.c5*tc.s4)/1.0E+3-(ez*tc.c2*tc.c3*tc.s1*tc.s5)/1.0E+3+(ez*tc.s1*tc.s2*tc.s3*tc.s5)/1.0E+3-(ez*tc.c2*tc.c4*tc.c5*tc.s1*tc.s3)/1.0E+3-(ez*tc.c3*tc.c4*tc.c5*tc.s1*tc.s2)/1.0E+3;
    Jacobian_result(1, 1) = tc.s2*(-9.0/5.0E+1)-tc.c2*tc.c3*1.7635E-1-tc.c2*tc.s3*4.35E-2-tc.c3*tc.s2*4.35E-2+tc.s2*tc.s3*1.7635E-1-(ez*tc.c2*tc.c3*tc.c5)/1.0E+3+(ez*tc.c5*tc.s2*tc.s3)/1.0E+3+(ez*tc.c2*tc.c4*tc.s3*tc.s5)/1.0E+3+(ez*tc.c3*tc.c4*tc.s2*tc.s5)/1.0E+3;
    Jacobian_result(1, 2) = tc.c2*tc.c3*(-1.7635E-1)-tc.c2*tc.s3*4.35E-2-tc.c3*tc.s2*4.35E-2+tc.s2*tc.s3*1.7635E-1-(ez*tc.c2*tc.c3*tc.c5)/1.0E+3+(ez*tc.c5*tc.s2*tc.s3)/1.0E+3+(ez*tc.c2*tc.c4*tc.s3*tc.s5)/1.0E+3+(ez*tc.c3*tc.c4*tc.s2*tc.s5)/1.0E+3;
    Jacobian_result(1, 3) = (ez*cosf(q2+q3)*tc.s4*tc.s5)/1.0E+3;
    Jacobian_result(1, 4) = (ez*tc.c2*tc.s3*tc.s5)/1.0E+3+(ez*tc.c3*tc.s2*tc.s5)/1.0E+3-(ez*tc.c2*tc.c3*tc.c4*tc.c5)/1.0E+3+(ez*tc.c4*tc.c5*tc.s2*tc.s3)/1.0E+3;
    Jacobian_result(2, 0) = tc.s1*(-2.342E-2)-tc.s1*tc.s2*(9.0/5.0E+1)-(ez*(tc.s5*(tc.c1*tc.s4-tc.c4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))-tc.c5*(tc.s1*tc.s2*tc.s3-tc.c2*tc.c3*tc.s1)))/1.0E+3+tc.s1*tc.s2*tc.s3*1.7635E-1-tc.c2*tc.c3*tc.s1*1.7635E-1-tc.c2*tc.s1*tc.s3*4.35E-2-tc.c3*tc.s1*tc.s2*4.35E-2;
    Jacobian_result(2, 1) = sigma*tc.c1;
    Jacobian_result(2, 2) = tc.c1*(cosf(q2+q3)*-8.7E+2+sinf(q2+q3)*3.527E+3+ez*sinf(q2+q3)*tc.c5*2.0E+1+ez*cosf(q2+q3)*tc.c4*tc.s5*2.0E+1)*(-5.0E-5);
    Jacobian_result(2, 3) = (ez*tc.s5*(-tc.c4*tc.s1+tc.c1*tc.c2*tc.s3*tc.s4+tc.c1*tc.c3*tc.s2*tc.s4))/1.0E+3;
    Jacobian_result(2, 4) = ez*tc.c5*tc.s1*tc.s4*(-1.0/1.0E+3)-(ez*tc.c1*tc.c2*tc.c3*tc.s5)/1.0E+3+(ez*tc.c1*tc.s2*tc.s3*tc.s5)/1.0E+3-(ez*tc.c1*tc.c2*tc.c4*tc.c5*tc.s3)/1.0E+3-(ez*tc.c1*tc.c3*tc.c4*tc.c5*tc.s2)/1.0E+3;
    Jacobian_result(3, 1) = tc.c1;
    Jacobian_result(3, 2) = tc.c1;
    Jacobian_result(3, 3) = cosf(q2+q3)*tc.s1;
    Jacobian_result(3, 4) = tc.c1*tc.c4+tc.s4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2);
    Jacobian_result(3, 5) = tc.s5*(tc.c1*tc.s4-tc.c4*(tc.c2*tc.s1*tc.s3+tc.c3*tc.s1*tc.s2))-tc.c5*(tc.s1*tc.s2*tc.s3-tc.c2*tc.c3*tc.s1);
    Jacobian_result(4, 0) = 1.0;
    Jacobian_result(4, 3) = -sinf(q2+q3);
    Jacobian_result(4, 4) = cosf(q2+q3)*tc.s4;
    Jacobian_result(4, 5) = -sinf(q2+q3)*tc.c5-cosf(q2+q3)*tc.c4*tc.s5;
    Jacobian_result(5, 1) = -tc.s1;
    Jacobian_result(5, 2) = -tc.s1;
    Jacobian_result(5, 3) = cosf(q2+q3)*tc.c1;
    Jacobian_result(5, 4) = -tc.c4*tc.s1+tc.s4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2);
    Jacobian_result(5, 5) = -tc.s5*(tc.s1*tc.s4+tc.c4*(tc.c1*tc.c2*tc.s3+tc.c1*tc.c3*tc.s2))+tc.c5*(tc.c1*tc.c2*tc.c3-tc.c1*tc.s2*tc.s3);
}
std::vector<float> time_container;

inline BLA::Matrix<6, 1, float> IRAM_ATTR getIK(BLA::Matrix<6, 1, float>& current_joint_config, const Pose* target_pose, int& itr_counter)
{
    float dampening_factor = 0.01f; // Adjust as needed for stability
    float max_step = 0.4f; // Max ~2.8 degrees per step
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
    BLA::Matrix<6, 6, float> J_simple_inverse;
    BLA::Matrix<6, 6, float> I;
    BLA::Matrix<6, 6, float> Jacobian;
    TrigValues trig_cache;
    auto start_time = micros();
    auto end_time = micros();
    float time = (float)(end_time - start_time);
    I.Fill(0);
    I(0,0) = 1.0f; I(1,1) = 1.0f; I(2,2) = 1.0f; I(3,3) = 1.0f; I(4,4) = 1.0f; I(5,5) = 1.0f;
    itr_counter = 0;
    error.Fill(1.0f);

    Serial.println("Starting IK solver...");
    while (BLA::Norm(error) > 1e-4f && itr_counter < 100)
    {
        // Serial.print("Iteration: ");Serial.println(itr_counter);
        itr_counter++;
        // step 0
        // start_time = micros();
        trig_cache = computeTrigValues(current_joint_config);
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 0: "); Serial.print(time); Serial.println(" microseconds");
        // Step 1
        // start_time = micros();
        getFK(current_joint_config, trig_cache, current_pose, T0_ee);
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 1: "); Serial.print(time); Serial.println(" microseconds");
      
        

        // Step 2: Compute the error vector (6x1)
        // start_time = micros();
        error(0) = target_pose->x - current_pose.x;
        error(1) = target_pose->y - current_pose.y;
        error(2) = target_pose->z - current_pose.z;
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 2: "); Serial.print(time); Serial.println(" microseconds");
      
        

        // Step 3
        // start_time = micros();
        current_rot_matrix = T0_ee.Submatrix<3,3>(0,0);
        target_rot_matrix = getRotationMatrix(target_pose->rx, target_pose->ry, target_pose->rz);
        rotation_err = target_rot_matrix * BLA::MatrixTranspose(current_rot_matrix);
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 3: "); Serial.print(time); Serial.println(" microseconds");
        

        // Extract the [dRx, dRy, dRz] vector
        // step 4
        // start_time = micros();
        angular_error = getAxisAngleError(rotation_err);
        error(3) = angular_error(0);
        error(4) = angular_error(1);
        error(5) = angular_error(2);
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 4: "); Serial.print(time); Serial.println(" microseconds");
        

        // start_time = micros();
        fillJacobian(current_joint_config, trig_cache, Jacobian); //step 5
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 5: "); Serial.print(time); Serial.println(" microseconds");
        

        // start_time = micros();
        J_dampened_least_squares = (Jacobian * BLA::MatrixTranspose(Jacobian) + dampening_factor*dampening_factor * I); // step 6
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 6: "); Serial.print(time); Serial.println(" microseconds");
        

        // start_time = micros();
        J_dampened_least_squares = BLA::MatrixTranspose(Jacobian) * BLA::Inverse(J_dampened_least_squares); //step 7
        // end_time = micros();
        // time = (float)(end_time - start_time);
        // Serial.print("Time taken for step 7: "); Serial.print(time); Serial.println(" microseconds");
        
        // J_simple_inverse = BLA::Inverse(Jacobian);

        delta_q = J_dampened_least_squares * error;

        
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
        // Serial.println("---***-");
    }
        // getFK(current_joint_config, FK_result_container, T0_ee);
        // Serial.println("FK result:");
        // printPose(FK_result_container);
        // Serial.println(" ");
        // for (int i = 0; i <= time_container.size(); i++) {
        //     Serial.println(); Serial.print("Time taken for step: "); Serial.print(i); Serial.print(" ");
        //     Serial.print(time_container[i]); Serial.print(" microseconds, ");
        // }
        radToDeg(current_joint_config);
    return current_joint_config;

}

