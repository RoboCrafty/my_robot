#include <chrono>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>

// Core Pinocchio includes
#include <pinocchio/fwd.hpp>
#include <Eigen/Dense>

// Pinocchio algorithms and parsers
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial.hpp>
#include <pinocchio/math.hpp>


#include "benchmark.h"

#include <nlohmann/json.hpp>

// using json = nlohmann::json

class Kinematics{
    private:
    // Private Variables
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex base_frame_id_;
    pinocchio::FrameIndex tool_frame_id_;
    pinocchio::Data::Matrix6x J_;
    Eigen::Matrix<double, 6, 6> JJ_t_;

    double  max_reach_;
    double  ik_pos_tol = 1e-3;
    double  ik_rot_tol = 1e-2; 
    int     ik_max_iters = 100;
    double  ik_max_step_rad = 0.5;
    double  ik_damping = 1e-4;

    struct IkResult { int status; int iters; Eigen::VectorXd q; };
    // Status: 1 -> success
    // Status: 2 -> failed, out of reach
    // Status: 3 -> failed, could not converge within iteration limit


    // Private Functions
    double computeRobotReachFromModel()
    {
        double max_reach = 0.0;

        // 1. Add tool frame's local offset relative to its parent joint
        const auto& tool_frame = model_.frames[tool_frame_id_];
        max_reach += tool_frame.placement.translation().norm();

        // 2. Trace parent joints up to the root (Joint 0 is Universe)
        pinocchio::JointIndex current_joint_id = tool_frame.parentJoint;

        while (current_joint_id > 0)
        {
            // Add the distance between this joint and its parent joint
            const auto& placement = model_.jointPlacements[current_joint_id];
            max_reach += placement.translation().norm();

            // Step up the tree
            current_joint_id = model_.parents[current_joint_id];
        }

        return max_reach;
    }

    
    public:
    // Public Variables
    int nv, nq;

    // Public Functions   
    Kinematics()
    {
        pinocchio::urdf::buildModel(PAROL_URDF_PATH, model_);
        data_ = pinocchio::Data(model_);
        nq = model_.nq;
        nv = model_.nv;
        J_.resize(6, model_.nv);
        J_.setZero();
        JJ_t_.setZero();
        base_frame_id_ = model_.getFrameId("base_link");
        tool_frame_id_ = model_.getFrameId("tcp_link");
        // std::cout << "nv is " << nv << std::endl;
        if (base_frame_id_ == model_.nframes || tool_frame_id_ == model_.nframes) {
        std::cerr << "URDF must contain frames named 'base_link' and 'tcp_link'.\n";}

        max_reach_ = computeRobotReachFromModel();
        std::cout << "max reach is " << max_reach_ << std::endl;

        std::cout << "Lower joint limits are: \n " <<  model_.lowerPositionLimit * 180/M_PI << std::endl;
        std::cout << "Upper joint limits are: \n " <<  model_.upperPositionLimit * 180/M_PI << std::endl;
    }

    double getManupulabilityIndex(Eigen::VectorXd q)
    {
        J_.setZero();
        pinocchio::computeFrameJacobian(model_, data_, q, tool_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_);
        JJ_t_.noalias() = J_ * J_.transpose();
        auto det = JJ_t_.determinant();
        auto res = sqrt(std::max(0.0, det));
        return res;
    }

    // @return A 6D vector of pose and orintation [X, Y, Z, RX, RY, RZ]
    Eigen::Matrix<double, 6, 1> ForwardKinematics(Eigen::VectorXd q){
        pinocchio::framesForwardKinematics(model_, data_, q);
        Eigen::VectorXd res(6);
        res.head<3>() =  data_.oMf[tool_frame_id_].translation();
        res.tail<3>() = pinocchio::rpy::matrixToRpy(data_.oMf[tool_frame_id_].rotation());
        return res;
    }
    // @param p A 6D vector of pose and orintation
    // @return A 6D vector of joint angles
    // @note Throws error if not reachable
    IkResult InverseKinematics_Positional(const pinocchio::SE3& target_oMF, const Eigen::VectorXd& q_init){
        
        IkResult result;

        const auto ik_start = std::chrono::steady_clock::now();
        if(target_oMF.translation().norm() > max_reach_){
            std::cout << "Target is outside the robot's maximum reach" << std::endl;
            result.status = 2;
            return result;
        }
        
        Eigen::VectorXd q = q_init;
        Eigen::VectorXd q_next(model_.nq);
        
        for(int i = 0; i < ik_max_iters; i++){
            // Compute current pose 
           pinocchio::computeJointJacobians(model_, data_, q);
            pinocchio::framesForwardKinematics(model_, data_, q);
            // Compute error
            const pinocchio::SE3 dMf = data_.oMf[tool_frame_id_].actInv(target_oMF);
            pinocchio::Motion err = pinocchio::log6(dMf);
            // Check if pose reached
            if (err.linear().norm() < ik_pos_tol && err.angular().norm() < ik_rot_tol) {
                const auto ik_end = std::chrono::steady_clock::now();
                const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(ik_end - ik_start).count();
                const double elapsed_ms = std::chrono::duration<double, std::milli>(ik_end - ik_start).count();
                result.q = q;
                result.status = 1;
                result.iters = i;
                // std::cout << "IK converged in " << i << " iterations!" << std::endl;
                // std::cout << "IK time: " << elapsed_us << " us (" << elapsed_ms << " ms)\n";
                return result;
            }
            // Compute Jacobian
            J_.setZero();
            pinocchio::getFrameJacobian(model_, data_, tool_frame_id_, pinocchio::LOCAL, J_);
            // Computed Damped Psuedo Inverse Jacobian
            JJ_t_.noalias() = J_ * J_.transpose();
            JJ_t_.diagonal().array() += ik_damping;  
            Eigen::VectorXd dq = J_.transpose() * JJ_t_.ldlt().solve(err.toVector());
            if (dq.norm() > ik_max_step_rad) {
                dq = dq * (ik_max_step_rad / dq.norm());
            }
            // Update q
            pinocchio::integrate(model_, q, dq, q_next);
            q.swap(q_next);
            q = q.cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
        }
        const auto ik_end = std::chrono::steady_clock::now();
        const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(ik_end - ik_start).count();
        const double elapsed_ms = std::chrono::duration<double, std::milli>(ik_end - ik_start).count();
        // std::cout << "IK time: " << elapsed_us << " us (" << elapsed_ms << " ms)\n";
        // throw std::runtime_error(
        //     "Inverse kinematics could not converge in " +
        //     std::to_string(ik_max_iters) + " iterations");
        result.iters = ik_max_iters;
        result.status = 3;
        return result;
    }

    void InverseKinematics_RRMC(const pinocchio::SE3& target_oMF){

    }
    void moveL(){}
    void moveL_delta(){}
    void speedL(){}
    void speedJ(){}
    void moveJ(){}
    void moveJ_delta(){}


    bool isPoseReachable(const pinocchio::SE3& target_oMF){
        Eigen::VectorXd q(6); q << 0,0,0,0,0,0;
        auto res = this->InverseKinematics_Positional(target_oMF, q);
        if(res.status == 1) return true;
        else if(res.status == 2 || res.status == 3) return false;
    }

    // Getters
    const pinocchio::Model& get_model(){
        return model_;
    }

    pinocchio::Data get_data(){
        return data_;
    }

    
};

Eigen::VectorXd getAngle(int nq);
pinocchio::SE3 getTarget(int nq);


int main()
{
    Kinematics parolKins;
    std::cout << std::fixed << std::setprecision(4);

    // auto q = getAngle(parolKins.nq);
    
    // std::cout << "FK  is: \n" << parolKins.ForwardKinematics(q) << std::endl;
    // std::cout << "Manpulability Index is: " << parolKins.getManupulabilityIndex(q) << std::endl;
    // Eigen::Vector3d pose; pose << 0.5 , 0.0, 0.0;
    // pose.head<3> = parolKins.ForwardKinematics(q) 
    // Eigen::Matrix3d rpy; rpy.setIdentity();

    Eigen::VectorXd q(6); q << 0,0,0,0,0,0;

    Eigen::Matrix<double, 6, 1> fk = parolKins.ForwardKinematics(q);
    std::cout << "Starting Pose: \n" << fk << std::endl;
    
   while (1)
   {
        // auto t = getTarget(parolKins.nq);
        // std::cout << "Reachable = " << parolKins.isPoseReachable(t) << std::endl;
        auto t = getTarget(parolKins.nq);
        auto res = parolKins.InverseKinematics_Positional(t, q);

        if(res.ok == true){
            std::cout << "computed IK: \n" << res.q * 180/M_PI << std::endl;
        }
        else if(res.ok == false){

        }
        
        // std::cout << "computed IK: \n" << res << std::endl;

        return 1;
   }
    
    

}











int cat() {
    pinocchio::Model model;
    pinocchio::Data data(model);
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.resize(6, model.nv);
    J.setZero();

    const Eigen::VectorXd q_input = pinocchio::neutral(model);
    const auto base_frame = model.getFrameId("base_link");
    const auto tool_frame = model.getFrameId("tcp_link");

    if (base_frame == model.nframes || tool_frame == model.nframes) {
        std::cerr << "URDF must contain frames named 'base_link' and 'tcp_link'.\n";
        return 1;
    }

    pinocchio::framesForwardKinematics(model, data, q_input);
    // Store the absolute home rotation matrix
    Eigen::Matrix3d R_home = data.oMf[tool_frame].rotation();

    std::cout << "Base pose in world:\n" << data.oMf[base_frame] << "\n\n";
    std::cout << "TCP pose in world:\n" << data.oMf[tool_frame].translation() << '\n';
    std::cout << "TCP rotvec in world:\n" << pinocchio::rpy::matrixToRpy(data.oMf[tool_frame].rotation()) << '\n';
    std::cout << "TCP Absolute rotvec in world:\n" << pinocchio::rpy::matrixToRpy(R_home.transpose() * data.oMf[tool_frame].rotation()) << '\n';


    auto r = getAngle(model.nq);
    pinocchio::framesForwardKinematics(model, data, r);
    std::cout << "TCP pose in world:\n" << data.oMf[tool_frame].translation() << '\n';
    std::cout << "TCP absolute rotvec in world:\n" << pinocchio::rpy::matrixToRpy(R_home.transpose() * data.oMf[tool_frame].rotation()) << '\n';

    r = getAngle(model.nq);
    pinocchio::computeFrameJacobian(model, data, r, tool_frame, pinocchio::LOCAL_WORLD_ALIGNED,J);
    std::cout << "Jacobian is: \n" << J << std::endl;

    benchmarkKinematics(model, data, tool_frame);
}   


// Eigen::MatrixXd getJacobian(Eigen::VectorXd q_input)
// {
//     // pinocchio::computeJointJacobian(model, da)

// }

Eigen::VectorXd getAngle(int nq)
{
    std::cout << "Enter the joint angles (in degrees) for the " << nq << " joints, separated by spaces:\n";
    Eigen::VectorXd q_input(nq);
    for (int i = 0; i < nq; ++i) {
        std::cin >> q_input[i];
    }
    return q_input * M_PI / 180.0; // Convert degrees to radians
}

pinocchio::SE3 getTarget(int nq)
{
    std::cout << "Enter target x y z rx ry rz separated by spaces:\n";
    Eigen::VectorXd q_input(nq);
    for (int i = 0; i < nq; ++i) {
        std::cin >> q_input[i];
    }
    Eigen::Vector3d xyz = q_input.head<3>();
    Eigen::Vector3d rpy = q_input.tail<3>();
    pinocchio::SE3 res (pinocchio::rpy::rpyToMatrix(rpy), xyz);

    return res; // Convert degrees to radians
}

