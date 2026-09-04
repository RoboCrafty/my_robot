#include <chrono>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include "kinematics.hpp"

#include "benchmark.h"

#include <nlohmann/json.hpp>

// using json = nlohmann::json


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

        if(res.status == 1){
            std::cout << "computed IK: \n" << res.q * 180/M_PI << std::endl;
        }
        else if(res.status == 2){
            std::cout << "IK failed: target out of reach\n";
        }
        else {
            std::cout << "IK failed: no convergence in " << res.iters << " iterations\n";
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

