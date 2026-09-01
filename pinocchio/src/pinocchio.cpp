#include <chrono>
#include <iostream>
#include <iomanip>
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

//
#include <pinocchio/src/math/rpy.hxx>

#include "benchmark.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json



Eigen::VectorXd getAngle();
void benchmarkKinematics(pinocchio::Data &data, pinocchio::FrameIndex tool_frame);
pinocchio::Model model;
pinocchio::Data::Matrix6x J(6, model.nv);
int main() {
    json j;
    j.dump();
    pinocchio::urdf::buildModel(PAROL_URDF_PATH, model);
    pinocchio::Data data(model);
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


    // auto r = getAngle();
    // pinocchio::framesForwardKinematics(model, data, r);
    // std::cout << "TCP pose in world:\n" << data.oMf[tool_frame].translation() << '\n';
    // std::cout << "TCP absolute rotvec in world:\n" << pinocchio::rpy::matrixToRpy(R_home.transpose() * data.oMf[tool_frame].rotation()) << '\n';

    // auto r = getAngle();
    // pinocchio::computeFrameJacobian(model, data, r, tool_frame, pinocchio::LOCAL_WORLD_ALIGNED,J);
    // std::cout << "Jacobian is: \n" << J << std::endl;

    // benchmarkKinematics(model, data, tool_frame);
}   


// Eigen::MatrixXd getJacobian(Eigen::VectorXd q_input)
// {
//     // pinocchio::computeJointJacobian(model, da)

// }