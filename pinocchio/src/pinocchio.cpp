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

Eigen::VectorXd getAngle();
void benchmarkKinematics(pinocchio::Data &data, pinocchio::FrameIndex tool_frame);
pinocchio::Model model;
pinocchio::Data::Matrix6x J(6, model.nv);
int main() {
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

    benchmarkKinematics(data, tool_frame);
}   


Eigen::VectorXd getAngle()
{
    std::cout << "Enter the joint angles (in degrees) for the " << model.nq << " joints, separated by spaces:\n";
    Eigen::VectorXd q_input(model.nq);
    for (int i = 0; i < model.nq; ++i) {
        std::cin >> q_input[i];
    }
    return q_input * M_PI / 180.0; // Convert degrees to radians
}

void benchmarkKinematics(pinocchio::Data &data, pinocchio::FrameIndex tool_frame)
{
    constexpr std::size_t benchmark_iterations = 10000;
    using Clock = std::chrono::steady_clock;

    std::vector<Eigen::VectorXd> configurations;
    configurations.reserve(benchmark_iterations);
    for (std::size_t iteration = 0; iteration < benchmark_iterations; ++iteration) {
        configurations.push_back(pinocchio::randomConfiguration(model));
    }

    const auto fk_start = Clock::now();
    for (const auto &configuration : configurations) {
        pinocchio::framesForwardKinematics(model, data, configuration);
    }
    const auto fk_end = Clock::now();

    const auto jacobian_start = Clock::now();
    for (const auto &configuration : configurations) {
        J.setZero();
        pinocchio::computeFrameJacobian(
            model, data, configuration, tool_frame, pinocchio::LOCAL_WORLD_ALIGNED, J);
    }
    const auto jacobian_end = Clock::now();

    const auto fk_microseconds =
        std::chrono::duration<double, std::micro>(fk_end - fk_start).count();
    const auto jacobian_microseconds =
        std::chrono::duration<double, std::micro>(jacobian_end - jacobian_start).count();

    std::cout << std::fixed << std::setprecision(3)
              << "\nBenchmark over " << benchmark_iterations << " random poses:\n"
              << "Forward kinematics: " << fk_microseconds / benchmark_iterations
              << " us average (" << fk_microseconds / 1000.0 << " ms total)\n"
              << "Tool-frame Jacobian: " << jacobian_microseconds / benchmark_iterations
              << " us average (" << jacobian_microseconds / 1000.0 << " ms total)\n";
}

// Eigen::MatrixXd getJacobian(Eigen::VectorXd q_input)
// {
//     // pinocchio::computeJointJacobian(model, da)

// }