#pragma once
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

#include <pinocchio/fwd.hpp>
#include <Eigen/Dense>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

void benchmarkKinematics(pinocchio::Model &model, pinocchio::Data &data, pinocchio::FrameIndex tool_frame)
{
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    constexpr std::size_t benchmark_iterations = 10000;
    constexpr double damping = 1e-6;
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
    Eigen::MatrixXd jacobian_pseudoinverse(model.nv, 6);
    pinocchio::Data::Matrix6 damped_jacobian_product;

    const auto jacobian_pseudoinverse_start = Clock::now();
    for (const auto &configuration : configurations) {
        J.setZero();
        pinocchio::computeFrameJacobian(
            model, data, configuration, tool_frame, pinocchio::LOCAL_WORLD_ALIGNED, J);

        damped_jacobian_product.noalias() = J * J.transpose();
        damped_jacobian_product.diagonal().array() += damping;
        jacobian_pseudoinverse.noalias() =
            J.transpose() * damped_jacobian_product.ldlt().solve(
                pinocchio::Data::Matrix6::Identity());
    }
    const auto jacobian_pseudoinverse_end = Clock::now();

    volatile double benchmark_sink = jacobian_pseudoinverse.sum();
    (void)benchmark_sink;

    const auto fk_microseconds =
        std::chrono::duration<double, std::micro>(fk_end - fk_start).count();
    const auto jacobian_microseconds =
        std::chrono::duration<double, std::micro>(jacobian_end - jacobian_start).count();
    const auto jacobian_pseudoinverse_microseconds =
        std::chrono::duration<double, std::micro>(
            jacobian_pseudoinverse_end - jacobian_pseudoinverse_start).count();

    std::cout << std::fixed << std::setprecision(3)
              << "\nBenchmark over " << benchmark_iterations << " random poses:\n"
              << "Forward kinematics: " << fk_microseconds / benchmark_iterations
              << " us average (" << fk_microseconds / 1000.0 << " ms total)\n"
              << "Tool-frame Jacobian: " << jacobian_microseconds / benchmark_iterations
              << " us average (" << jacobian_microseconds / 1000.0 << " ms total)\n"
              << "Jacobian + damped pseudoinverse: "
              << jacobian_pseudoinverse_microseconds / benchmark_iterations
              << " us average (" << jacobian_pseudoinverse_microseconds / 1000.0
              << " ms total)\n";
}