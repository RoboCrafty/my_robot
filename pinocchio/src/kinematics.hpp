#pragma once

// NOTE: pinocchio/fwd.hpp must be the first Pinocchio/Eigen/Boost header in any
// translation unit, so include this header before everything else.
#include <pinocchio/fwd.hpp>
#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial.hpp>
#include <pinocchio/math.hpp>

#include <chrono>
#include <cmath>
#include <iostream>

// All angles are RADIANS and all lengths are METRES at this boundary.
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
    // Singularity conditioning. sigma_min below sing_eps_ ramps damping in;
    // both are arm-specific -- watch the reported sigma_min and tune.
    double  sing_eps_ = 0.02;
    double  sing_lambda_max_ = 0.05;

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

    struct IkResult { int status; int iters; Eigen::VectorXd q; };
    // Status: 1 -> success
    // Status: 2 -> failed, out of reach
    // Status: 3 -> failed, could not converge within iteration limit

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

    // ---- Cartesian primitives for the real-time loop ----

    pinocchio::SE3 fkPose(const Eigen::VectorXd& q){
        pinocchio::framesForwardKinematics(model_, data_, q);
        return data_.oMf[tool_frame_id_];
    }

    static pinocchio::SE3 poseToSE3(const Eigen::Matrix<double,6,1>& p){
        const Eigen::Vector3d rpy = p.tail<3>();
        const Eigen::Vector3d xyz = p.head<3>();
        return pinocchio::SE3(pinocchio::rpy::rpyToMatrix(rpy), xyz);
    }

    static Eigen::Matrix<double,6,1> se3ToPose(const pinocchio::SE3& T){
        Eigen::Matrix<double,6,1> p;
        p.head<3>() = T.translation();
        p.tail<3>() = pinocchio::rpy::matrixToRpy(T.rotation());
        return p;
    }

    // Straight-line Cartesian interpolation: lerp position, geodesic slerp rotation.
    // Deliberately NOT the SE(3) geodesic (exp6/log6), which traces a screw/helix.
    static pinocchio::SE3 interpolatePose(const pinocchio::SE3& a, const pinocchio::SE3& b, double s){
        const Eigen::Vector3d p = a.translation() + s * (b.translation() - a.translation());
        const Eigen::Matrix3d R_ab = a.rotation().transpose() * b.rotation();
        const Eigen::Vector3d w = pinocchio::log3(R_ab);
        const Eigen::Matrix3d R = a.rotation() * pinocchio::exp3(Eigen::Vector3d(s * w));
        return pinocchio::SE3(R, p);
    }

    // Path lengths between two poses, in metres and radians.
    static void poseDistance(const pinocchio::SE3& a, const pinocchio::SE3& b,
                             double& lin, double& ang){
        lin = (b.translation() - a.translation()).norm();
        const Eigen::Matrix3d R_ab = a.rotation().transpose() * b.rotation();
        ang = pinocchio::log3(R_ab).norm();
    }

    // Pose error as a world-aligned twist [v; w], matching a LOCAL_WORLD_ALIGNED Jacobian.
    static Eigen::Matrix<double,6,1> poseError(const pinocchio::SE3& current,
                                               const pinocchio::SE3& desired){
        Eigen::Matrix<double,6,1> e;
        e.head<3>() = desired.translation() - current.translation();
        const Eigen::Matrix3d R_err = desired.rotation() * current.rotation().transpose();
        e.tail<3>() = pinocchio::log3(R_err);
        return e;
    }

    struct RrmcResult {
        double manipulability; // sqrt(det(J J^T))
        double sigma_min;      // smallest singular value of J -- 0 at a singularity
        double damping;        // lambda^2 actually applied
        double track_err;      // fraction of the requested twist NOT achieved, 0..1
    };

    // Smallest singular value of the tool Jacobian: distance to a singularity.
    double sigmaMin(const Eigen::VectorXd& q,
                    pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED){
        J_.setZero();
        pinocchio::computeFrameJacobian(model_, data_, q, tool_frame_id_, rf, J_);
        JJ_t_.noalias() = J_ * J_.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> es(JJ_t_, Eigen::EigenvaluesOnly);
        return std::sqrt(std::max(0.0, es.eigenvalues()(0)));
    }

    // Damped least-squares resolved rate with Chiaverini-style adaptive damping:
    // no damping in well-conditioned poses, ramping in smoothly near a singularity
    // so dq stays bounded instead of blowing up.
    // track_err is DIRECTIONAL: it only rises for twist components the arm truly
    // cannot produce, so pure translation still works at a wrist singularity.
    RrmcResult resolvedRate(const Eigen::VectorXd& q,
                            const Eigen::Matrix<double,6,1>& twist,
                            Eigen::VectorXd& dq_out,
                            pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED){
        J_.setZero();
        pinocchio::computeFrameJacobian(model_, data_, q, tool_frame_id_, rf, J_);
        JJ_t_.noalias() = J_ * J_.transpose();

        RrmcResult r;
        r.manipulability = std::sqrt(std::max(0.0, JJ_t_.determinant()));
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> es(JJ_t_, Eigen::EigenvaluesOnly);
        r.sigma_min = std::sqrt(std::max(0.0, es.eigenvalues()(0)));

        r.damping = 0.0;
        if (r.sigma_min < sing_eps_) {
            const double k = 1.0 - r.sigma_min / sing_eps_;
            r.damping = sing_lambda_max_ * sing_lambda_max_ * k * k;
        }
        JJ_t_.diagonal().array() += (ik_damping + r.damping);
        dq_out.noalias() = J_.transpose() * JJ_t_.ldlt().solve(twist);

        const double tn = twist.norm();
        r.track_err = (tn > 1e-9) ? ((J_ * dq_out) - twist).norm() / tn : 0.0;
        return r;
    }

    // @param p A 6D vector of pose and orintation
    // @return A 6D vector of joint angles
    IkResult InverseKinematics_Positional(const pinocchio::SE3& target_oMF, const Eigen::VectorXd& q_init){

        IkResult result;

        if(target_oMF.translation().norm() > max_reach_){
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
                result.q = q;
                result.status = 1;
                result.iters = i;
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
        result.iters = ik_max_iters;
        result.status = 3;
        return result;
    }

    bool isPoseReachable(const pinocchio::SE3& target_oMF){
        Eigen::VectorXd q(6); q << 0,0,0,0,0,0;
        auto res = this->InverseKinematics_Positional(target_oMF, q);
        return res.status == 1;
    }

    // Getters
    const pinocchio::Model& get_model(){
        return model_;
    }

    const pinocchio::Data& get_data(){
        return data_;
    }
};
