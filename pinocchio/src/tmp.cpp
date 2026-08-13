#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <zmq.hpp>
#include <iostream>
#include <vector>

int main() {
    // 1. Load URDF with Pinocchio
    const std::string urdf_filename = "assets/parol6.urdf";
    pinocchio::Model model;

    pinocchio::urdf::buildModel(urdf_filename,model);
    pinocchio::Data data(model);

    // End-effector joint/frame ID (using last joint as example)
    const pinocchio::JointIndex JOINT_ID = model.njoints - 1; 

    // 2. Setup ZeroMQ REP (Reply) Socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind("tcp://*:5555");
    std::cout << "C++ Pinocchio IK Server listening on tcp://*:5555..." << std::endl;

    Eigen::VectorXd q = pinocchio::neutral(model);

    while (true) {
        // 3. Receive Target Position [x, y, z] from Python
        zmq::message_t request;
        auto result = socket.recv(request, zmq::recv_flags::none);
        if (!result) break;

        // Interpret incoming byte buffer as 3 doubles
        const double* target_pos = static_cast<const double*>(request.data());
        Eigen::Vector3d target_p(target_pos[0], target_pos[1], target_pos[2]);

        // 4. Run Pinocchio Inverse Kinematics (CLIK)
        const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), target_p);
        const double eps = 1e-4;
        const int max_iter = 50;
        const double dt = 0.1;
        const double damp = 1e-6;

        for (int i = 0; i < max_iter; ++i) {
            pinocchio::forwardKinematics(model, data, q);
            const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

            if (err.norm() < eps) break;
            model.
            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
            J.setZero();
            J = -pinocchio::Jlog6(iMd.i nverse()) * J;

            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() +=   ;
            Eigen::VectorXd v = -J.transpose() * JJt.ldlt().solve(err);

            q = pinocchio::integrate(model, q, v * dt);
        }

        // 5. Send Solved Joint Angles (q) Back to Python
        std::vector<double> q_out(q.data(), q.data() + q.size());
        zmq::message_t reply(q_out.data(), q_out.size() * sizeof(double));
        socket.send(reply, zmq::send_flags::none);
    }

    return 0;
}
