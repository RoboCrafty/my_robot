
#include <Eigen/Core>

#include <boost/core/ref.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/functional.hpp>
#include <boost/variant.hpp>

#include <iostream>
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/eigen-common.hpp"
#include "pinocchio/math.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/spatial.hpp"
#include "pinocchio/utils/check.hpp"
#include <pinocchio/algorithm/model.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include <zmq.h>

int main(){
    // pinocchio::forwardKinematics(1);
    const std::string urdf_filename = "/Users/fudayl/git/my_robot/pinocchio/src/assets/parol6.urdf";
    pinocchio::Model model;
    // pinocchio::urdf::buildModel(urdf_filename,model);
    pinocchio::buildModels::manipulator(model);
    pinocchio::Data data(model);

    Eigen::VectorXd q_input(6);
    q_input << 0.0, 1.0, 11.0, 1.57, 0.0, 0.1;

    // data.q_in = q_input;

    int end_effector_frame = model.getFrameId("tool"); 

    std::cout << "frame id is: " << end_effector_frame << std::endl;


    int base_frame = model.getFrameId("base_link"); 
    if (base_frame != -1) {
        auto base_pose = data.oMf[base_frame];
        std::cout << "\nBase Link Pose (Relative to World):" << std::endl;
        std::cout << base_pose << std::endl;
    } else {
        std::cerr << "Warning: 'base_link' not found!" << std::endl;
    }

    std::cout << "--- Performing Forward Kinematics (FK) ---" << std::endl;

    // 4. Run Forward Kinematics (The Calculation)
    pinocchio::forwardKinematics(model, data, q_input);

    auto pose = data.oMf[17];
    // auto position = pose.;

    std::cout << "Target Frame ('tool') Position (X, Y, Z):" << std::endl;
    std::cout << pose << std::endl; 

    


}