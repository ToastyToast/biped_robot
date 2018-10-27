#include "biped_kinematics_dynamics/solvers/biped_ik_solver_analytical.h"

using namespace biped_kinematics_dynamics;


BipedIKSolverAnalytical::BipedIKSolverAnalytical(const std::shared_ptr<RobotTree>& tree_ptr)
    : m_robot_tree_ptr(tree_ptr)
{
    if (!tree_ptr) {
        throw std::runtime_error{"Valid RobotTree is required for IK solver"};
    }
}

BipedIKSolverAnalytical::~BipedIKSolverAnalytical()
{

}

void BipedIKSolverAnalytical::cartesianToJoint(const std::string& start_link_name, const Eigen::Vector3f& target_pos,
                                         const Eigen::Quaternionf& target_rot)
{
    auto ankle_link = m_robot_tree_ptr->findLink(start_link_name);
    if (!ankle_link) {
        ROS_ERROR("ankle link not found");
        return;
    }
    
    auto ankle_link_joint = ankle_link->getParentJoint();
    if (!ankle_link_joint) {
        ROS_ERROR("Ankle link has no parent joint. Check your urdf file");
        return;
    }
    
    std::string prefix = ankle_link->getLinkName().substr(0, 2);
    bool left = true;
    if (prefix == "l_") {
        left = true;
    } else if (prefix == "r_") {
        left = false;
    } else {
        ROS_ERROR("Incorrect name format for biped robot! Check your urdf file");
        return;
    }
    
    std::string pelvis_yaw_name = prefix + "pelvis_yaw";
    SE3 base_to_pelvis_yaw = m_robot_tree_ptr->calculateFKRootToJoint(pelvis_yaw_name);
    SE3 base_to_ankle = m_robot_tree_ptr->calculateFKRootToJoint(ankle_link_joint->getJointName());
    
    SE3 pelvis_to_ankle;
    Eigen::Quaternionf inv_rot = base_to_pelvis_yaw.rot.inverse();
    pelvis_to_ankle.rot = inv_rot * base_to_ankle.rot;
    pelvis_to_ankle.pos = inv_rot * base_to_ankle.pos - inv_rot * base_to_pelvis_yaw.pos;
    
    std::cout << "Translation" << '\n';
    std::cout << pelvis_to_ankle.pos << '\n';
    std::cout << "Rotation: in quaternion" << '\n';
    std::cout << pelvis_to_ankle.rot.vec() << '\n';
    std::cout << pelvis_to_ankle.rot.w() << '\n';
}
