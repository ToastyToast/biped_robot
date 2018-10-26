#include "biped_kinematics_dynamics/robot_tree/solvers/robot_tree_ik_solver_analytical.h"

using namespace biped_kinematics_dynamics;


RobotTreeIKSolverAnalytical::RobotTreeIKSolverAnalytical(const std::shared_ptr<RobotTree>& tree_ptr)
    : m_robot_tree_ptr(tree_ptr)
{
    if (!tree_ptr) {
        throw std::runtime_error{"Valid RobotTree is required for IK solver"};
    }
}

RobotTreeIKSolverAnalytical::~RobotTreeIKSolverAnalytical()
{

}

void RobotTreeIKSolverAnalytical::cartesianToJoint(const std::string& start_link_name, const Eigen::Vector3f& target_pos,
                                         const Eigen::Quaternionf& target_rot)
{
    auto ankle_link = m_robot_tree_ptr->findLink(start_link_name);
    
    if (!ankle_link) {
        ROS_ERROR("ankle link not found");
    }
    
    std::cout << "ankle link found" << '\n';
}
