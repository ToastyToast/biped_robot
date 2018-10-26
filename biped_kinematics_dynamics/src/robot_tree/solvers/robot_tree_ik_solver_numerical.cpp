#include "biped_kinematics_dynamics/robot_tree/solvers/robot_tree_ik_solver_numerical.h"

using namespace biped_kinematics_dynamics;

RobotTreeIKSolverNumerical::RobotTreeIKSolverNumerical(const std::shared_ptr<RobotTree>& tree_ptr)
    : m_robot_tree_ptr(tree_ptr)
{
    if (!tree_ptr) {
        throw std::runtime_error{"Valid RobotTree is required for IK solver"};
    }
}

RobotTreeIKSolverNumerical::~RobotTreeIKSolverNumerical()
{

}

void RobotTreeIKSolverNumerical::cartesianToJoint(const std::string& start_link_name, const Eigen::Vector3f& target_pos,
                                         const Eigen::Quaternionf& target_rot)
{
    std::cout << "Numerical IK solver" << '\n';
}
