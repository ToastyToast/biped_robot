#include "biped_kinematics_dynamics/solvers/biped_ik_solver_numerical.h"

using namespace biped_kinematics_dynamics;

BipedIKSolverNumerical::BipedIKSolverNumerical(const std::shared_ptr<RobotTree>& tree_ptr)
    : m_robot_tree_ptr(tree_ptr)
{
    if (!tree_ptr) {
        throw std::runtime_error{"Valid RobotTree is required for IK solver"};
    }
}

BipedIKSolverNumerical::~BipedIKSolverNumerical()
{

}

void BipedIKSolverNumerical::cartesianToJoint(const std::string& target_link_name, const SE3& target_transform)
{
    std::cout << "Numerical IK solver" << '\n';
}
