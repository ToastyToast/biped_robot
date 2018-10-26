#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_IK_SOLVER_NUMERICAL_H
#define BIPED_KINEMATICS_DYNAMICS_DOROB_TREE_IK_SOLVER_NUMERICAL_H

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

namespace biped_kinematics_dynamics {

class RobotTreeIKSolverNumerical {
public:
    RobotTreeIKSolverNumerical(const std::shared_ptr<RobotTree>& tree_ptr);
    ~RobotTreeIKSolverNumerical();
    
    void cartesianToJoint(const std::string& start_link_name, const Eigen::Vector3f& target_pos,
                          const Eigen::Quaternionf& target_rot);
private:
    std::shared_ptr<RobotTree> m_robot_tree_ptr {nullptr};
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_IK_SOLVER_NUMERICAL_H
