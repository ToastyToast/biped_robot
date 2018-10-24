#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_IK_SOLVER_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_IK_SOLVER_H

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

namespace biped_kinematics_dynamics {

class RobotTreeIKSolver {
public:
    RobotTreeIKSolver(const std::shared_ptr<RobotTree>& tree_ptr);
    ~RobotTreeIKSolver();
    
    void cartesianToJoint(const std::string& start_link_name, const Eigen::Vector3f& target_pos,
                          const Eigen::Quaternionf& target_rot);
private:
    std::shared_ptr<RobotTree> m_robot_tree_ptr {nullptr};
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_IK_SOLVER_H
