#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H

#include <string>
#include <ostream>
#include <unordered_map>

#include <urdf/model.h>

#include "biped_kinematics_dynamics/robot_tree/robot_link.h"
#include "biped_kinematics_dynamics/robot_tree/robot_joint.h"

namespace biped_kinematics_dynamics {

class RobotTree {
public:
    using LinkMap = std::unordered_map<std::string, RobotJoint::Ptr>;
    using JointMap = std::unordered_map<std::string, RobotLink::Ptr>;
public:
    explicit RobotTree(const std::string& robot_model_description);
    explicit RobotTree(const urdf::Model& urdf_model);
    ~RobotTree();

    friend std::ostream& operator<<(std::ostream& out, const RobotTree& robot_tree);
private:
    void parseURDFModel(const urdf::Model& urdf_model);
private:
    LinkMap m_robot_links {};
    JointMap m_robot_joints {};
};

std::ostream& operator<<(std::ostream& out, const RobotTree& robot_tree);

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H
