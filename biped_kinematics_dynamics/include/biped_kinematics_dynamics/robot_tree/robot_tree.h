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
    using LinkMap = std::unordered_map<std::string, RobotLink::Ptr>;
    using JointMap = std::unordered_map<std::string, RobotJoint::Ptr>;
public:
    explicit RobotTree(const std::string& robot_model_description);
    explicit RobotTree(const urdf::Model& urdf_model);
    ~RobotTree();
    
    void addLink(const RobotLink::Ptr& link);
    void addJoint(const RobotJoint::Ptr& joint);
    
    RobotLink::Ptr findLink(const std::string& link_name) const;
    RobotJoint::Ptr findJoint(const std::string& joint_name) const;

    friend std::ostream& operator<<(std::ostream& out, const RobotTree& robot_tree);
private:
    void parseURDFModel(const urdf::Model& urdf_model);
    RobotLink::Ptr parseURDFLink(const urdf::LinkSharedPtr& urdf_link);
private:
    RobotLink::Ptr m_root_link {};
    
    LinkMap m_robot_links;
    JointMap m_robot_joints;
};

std::ostream& operator<<(std::ostream& out, const RobotTree& robot_tree);
std::ostream& operator<<(std::ostream& out, const RobotLink::Ptr& robot_link);

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H
