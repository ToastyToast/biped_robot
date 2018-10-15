#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_JOINT_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_JOINT_H

#include <string>
#include <memory>

namespace biped_kinematics_dynamics {

class RobotJoint {
public:
    using Ptr = std::shared_ptr<RobotJoint>;
    using ConstPtr = std::shared_ptr<const RobotJoint>;
    using WeakPtr = std::weak_ptr<RobotJoint>;
    using ConstWeakPtr = std::weak_ptr<const RobotJoint>;
public:
    RobotJoint(const std::string& joint_name);
    ~RobotJoint();

    std::string getJointName() const;
    
    std::string getParentLinkName() const;
    std::string getChildLinkName() const;
private:
    std::string m_joint_name {};
    
    std::string m_parent_link_name {};
    std::string m_child_link_name {};
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_JOINT_H
