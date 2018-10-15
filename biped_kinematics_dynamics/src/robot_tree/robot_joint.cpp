#include "biped_kinematics_dynamics/robot_tree/robot_joint.h"

using namespace biped_kinematics_dynamics;

RobotJoint::RobotJoint(const std::string& joint_name)
{

}

RobotJoint::~RobotJoint()
{

}

std::string RobotJoint::getJointName() const
{
    return m_joint_name;
}

std::string RobotJoint::getParentLinkName() const
{
    return m_parent_link_name;
}

std::string RobotJoint::getChildLinkName() const
{
    return m_child_link_name;
}
