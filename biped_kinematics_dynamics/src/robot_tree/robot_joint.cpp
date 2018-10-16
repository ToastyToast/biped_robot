#include "biped_kinematics_dynamics/robot_tree/robot_joint.h"

using namespace biped_kinematics_dynamics;

RobotJoint::RobotJoint(const std::string& joint_name,
    const std::string& parent_link_name,
    const std::string& child_link_name)
    : m_joint_name(joint_name),
    m_parent_link_name(parent_link_name),
    m_child_link_name(child_link_name)
{
}

RobotJoint::~RobotJoint()
{

}


void RobotJoint::setParentToJointTrans(const Eigen::Vector3f& trans)
{
    m_joint_data.parent_to_joint_trans = trans;
}

Eigen::Vector3f RobotJoint::getParentToJointTrans() const
{
    return m_joint_data.parent_to_joint_trans;
}

void RobotJoint::setParentToJointQuat(const Eigen::Quaternionf& quat)
{
    m_joint_data.parent_to_joint_quat = quat;
}

Eigen::Quaternionf RobotJoint::getParentToJointQuat() const
{
    return m_joint_data.parent_to_joint_quat;
}

void RobotJoint::setJointType(const JointType& type)
{
    m_joint_data.type = type;
}

RobotJoint::JointType RobotJoint::getJointType() const
{
    return m_joint_data.type;
}

void RobotJoint::setJointAxis(const Eigen::Vector3f& axis)
{
    m_joint_data.axis = axis;
}

Eigen::Vector3f RobotJoint::getJointAxis() const
{
    return m_joint_data.axis;
}

void RobotJoint::setJointLimits(const JointLimits& limits)
{
    m_joint_data.limits = limits;
}

RobotJoint::JointLimits RobotJoint::getJointLimits() const
{
    return m_joint_data.limits;
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
