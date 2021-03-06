#include "biped_kinematics_dynamics/robot_tree/robot_link.h"

#include <algorithm>

using namespace biped_kinematics_dynamics;

RobotLink::RobotLink(const std::string& link_name)
    : m_link_name(link_name), m_parent_link {nullptr}
{
    if (link_name == "") {
        throw std::runtime_error{"RobotLink should have a name"};
    }
}

RobotLink::~RobotLink()
{

}

std::string RobotLink::getLinkName() const
{
    return m_link_name;
}

void RobotLink::setParentLink(const RobotLink::Ptr& parent_link)
{
    if (parent_link) {
        m_parent_link = parent_link;
    }
}

RobotLink::Ptr RobotLink::getParentLink() const
{
    return m_parent_link;
}


void RobotLink::setParentJoint(const RobotJoint::Ptr& parent_joint)
{
    if (parent_joint) {
        m_parent_joint = parent_joint;
    }
}

RobotJoint::Ptr RobotLink::getParentJoint() const
{
    return m_parent_joint;
}


void RobotLink::addChildLink(const RobotLink::Ptr& child_link)
{
    auto result = std::find_if(m_child_links.begin(), m_child_links.end(),
        [&](const RobotLink::Ptr& link) {
            return link->getLinkName() == child_link->getLinkName();
        });
    
    if (result == m_child_links.end()) {
        m_child_links.push_back(child_link);
    }
}

RobotLink::Vector RobotLink::getChildLinks() const
{
    return m_child_links;
}



void RobotLink::addChildJoint(const RobotJoint::Ptr& child_joint)
{
    auto result = std::find_if(m_child_joints.begin(), m_child_joints.end(),
        [&](const RobotJoint::Ptr& link) {
            return link->getJointName() == child_joint->getJointName();
        });
    
    if (result == m_child_joints.end()) {
        m_child_joints.push_back(child_joint);
    }
}

RobotJoint::Vector RobotLink::getChildJoints() const
{
    return m_child_joints;
}
