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

void RobotLink::addChildLink(const RobotLink::Ptr& child_link)
{
    auto result = std::find_if(m_child_links.begin(), m_child_links.end(),
        [&](const RobotLink::Ptr& link) {
            return link->getLinkName() != child_link->getLinkName();
        });
    
    if (result != m_child_links.end()) {
        m_child_links.push_back(child_link);
    }
}

RobotLink::Vector RobotLink::getChildLinks() const
{
    return m_child_links;
}
