#include "biped_kinematics_dynamics/robot_tree/robot_link.h"

using namespace biped_kinematics_dynamics;

RobotLink::RobotLink(const std::string& link_name)
{

}

RobotLink::~RobotLink()
{

}

std::string RobotLink::getLinkName() const
{
    return m_link_name;
}
