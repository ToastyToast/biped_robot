
#include <biped_kinematics_dynamics/robot_tree/robot_tree.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

using namespace biped_kinematics_dynamics;

RobotTree::RobotTree(const std::string& robot_model_description)
{
    if (robot_model_description.empty()) {
        throw std::runtime_error{"Robot model description should be a non-empty string"};
    }
}

RobotTree::RobotTree(const urdf::Model& urdf_model)
{
    for (auto link_it = urdf_model.links_.begin(); link_it != urdf_model.links_.end(); link_it++) {
        std::cout << link_it->second->name << '\n';
    }

    for (auto joint_it = urdf_model.joints_.begin(); joint_it != urdf_model.joints_.end(); joint_it++) {
        std::cout << joint_it->second->name << '\n';
    }
}

RobotTree::~RobotTree()
{

}

std::ostream& biped_kinematics_dynamics::operator<<(std::ostream& out, const RobotTree& robot_tree)
{
    out << "robot tree will be printed here";
    return out;
}
