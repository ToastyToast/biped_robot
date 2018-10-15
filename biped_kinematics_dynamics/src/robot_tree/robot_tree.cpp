
#include <biped_kinematics_dynamics/robot_tree/robot_tree.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

using namespace biped_kinematics_dynamics;

RobotTree::RobotTree(const std::string& robot_model_description)
{
    if (robot_model_description.empty()) {
        throw std::runtime_error{"Robot model description should be a non-empty string"};
    }
    
    urdf::Model model;
    if (!model.initString(robot_model_description)) {
        throw std::runtime_error{"Can't parse robot model description XML string"};
    }
    
    parseURDFModel(model);
}

RobotTree::RobotTree(const urdf::Model& urdf_model)
{
    
    
    parseURDFModel(urdf_model);
}

RobotTree::~RobotTree()
{

}

void RobotTree::parseURDFModel(const urdf::Model& urdf_model)
{
    for (auto link_it = urdf_model.links_.begin(); link_it != urdf_model.links_.end(); link_it++) {
        auto urdf_link_ptr = link_it->second;
        std::cout << urdf_link_ptr->name << '\n';
    }

    for (auto joint_it = urdf_model.joints_.begin(); joint_it != urdf_model.joints_.end(); joint_it++) {
        auto urdf_joint_ptr = joint_it->second;
        std::cout << urdf_joint_ptr->name << '\n';
    }
}


std::ostream& biped_kinematics_dynamics::operator<<(std::ostream& out, const RobotTree& robot_tree)
{
    out << "robot tree will be printed here";
    return out;
}
