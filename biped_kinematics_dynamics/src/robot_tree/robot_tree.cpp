
#include <biped_kinematics_dynamics/robot_tree/robot_tree.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

using namespace biped_kinematics_dynamics;

RobotTree::RobotTree(const std::string& model_name) {

}

RobotTree::RobotTree(const urdf::Model& urdf_model) {
    for (auto link_it = urdf_model.links_.begin(); link_it != urdf_model.links_.end(); link_it++) {
        std::cout << link_it->second->name << '\n';
    }
}

RobotTree::~RobotTree() {

}

std::ostream& biped_kinematics_dynamics::operator<<(std::ostream& out, const RobotTree& robot_tree) {
    out << "robot tree will be printed here";
    return out;
}
