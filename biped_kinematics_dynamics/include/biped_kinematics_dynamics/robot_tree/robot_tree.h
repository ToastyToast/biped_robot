#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H

#include <string>
#include <ostream>

#include <urdf/model.h>

namespace biped_kinematics_dynamics {

class RobotTree {
public:
public:
    explicit RobotTree(const std::string& robot_model_description);
    explicit RobotTree(const urdf::Model& urdf_model);
    ~RobotTree();

    friend std::ostream& operator<<(std::ostream& out, const RobotTree& robot_tree);
private:
};

std::ostream& operator<<(std::ostream& out, const RobotTree& robot_tree);

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_H
