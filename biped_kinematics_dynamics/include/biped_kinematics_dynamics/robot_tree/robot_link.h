#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_LINK_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_LINK_H

#include <string>
#include <memory>

namespace biped_kinematics_dynamics {

class RobotLink {
public:
    using Ptr = std::shared_ptr<RobotLink>;
    using ConstPtr = std::shared_ptr<const RobotLink>;
    using WeakPtr = std::weak_ptr<RobotLink>;
    using ConstWeakPtr = std::weak_ptr<const RobotLink>;
public:
    RobotLink(const std::string& link_name);
    ~RobotLink();

    std::string getLinkName() const;
private:
    std::string m_link_name {};
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_LINK_H