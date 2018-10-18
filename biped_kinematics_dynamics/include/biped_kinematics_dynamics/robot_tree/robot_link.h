#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_LINK_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_LINK_H

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "biped_kinematics_dynamics/robot_tree/robot_joint.h"

namespace biped_kinematics_dynamics {

class RobotLink {
public:
    using Ptr = std::shared_ptr<RobotLink>;
    using ConstPtr = std::shared_ptr<const RobotLink>;
    using WeakPtr = std::weak_ptr<RobotLink>;
    using ConstWeakPtr = std::weak_ptr<const RobotLink>;
    
    using Vector = std::vector<RobotLink::Ptr>;
public:
    RobotLink(const std::string& link_name);
    ~RobotLink();
    
    std::string getLinkName() const;
    
    void setParentLink(const RobotLink::Ptr& parent_link);
    RobotLink::Ptr getParentLink() const;
    
    void setParentJoint(const RobotJoint::Ptr& parent_joint);
    RobotJoint::Ptr getParentJoint() const;
    
    void addChildLink(const RobotLink::Ptr& child_link);
    RobotLink::Vector getChildLinks() const;
    
    void addChildJoint(const RobotJoint::Ptr& child_joint);
    RobotJoint::Vector getChildJoints() const;
private:
    RobotLink::Ptr m_parent_link {nullptr};
    RobotJoint::Ptr m_parent_joint {nullptr};
    RobotLink::Vector m_child_links;
    RobotJoint::Vector m_child_joints;
    
    std::string m_link_name;
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_LINK_H