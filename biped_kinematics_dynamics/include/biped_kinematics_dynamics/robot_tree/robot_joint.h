#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_JOINT_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_JOINT_H

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace biped_kinematics_dynamics {

class RobotJoint {
public:
    using Ptr = std::shared_ptr<RobotJoint>;
    using ConstPtr = std::shared_ptr<const RobotJoint>;
    using WeakPtr = std::weak_ptr<RobotJoint>;
    using ConstWeakPtr = std::weak_ptr<const RobotJoint>;
    
    using Vector = std::vector<RobotJoint::Ptr>;
public:
    enum class JointType {
        REVOLUTE,
        PRISMATIC
    };
    
    struct JointLimits {
        float lower {};
        float upper {};
        float velocity {};
        float effort {};
    };
    
    struct JointData {
        JointType type;
        Eigen::Vector3f axis;
        JointLimits limits;
        
        Eigen::Vector3f parent_to_joint_trans;
        Eigen::Quaternionf parent_to_joint_quat;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
public:
    RobotJoint(const std::string& joint_name,
        const std::string& parent_link_name,
        const std::string& child_link_name);
    ~RobotJoint();
    
    void setParentToJointTrans(const Eigen::Vector3f& trans);
    Eigen::Vector3f getParentToJointTrans() const;
    void setParentToJointQuat(const Eigen::Quaternionf& quat);
    Eigen::Quaternionf getParentToJointQuat() const;

    std::string getJointName() const;
    
    std::string getParentLinkName() const;
    std::string getChildLinkName() const;
private:
    JointData m_joint_data;
    
    std::string m_joint_name;
    
    std::string m_parent_link_name;
    std::string m_child_link_name;
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_JOINT_H
