#include "biped_kinematics_dynamics/robot_tree_publisher.h"

using namespace biped_kinematics_dynamics;

RobotTreePublisher::RobotTreePublisher(const std::shared_ptr<RobotTree>& tree_ptr)
{
    m_robot_tree = tree_ptr;
    if (!m_robot_tree) {
        throw std::runtime_error{"Valid RobotTree is required"};
    }
    
    /*
    ros::NodeHandle n("~");
    
    n.param("publish_rate", m_publish_rate, 100.0f);
    std::string joint_state_topic;
    n.param<std::string>("joint_state_topic", joint_state_topic, "joint_states");
    
    m_joint_state_sub = n.subscribe(joint_state_topic, 1, &RobotTreePublisher::callbackJointState, this);
    */
}

RobotTreePublisher::~RobotTreePublisher()
{

}

void RobotTreePublisher::callbackJointState(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{

}

void RobotTreePublisher::publishTransforms()
{
    RobotLink::Ptr root_link = m_robot_tree->getRootLink();
    if (!root_link) {
        throw std::runtime_error{"RobotTree has no root link!"};
    }
    
    Eigen::Vector3f base_pos(0.0f, 0.0f, 0.0f);
    Eigen::Quaternionf base_quat(1.0f, 0.0f, 0.0f, 0.0f);
    std::string root_link_name = root_link->getLinkName();
    
    std::vector<geometry_msgs::TransformStamped> transforms;
    
    auto child_joints = root_link->getChildJoints();
    for (const auto& child_joint : child_joints) {
        Eigen::Vector3f child_pos = child_joint->getParentToJointTrans();
        Eigen::Quaternionf child_quat = child_joint->getParentToJointQuat();
        
        child_pos = child_pos + base_pos;
        child_quat = child_quat * base_quat;
        child_quat.normalize();
        
        std::string child_link_name = child_joint->getChildLinkName();
        
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = root_link_name;
        tf.child_frame_id = child_link_name;
        tf.transform.translation.x = child_pos(0);
        tf.transform.translation.y = child_pos(1);
        tf.transform.translation.z = child_pos(2);
        //
        tf.transform.rotation.w = child_quat.w();
        tf.transform.rotation.x = child_quat.vec()(0);
        tf.transform.rotation.y = child_quat.vec()(0);
        tf.transform.rotation.z = child_quat.vec()(0);
        
        transforms.push_back(tf);
    }
    
    m_tf_broadcaster.sendTransform(transforms);
}



