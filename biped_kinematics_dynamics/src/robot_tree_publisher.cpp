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
        throw std::runtime_error{"Invalid RobotTree. No root link"};
    }
    
    std::vector<geometry_msgs::TransformStamped> transforms;
    
    for (const auto& joint_kv : m_robot_tree->getJointMap()) {
        auto robot_joint = joint_kv.second;
        Eigen::Vector3f joint_pos = robot_joint->getParentToJointTrans();
        Eigen::Quaternionf joint_quat = robot_joint->getParentToJointQuat();
        
        std::string parent_link_name = robot_joint->getParentLinkName();
        std::string child_link_name = robot_joint->getChildLinkName();
        
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = parent_link_name;
        tf.child_frame_id = child_link_name;
        tf.transform.translation.x = joint_pos(0);
        tf.transform.translation.y = joint_pos(1);
        tf.transform.translation.z = joint_pos(2);
        //
        tf.transform.rotation.w = joint_quat.w();
        tf.transform.rotation.x = joint_quat.vec()(0);
        tf.transform.rotation.y = joint_quat.vec()(0);
        tf.transform.rotation.z = joint_quat.vec()(0);
        transforms.push_back(tf);
    }
    
    m_tf_broadcaster.sendTransform(transforms);
}



