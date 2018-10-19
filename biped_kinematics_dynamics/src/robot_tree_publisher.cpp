#include "biped_kinematics_dynamics/robot_tree_publisher.h"

using namespace biped_kinematics_dynamics;

RobotTreePublisher::RobotTreePublisher(const std::shared_ptr<RobotTree>& tree_ptr) {
    m_robot_tree = tree_ptr;
    if (!m_robot_tree) {
        throw std::runtime_error{"Valid RobotTree is required"};
    }
    
    ros::NodeHandle nh("");
    
    m_joint_state_sub = nh.subscribe("joint_states", 1, &RobotTreePublisher::callbackJointState, this);
}

RobotTreePublisher::~RobotTreePublisher()
{

}

void RobotTreePublisher::update()
{
    ros::Time current_time = ros::Time::now();
    float diff = (current_time - m_last_published).toSec();
    
    if (diff > (1.0f / m_rate)) {
        publishTransforms();
    }
}

void RobotTreePublisher::callbackJointState(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->name.size() != joint_state_msg->position.size()) {
        ROS_ERROR("Ignored invalid JointState message");
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock_guard(m_tree_mutex);
        
        const auto& joint_map = m_robot_tree->getJointMap();
        for (int state_idx = 0; state_idx < joint_state_msg->name.size(); state_idx++) {
            std::string joint_name = joint_state_msg->name[state_idx];
            float joint_position = joint_state_msg->position[state_idx];
            
            auto joint_itr = joint_map.find(joint_name);
            if (joint_itr == joint_map.end()) {
                ROS_ERROR("Invalid joint name in JointState message");
                continue;
            }
            auto joint_ptr = joint_itr->second;
            
            Eigen::Vector3f joint_axis = joint_ptr->getJointAxis();
            Eigen::Quaternionf joint_quat = joint_ptr->getParentToJointQuat();
            
            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (joint_axis == Eigen::Vector3f::UnitX()) {
                roll = joint_position;
            } else if (joint_axis == Eigen::Vector3f::UnitY()) {
                pitch = joint_position;
            } else if (joint_axis == Eigen::Vector3f::UnitZ()) {
                yaw = joint_position;
            } else {
                ROS_ERROR("Invalid joint axis for joint %s", joint_name.c_str());
                continue;
            }
    
            joint_quat = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            joint_ptr->setParentToJointQuat(joint_quat);
        }
    }
    
    publishTransforms();
}

void RobotTreePublisher::publishTransforms()
{
    std::lock_guard<std::mutex> lock_guard(m_tree_mutex);
    
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
        tf.transform.rotation.y = joint_quat.vec()(1);
        tf.transform.rotation.z = joint_quat.vec()(2);
    
        transforms.push_back(tf);
    }
    
    m_last_published = ros::Time::now();
    m_tf_broadcaster.sendTransform(transforms);
}



