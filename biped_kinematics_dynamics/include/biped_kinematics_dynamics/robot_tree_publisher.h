#ifndef BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_PUBLISHER_H
#define BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_PUBLISHER_H

#include <mutex>
#include <thread>
#include <unordered_map>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

namespace biped_kinematics_dynamics {

class RobotTreePublisher {
public:
    RobotTreePublisher(const std::shared_ptr<RobotTree>& tree_ptr);
    ~RobotTreePublisher();
    
    void update();
    
    void callbackJointState(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    
    void publishTransforms();
private:
    float m_rate {100.0f};
    
    std::mutex m_tree_mutex;
    std::shared_ptr<RobotTree> m_robot_tree {nullptr};
    
    ros::Time m_last_published;
    
    ros::Subscriber m_joint_state_sub;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_ROBOT_TREE_PUBLISHER_H
