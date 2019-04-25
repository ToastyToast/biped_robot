#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include "biped_control/inverted_pendulum.h"
#include "biped_control/walking_pattern.h"

using namespace biped_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverted_pendulum_test_node");
    ros::NodeHandle nh("");
    
    ros::Rate rate(100.0f);
    
    ros::Publisher x_pos_pub = nh.advertise<std_msgs::Float32>("position", 0);
    ros::Publisher x_vel_pub = nh.advertise<std_msgs::Float32>("velocity", 0);

    ros::Publisher lip_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("lip_pose", 0);
    
    ROS_INFO("Starting inverted_pendulum_test_node");
    
    float dt = 0.01f;
    float target_pos = 0.4;
    float step_size = 0.5f;
    
    LIP1D lip(0.01f, 0.25f);
    lip.setHeight(0.8f);
    
    std_msgs::Float32 msg;
    geometry_msgs::PoseStamped pose_msg;
    while (nh.ok()) {
        lip.integrate(dt);
        
        if (lip.getPosition() >= target_pos) {
            std::cout << "STEP\n";
            lip.setOrigin(lip.getOrigin() + step_size);
            target_pos = lip.getOrigin() + step_size/2.0;
        }
        
        std::cout << lip.getPosition() << " -> " << target_pos << " | " << ", " << lip.getVelocity() << "\n";
        
        msg.data = lip.getPosition();
        x_pos_pub.publish(msg);
        
        msg.data = lip.getVelocity();
        x_vel_pub.publish(msg);
        
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "lip_frame";
        pose_msg.pose.position.x = lip.getPosition();
        pose_msg.pose.position.y = 0;
        pose_msg.pose.position.z = lip.getHeight();
        
        lip_pose_pub.publish(pose_msg);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

