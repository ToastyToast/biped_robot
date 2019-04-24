#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include "biped_control/inverted_pendulum.h"

using namespace biped_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverted_pendulum_test_node");
    ros::NodeHandle nh("");
    
    ros::Publisher x_pos_pub = nh.advertise<std_msgs::Float32>("x_position", 0);
    ros::Publisher x_vel_pub = nh.advertise<std_msgs::Float32>("x_velocity", 0);
    ros::Publisher y_pos_pub = nh.advertise<std_msgs::Float32>("y_position", 0);
    ros::Publisher y_vel_pub = nh.advertise<std_msgs::Float32>("y_velocity", 0);
    ros::Publisher z_pos_pub = nh.advertise<std_msgs::Float32>("z_position", 0);
    
    ros::Publisher lip_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("lip_pose", 0);
    
    ROS_INFO("Starting inverted_pendulum_test_node");
    
    float prev = 0.0f;
    float curr = 0.0f;
    float target_x_pos = 0.5;
    float x_step = 1.0f;
    float step = 0.01f;
    
    LIP3D lip(0.0f, -0.1f, 0.1f, 0.0f);
    lip.setPlaneConstraint(0.0f, 0.0f, 1.0f);
    
    std_msgs::Float32 msg;
    geometry_msgs::PoseStamped pose_msg;
    
    ros::Rate rate(60.0f);
    while (nh.ok()) {
        float dt = fabs(curr - prev);
        
        lip.integrate(dt);
        
        msg.data = lip.getXPosition();
        x_pos_pub.publish(msg);
        
        msg.data = lip.getXVelocity();
        x_vel_pub.publish(msg);
        
        msg.data = lip.getYPosition();
        y_pos_pub.publish(msg);
        
        msg.data = lip.getYVelocity();
        y_vel_pub.publish(msg);
        
        msg.data = lip.getZPosition();
        z_pos_pub.publish(msg);
        
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "lip_frame";
        pose_msg.pose.position.x = lip.getXPosition();
        pose_msg.pose.position.y = lip.getYPosition();
        pose_msg.pose.position.z = lip.getZPosition();
        
        lip_pose_pub.publish(pose_msg);
        
        if (lip.getXPosition() >= target_x_pos) {
            curr = 0.0f;
            prev = 0.0f;
            lip.setXPosition(-lip.getXPosition());
            lip.setYPosition(-lip.getYPosition());
        } else {
            prev = curr;
            curr += step;
        }
        
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

