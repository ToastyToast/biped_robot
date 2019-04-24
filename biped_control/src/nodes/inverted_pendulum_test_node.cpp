#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include "biped_control/inverted_pendulum.h"
#include "biped_control/walk_pattern.h"

using namespace biped_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverted_pendulum_test_node");
    ros::NodeHandle nh("");
    
    ros::Rate rate(20.0f);
    
    ros::Publisher x_pos_pub = nh.advertise<std_msgs::Float32>("x_position", 0);
    ros::Publisher x_vel_pub = nh.advertise<std_msgs::Float32>("x_velocity", 0);
    ros::Publisher y_pos_pub = nh.advertise<std_msgs::Float32>("y_position", 0);
    ros::Publisher y_vel_pub = nh.advertise<std_msgs::Float32>("y_velocity", 0);
    ros::Publisher z_pos_pub = nh.advertise<std_msgs::Float32>("z_position", 0);
    
    ros::Publisher lip_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("lip_pose", 0);
    
    ROS_INFO("Starting inverted_pendulum_test_node");
    
    float dt = 0.01f;
    
    LIP3D lip(0.0f, 0.1f, 0.0f, 0.0f);
    lip.setPlaneConstraint(0.0f, 0.0f, 0.8f);
    
    WalkPattern::FootPosition foot_pos(0.0, -0.1f);
    WalkPattern walk_pattern(0.8f, foot_pos);
    walk_pattern.setCOMParametersFromLIP(lip);
    
    walk_pattern.addStep(WalkPattern::StepParameters(0.0f, 0.2f));
    walk_pattern.addStep(WalkPattern::StepParameters(0.3f, 0.2f));
    walk_pattern.addStep(WalkPattern::StepParameters(0.3f, 0.2f));
    walk_pattern.addStep(WalkPattern::StepParameters(0.3f, 0.2f));
    walk_pattern.addStep(WalkPattern::StepParameters(0.0f, 0.2f));
    
    std_msgs::Float32 msg;
    geometry_msgs::PoseStamped pose_msg;
    while (nh.ok()) {
        walk_pattern.integrate(dt);
        
        foot_pos = walk_pattern.getFootPosition();
        lip = walk_pattern.getLIPState();
        
        std::cout << foot_pos.x << ", " << foot_pos.y << " | " << lip.getXPosition() << ", " << lip.getYPosition() << "\n";
        
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
        pose_msg.pose.position.x = lip.getXPosition() + foot_pos.x;
        pose_msg.pose.position.y = lip.getYPosition() + foot_pos.y;
        pose_msg.pose.position.z = lip.getZPosition();
        
        lip_pose_pub.publish(pose_msg);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

