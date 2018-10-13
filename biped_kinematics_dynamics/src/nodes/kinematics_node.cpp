#include <ros/ros.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

using namespace biped_kinematics_dynamics;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh("~");

    std::string model_name;
    if (!nh.getParam("robot_model", model_name)) {
        ROS_ERROR("URDF model is required!");
        return -1;
    }

    urdf::Model robot_model;
    if (!robot_model.initFile(model_name)) {
        ROS_ERROR("Failed to parse URDF file");
        return -1;
    }

    RobotTree robot_tree(robot_model);
    std::cout << robot_tree << '\n';

    ROS_INFO("Starting kinematics_node");
    ros::Rate rate(100);
    while (nh.ok()) {
        rate.sleep();
    }

    return 0;
}

