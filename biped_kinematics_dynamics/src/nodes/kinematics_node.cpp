#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh;

    ROS_INFO("Starting kinematics_node");

    ros::Rate rate(100);
    while (nh.ok()) {
        rate.sleep();
    }

    return 0;
}

