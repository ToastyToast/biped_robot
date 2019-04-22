#include <ros/ros.h>

#include "biped_control/limp.h"

using namespace biped_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "limp_test_node");
    ros::NodeHandle nh("");
    
    ROS_INFO("Starting limp_test_node");
    
    while (nh.ok()) {
        ros::spinOnce();
    }

    return 0;
}

