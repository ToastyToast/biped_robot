#include <ros/ros.h>

#include "biped_control/trajectory/polynomial_trajectory.h"

using namespace biped_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "biped_kinematics_node");
    ros::NodeHandle nh("");
    
    PolynomialTrajectory traj;

    ROS_INFO("Starting biped_control_test_node");
    ROS_INFO("%f", traj.evaluate(0.0f));
    
    while (nh.ok()) {
        ros::spinOnce();
    }

    return 0;
}

