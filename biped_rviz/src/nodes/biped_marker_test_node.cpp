#include <ros/ros.h>

#include "biped_rviz/markers/basic_marker.h"

using namespace biped_rviz;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "biped_marker_test_node");
    ros::NodeHandle nh("");
    
    ROS_INFO("Starting biped_marker_test_node");
    
    BasicMarker marker("red_marker", "base_link");
    
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(marker.getName(), 1);
    
    ros::Rate rate(1.0);
    while (ros::ok()) {
        ros::spinOnce();
        
        marker_pub.publish(marker.getROSMsg());
        
        rate.sleep();
    }
    
    return 0;
}
