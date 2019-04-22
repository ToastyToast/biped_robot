#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "biped_control/inverted_pendulum.h"

using namespace biped_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inverted_pendulum_test_node");
    ros::NodeHandle nh("");
    
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float32>("position", 0);
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("velocity", 0);
    
    ROS_INFO("Starting inverted_pendulum_test_node");
    
    float prev = 0.0f;
    float curr = 0.0f;
    float target_t = 0.5f;
    float step = 0.01f;
    
    LIP1D lip(0.01f, 0.0f);
    
    std_msgs::Float32 msg;
    ros::Rate rate(60.0f);
    while (nh.ok()) {
        float dt = fabs(curr - prev);
        
        lip.integrate(dt);
    
        std::cout << lip.getPosition() << ", " << lip.getVelocity() << "\n";
        
        msg.data = lip.getPosition();
        pos_pub.publish(msg);
        
        msg.data = lip.getVelocity();
        vel_pub.publish(msg);
        
        if (curr > target_t) {
            curr = 0.0f;
            prev = 0.0f;
            lip.setPosition(-lip.getPosition());
            //lip.setVelocity(0.0f);
        } else {
            prev = curr;
            curr += step;
        }
        
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

