#include <ros/ros.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

using namespace biped_kinematics_dynamics;

void printUntilRoot(const RobotLink::Ptr& link);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh("~");

    urdf::Model robot_model;
    if (!robot_model.initParam("robot_description")) {
        ROS_ERROR("Failed to load robot description");
        return -1;
    }

    RobotTree robot_tree(robot_model);
    std::cout << robot_tree << '\n';
    
    auto robot_link = robot_tree.findLink("r_ankle_roll_link");
    printUntilRoot(robot_link);
    std::cout << '\n';
    
    robot_link = robot_tree.findLink("l_ankle_roll_link");
    printUntilRoot(robot_link);
    std::cout << '\n';

    ROS_INFO("Starting kinematics_node");
    ros::Rate rate(100);
    while (nh.ok()) {
        rate.sleep();
    }

    return 0;
}

void printUntilRoot(const RobotLink::Ptr& link)
{
    auto robot_link = link;
    while (robot_link) {
        std::cout << "=== " << robot_link->getLinkName() << " ===\n";
        auto parent_joint = robot_link->getParentJoint();
        if (parent_joint) {
            std::cout << "Parent joint: " << parent_joint->getJointName() << '\n';
            std::cout << parent_joint->getParentToJointTrans() << '\n';
            auto quat = parent_joint->getParentToJointQuat();
            std::cout << "Quat:\n" << quat.w() << '\n' << quat.vec() << '\n';
        }
        
        auto child_links = robot_link->getChildLinks();
        if (!child_links.empty()) {
            std::cout << "( ";
            for (const auto& child_link : child_links) {
                std::cout << child_link->getLinkName() << " ";
            }
            std::cout << ")\n";
        }
        robot_link = robot_link->getParentLink();
    }
}


