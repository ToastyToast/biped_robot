#include <ros/ros.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"
#include "biped_kinematics_dynamics/robot_tree_publisher.h"
#include "biped_kinematics_dynamics/robot_tree/solvers/robot_tree_ik_solver.h"

using namespace biped_kinematics_dynamics;

void printUntilRoot(const RobotLink::Ptr& link);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh("");

    urdf::Model robot_model;
    if (!robot_model.initParam("robot_description")) {
        ROS_ERROR("Failed to load robot description");
        return -1;
    }
    
    try {
        std::shared_ptr<RobotTree> robot_tree = std::make_shared<RobotTree>(robot_model);
        std::cout << robot_tree << '\n';
    
        auto robot_link = robot_tree->findLink("r_ankle");
        printUntilRoot(robot_link);
        std::cout << '\n';
    
        robot_link = robot_tree->findLink("l_ankle");
        printUntilRoot(robot_link);
        std::cout << '\n';
        
        ROS_INFO("Starting kinematics_node");
        
        Eigen::Vector3f target_pos(0.0f, 0.0f, 0.0f);
        Eigen::Quaternionf target_rot(1.0f, 0.0f, 0.0f, 0.0f);
        RobotTreeIKSolver ik_solver(robot_tree);
        ik_solver.cartesianToJoint("r_ankle", target_pos, target_rot);
        
        ros::Time last_time = ros::Time::now();
        
        RobotTreePublisher treePublisher(robot_tree);
        while (nh.ok()) {
            treePublisher.update();
            ros::spinOnce();
            
            if ((ros::Time::now() - last_time).toSec() >= 1.0f) {
                SE3 to_ankle = robot_tree->calculateFKPelvisToJoint("l_ankle_fixed");
                std::cout << "=================" << '\n';
                std::cout << to_ankle.pos << '\n';
                std::cout << '\n';
                std::cout << to_ankle.rot.vec() << '\n';
                std::cout << to_ankle.rot.w() << '\n';
                last_time = ros::Time::now();
            }
            
        }
    } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
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


