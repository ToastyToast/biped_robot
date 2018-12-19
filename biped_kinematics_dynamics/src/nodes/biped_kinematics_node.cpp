#include <ros/ros.h>

#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"
#include "biped_kinematics_dynamics/robot_tree_publisher.h"
#include "biped_kinematics_dynamics/solvers/biped_ik_solver_analytical.h"
#include "biped_kinematics_dynamics/solvers/biped_ik_solver_numerical.h"

using namespace biped_kinematics_dynamics;

void printUntilRoot(const RobotLink::Ptr& link);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "biped_kinematics_node");
    ros::NodeHandle nh("");
    
    ROS_INFO("Starting biped_kinematics_node");

    urdf::Model robot_model;
    if (!robot_model.initParam("robot_description")) {
        ROS_ERROR("Failed to load robot description");
        return -1;
    }
    
    try {
        std::shared_ptr<RobotTree> robot_tree_ptr = std::make_shared<RobotTree>(robot_model);
        std::cout << *robot_tree_ptr << '\n';
    
        auto robot_link = robot_tree_ptr->findLink("r_ankle");
        printUntilRoot(robot_link);
        std::cout << '\n';
    
        robot_link = robot_tree_ptr->findLink("l_ankle");
        printUntilRoot(robot_link);
        std::cout << '\n';
        
        SE3 target_transform;
        target_transform.pos = Eigen::Vector3f(0.3f, -0.5f, -0.8);
        target_transform.rot = Eigen::AngleAxisf(-0.3f, Eigen::Vector3f::UnitX());
        
        ros::Time last_time = ros::Time::now();
        
        RobotTreePublisher treePublisher(robot_tree_ptr);
        while (nh.ok()) {
            treePublisher.update();
            ros::spinOnce();
            
            if ((ros::Time::now() - last_time).toSec() >= 1.0f) {
                std::cout << "========= IK " << '\n';
                BipedIKSolverAnalytical ik_solver(robot_tree_ptr);
                ik_solver.cartesianToJoint("r_ankle", target_transform);
                
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


