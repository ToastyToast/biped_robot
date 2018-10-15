#include "biped_kinematics_dynamics/robot_tree/robot_tree.h"

using namespace biped_kinematics_dynamics;

RobotTree::RobotTree(const std::string& robot_model_description)
{
    if (robot_model_description.empty()) {
        throw std::runtime_error{"Robot model description should be a non-empty string"};
    }
    
    urdf::Model model;
    if (!model.initString(robot_model_description)) {
        throw std::runtime_error{"Can't parse robot model description XML string"};
    }
    
    parseURDFModel(model);
}

RobotTree::RobotTree(const urdf::Model& urdf_model)
{
    parseURDFModel(urdf_model);
}

RobotTree::~RobotTree()
{

}

void RobotTree::addLink(const RobotLink::Ptr& link)
{
    if (link) {
        std::string key = link->getLinkName();
    
        if (m_robot_links.find(key) == m_robot_links.end()) {
            m_robot_links.insert({key, link});
        }
    }
}

void RobotTree::addJoint(const RobotJoint::Ptr& joint)
{
    if (joint) {
        std::string key = joint->getJointName();
    
        if (m_robot_joints.find(key) == m_robot_joints.end()) {
            m_robot_joints.insert({key, joint});
        }
    }
}


RobotLink::Ptr RobotTree::findLink(const std::string& link_name) const
{
    auto itr = m_robot_links.find(link_name);
    if (itr != m_robot_links.end()) {
        return itr->second;
    } else {
        return RobotLink::Ptr {nullptr};
    }
}

RobotJoint::Ptr RobotTree::findJoint(const std::string& joint_name) const
{
    auto itr = m_robot_joints.find(joint_name);
    if (itr != m_robot_joints.end()) {
        return itr->second;
    } else {
        return RobotJoint::Ptr {nullptr};
    }
}

void RobotTree::parseURDFModel(const urdf::Model& urdf_model)
{
    auto urdf_root_link = urdf_model.root_link_;
    
    for (auto link_it = urdf_model.links_.begin(); link_it != urdf_model.links_.end(); link_it++) {
        auto urdf_link = link_it->second;
        
        if (!urdf_link) {
            continue;
        }
        
        RobotLink::Ptr link = std::make_shared<RobotLink>(urdf_link->name);
        addLink(link);
        
        std::cout << link->getLinkName() << '\n';
    }

    for (auto joint_it = urdf_model.joints_.begin(); joint_it != urdf_model.joints_.end(); joint_it++) {
        auto urdf_joint = joint_it->second;
       
        if (!urdf_joint) {
            continue;
        }
        
        RobotJoint::Ptr joint = std::make_shared<RobotJoint>(urdf_joint->name);
        addJoint(joint);
        
        std::cout << joint->getJointName() << '\n';
    }
}


std::ostream& biped_kinematics_dynamics::operator<<(std::ostream& out, const RobotTree& robot_tree)
{
    out << "Links: " << robot_tree.m_robot_links.size() << '\n';
    out << "Joints: " << robot_tree.m_robot_joints.size() << '\n';
    
    return out;
}
