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
    m_root_link = parseURDFLink(urdf_model.root_link_);
}

RobotLink::Ptr RobotTree::parseURDFLink(const urdf::LinkSharedPtr& urdf_link)
{
    auto child_links = urdf_link->child_links;
    auto child_joints = urdf_link->child_joints;
    
    RobotLink::Ptr parent_link = std::make_shared<RobotLink>(urdf_link->name);
    addLink(parent_link);
    
    for (const auto& child_urdf_link : child_links) {
        RobotLink::Ptr child_link = parseURDFLink(child_urdf_link);
        child_link->setParentLink(parent_link);
        
        parent_link->addChildLink(child_link);
    }
    
    for (const auto& child_urdf_joint : child_joints) {
        RobotJoint::Ptr child_joint = std::make_shared<RobotJoint>(
            child_urdf_joint->name,
            child_urdf_joint->parent_link_name,
            child_urdf_joint->child_link_name);
        
        auto to_joint_urdf_trans = child_urdf_joint->parent_to_joint_origin_transform.position;
        auto to_joint_urdf_quat = child_urdf_joint->parent_to_joint_origin_transform.rotation;
        
        Eigen::Vector3f to_joint_trans(
            to_joint_urdf_trans.x,
            to_joint_urdf_trans.y,
            to_joint_urdf_trans.z
        );
        
        Eigen::Quaternionf to_joint_quat(
            to_joint_urdf_quat.w,
            to_joint_urdf_quat.x,
            to_joint_urdf_quat.y,
            to_joint_urdf_quat.z
        );
        
        child_joint->setParentToJointTrans(to_joint_trans);
        child_joint->setParentToJointQuat(to_joint_quat);
        
        parent_link->addChildJoint(child_joint);
        addJoint(child_joint);
    }
    
    return parent_link;
}

void RobotTree::buildTree()
{
    if (!m_root_link) {
        return;
    }
}

std::ostream& biped_kinematics_dynamics::operator<<(std::ostream& out, const RobotTree& robot_tree)
{
    out << "Links: " << robot_tree.m_robot_links.size() << '\n';
    out << "Joints: " << robot_tree.m_robot_joints.size() << '\n';
    out << robot_tree.m_root_link;
    return out;
}

std::ostream& biped_kinematics_dynamics::operator<<(std::ostream& out, const RobotLink::Ptr& robot_link)
{
    
    auto child_links = robot_link->getChildLinks();
    auto child_joints = robot_link->getChildJoints();
    
    out << "[" << robot_link->getLinkName() << "]";
    
    if (!child_joints.empty()) {
        out << "( ";
        for (const auto& child_joint : child_joints) {
            out << child_joint->getJointName() << " ";
        }
        out << ")";
    }
    out << '\n';
    
    for (const auto& child_link : child_links) {
        out << child_link;
    }

    return out;
}