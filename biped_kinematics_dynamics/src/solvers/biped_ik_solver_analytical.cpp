#include "biped_kinematics_dynamics/solvers/biped_ik_solver_analytical.h"

using namespace biped_kinematics_dynamics;


BipedIKSolverAnalytical::BipedIKSolverAnalytical(const std::shared_ptr<RobotTree>& tree_ptr)
    : m_robot_tree_ptr(tree_ptr)
{
    if (!tree_ptr) {
        throw std::runtime_error{"Valid RobotTree is required for IK solver"};
    }
}

BipedIKSolverAnalytical::~BipedIKSolverAnalytical()
{

}

void BipedIKSolverAnalytical::cartesianToJoint(const std::string& target_link_name, const Eigen::Vector3f& target_pos)
{
    auto ankle_link = m_robot_tree_ptr->findLink(target_link_name);
    if (!ankle_link) {
        throw std::runtime_error{"ankle link not found"};
    }
    
    auto ankle_link_joint = ankle_link->getParentJoint();
    if (!ankle_link_joint) {
        throw std::runtime_error{"Ankle link has no parent joint. Check your urdf file"};
    }
    
    std::string prefix = ankle_link->getLinkName().substr(0, 2);
    bool left = true;
    if (prefix == "l_") {
        left = true;
    } else if (prefix == "r_") {
        left = false;
    } else {
        throw std::runtime_error{"Incorrect name format for biped robot! Check your urdf file"};
    }
    
    std::string pelvis_yaw_name = prefix + "pelvis_yaw";
    SE3 base_to_pelvis;
    SE3 base_to_ankle;
    try {
        base_to_pelvis = m_robot_tree_ptr->calculateFKRootToJoint(pelvis_yaw_name);
        base_to_ankle = m_robot_tree_ptr->calculateFKRootToJoint(ankle_link_joint->getJointName());
    } catch (const std::runtime_error& err) {
        std::stringstream ss;
        ss << "IK error: " << err.what();
        throw std::runtime_error{ss.str()};
    }
    
    SE3 pelvis_to_ankle;
    Eigen::Quaternionf inv_rot = base_to_pelvis.rot.inverse();
    pelvis_to_ankle.rot = inv_rot * base_to_ankle.rot;
    pelvis_to_ankle.pos = inv_rot * base_to_ankle.pos - inv_rot * base_to_pelvis.pos;
    
    Eigen::Vector3f r_ankle_to_pelvis = pelvis_to_ankle.rot.inverse() * (-pelvis_to_ankle.pos);
    
    std::cout << "Ankle to pelvis distance" << '\n';
    std::cout << r_ankle_to_pelvis << '\n';
    
    //TODO: get A, B from RobotTree;
    float A = 0.5f, B = 0.5f;
    float C = r_ankle_to_pelvis.norm();
    
    float knee_pitch = M_PI - std::acos((A*A + B*B - C*C) / (2.0f * A * B));
    
    float alpha = std::asin(A*std::sin(M_PI - knee_pitch) / C);
    
    float rx = r_ankle_to_pelvis(0);
    float ry = r_ankle_to_pelvis(1);
    float rz = r_ankle_to_pelvis(2);
    
    float ankle_pitch = -std::atan2(
        rx,
        (rz < 0.0f ? -1.0f : 1.0f)*std::sqrt(ry*ry+rz*rz)
        ) - alpha;
    float ankle_roll = std::atan2(ry, rz);
    
    if (ankle_roll > M_PI / 2.0f) {
        ankle_roll = ankle_roll - M_PI / 2.0f;
    } else if (ankle_roll < -1.0f * M_PI / 2.0f) {
        ankle_roll = ankle_roll + M_PI / 2.0f;
    }
    
    Eigen::AngleAxisf ankle_roll_rot = Eigen::AngleAxisf(
        ankle_roll, Eigen::Vector3f(1, 0, 0)
        );
    Eigen::AngleAxisf ankle_pitch_rot = Eigen::AngleAxisf(
        ankle_pitch, Eigen::Vector3f(0, 1, 0)
        );
    Eigen::AngleAxisf knee_pitch_rot = Eigen::AngleAxisf(
        knee_pitch, Eigen::Vector3f(0, 1, 0)
        );
    
    Eigen::Quaternionf ankle_to_knee_quat = base_to_ankle.rot *
        ankle_roll_rot.inverse() *
        ankle_pitch_rot.inverse() *
        knee_pitch_rot.inverse();
    Eigen::Matrix3f R = ankle_to_knee_quat.toRotationMatrix();
    
    float pelvis_yaw = std::atan2(-R(0, 1), R(1, 1));
    float pelvis_roll = std::atan2(
        R(2,1),
        -R(0, 1)*std::sin(pelvis_yaw) + R(1, 1) * std::cos(pelvis_yaw)
        );
    float pelvis_pitch = std::atan2(-R(2, 0), R(2, 2));
    
    std::cout << "pelvis_yaw: " << pelvis_yaw << '\n';
    std::cout << "pelvis_roll: " << pelvis_roll << '\n';
    std::cout << "pelvis_pitch: " << pelvis_pitch << '\n';
    std::cout << "knee_pitch: " << knee_pitch << '\n';
    std::cout << "ankle_pitch: " << ankle_pitch << '\n';
    std::cout << "ankle_roll: " << ankle_roll << '\n';
}
