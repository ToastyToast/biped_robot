#ifndef BIPED_KINEMATICS_DYNAMICS_LIE_GROUPS_H
#define BIPED_KINEMATICS_DYNAMICS_LIE_GROUPS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace biped_kinematics_dynamics {

struct SO3 {
    Eigen::Quaternionf rot{1.0f, 0.0f, 0.0f, 0.0f};
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SE3 {
    Eigen::Vector3f pos{0.0f, 0.0f, 0.0f};
    Eigen::Quaternionf rot{1.0f, 0.0f, 0.0f, 0.0f};
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_LIE_GROUPS_H
