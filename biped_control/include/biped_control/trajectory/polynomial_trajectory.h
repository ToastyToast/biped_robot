#ifndef BIPED_KINEMATICS_DYNAMICS_POLYNOMIAL_TRAJECTORY_H
#define BIPED_KINEMATICS_DYNAMICS_POLYNOMIAL_TRAJECTORY_H

#include "biped_control/trajectory/base_trajectory.h"

namespace biped_control {

class PolynomialTrajectory : public BaseTrajectory {
public:
    float evaluate(float t) override;
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_POLYNOMIAL_TRAJECTORY_H
