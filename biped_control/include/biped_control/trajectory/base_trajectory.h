#ifndef BIPED_KINEMATICS_DYNAMICS_BASETRAJECTORY_H
#define BIPED_KINEMATICS_DYNAMICS_BASETRAJECTORY_H

namespace biped_control {

class BaseTrajectory {
public:
    virtual float evaluate(float t) = 0;
};

}

#endif //BIPED_KINEMATICS_DYNAMICS_BASETRAJECTORY_H
