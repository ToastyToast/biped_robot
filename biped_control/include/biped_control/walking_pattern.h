#ifndef BIPED_CONTROL_WALK_PRIMITIVE_H
#define BIPED_CONTROL_WALK_PRIMITIVE_H

#include <vector>

#include "biped_control/inverted_pendulum.h"

namespace biped_control {

class WalkPattern {
public:
    struct FootPosition {
        FootPosition() = default;
        FootPosition(float fx, float fy) : x(fx), y(fy) {}
        
        float x {0.0f};
        float y {0.0f};
    };
    
    struct StepParameters {
        StepParameters() = default;
        StepParameters(float sx, float sy) : x_length(sx), y_length(sy) {}
        
        float x_length {0.0f};
        float y_length {0.0f};
    };
public:
    WalkPattern(float support_time, const WalkPattern::FootPosition& init_foot_pos);
    
    void setCOMParametersFromLIP(const LIP3D& lip);
    void addStep(const StepParameters& step);
    
    void integrate(float dt);
    
    LIP3D getLIPState() const;
    FootPosition getFootPosition() const;
private:
    LIP3D m_lip;
    
    FootPosition m_foot_pos;
    std::vector<StepParameters> m_steps;
    
    float m_curr_time {0.0f}, m_prev_time {0.0f};
    float m_support_dt {0.0f}, m_support_target {0.0f};
    int m_step_number {0};
};

}

#endif //BIPED_CONTROL_WALK_PRIMITIVE_H
