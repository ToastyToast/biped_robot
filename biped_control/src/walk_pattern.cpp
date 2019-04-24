#include <iostream>

#include "biped_control/walk_pattern.h"

using namespace biped_control;

WalkPattern::WalkPattern(float support_time, const WalkPattern::FootPosition& init_foot_pos)
    : m_support_dt(support_time), m_support_target(support_time), m_foot_pos(init_foot_pos)
{
}

void WalkPattern::setCOMParametersFromLIP(const LIP3D& lip) {
    m_lip = lip;
}

void WalkPattern::addStep(const WalkPattern::StepParameters& step) {
    m_steps.push_back(step);
}

void WalkPattern::integrate(float dt) {
    m_prev_time = m_curr_time;
    m_curr_time += dt;

    std::cout << m_curr_time << std::endl;
    
    m_lip.integrate(dt);
    
    if (m_curr_time >= m_support_target && m_step_number < m_steps.size()) {
        StepParameters curr_step = m_steps[m_step_number++];
        
        m_foot_pos.x += curr_step.x_length;
        m_foot_pos.y += pow(-1, m_step_number) * curr_step.y_length;
        
        m_lip.setXPosition(curr_step.x_length/2.0f  - m_foot_pos.x);
        m_lip.setYPosition((pow(-1, m_step_number) * curr_step.y_length) / 2.0f - m_foot_pos.y);
        
        m_support_target = m_curr_time + m_support_dt;
    }
}

LIP3D WalkPattern::getLIPState() const {
    return m_lip;
}

WalkPattern::FootPosition WalkPattern::getFootPosition() const {
    return m_foot_pos;
}
