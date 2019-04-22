#ifndef _BIPED_CONTROL_INVERTED_PENDULUM_H_
#define _BIPED_CONTROL_INVERTED_PENDULUM_H_

#include <cmath>

namespace biped_control {

class LIP1D {
public:
    LIP1D() = default;
    LIP1D(float position, float velocity);
    
    void integrate(float t);
    
    float getPosition() const;
    void setPosition(float position);
    
    float getVelocity() const;
    void setVelocity(float velocity);
    
    float getHeight() const;
    void setHeight(float height);
    
    float getTau();
private:
    float m_position {0.0f};
    float m_velocity {0.0f};
    float m_g {9.81f};
    float m_height {1.0f};
};

}

#endif
