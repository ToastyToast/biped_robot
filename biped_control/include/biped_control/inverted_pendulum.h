#ifndef _BIPED_CONTROL_INVERTED_PENDULUM_H_
#define _BIPED_CONTROL_INVERTED_PENDULUM_H_

#include <cmath>

namespace biped_control {

class LIP1D {
public:
    LIP1D() = default;
    LIP1D(float position, float velocity);
    
    void integrate(float dt);
    
    float getPosition() const;
    void setPosition(float position);
    
    float getVelocity() const;
    void setVelocity(float velocity);
    
    float getHeight() const;
    void setHeight(float height);
    
    float getOrigin() const;
    void setOrigin(float origin);
    
    float getTau();
private:
    float m_position {0.0f};
    float m_velocity {0.0f};
    float m_origin {0.0f};
    float m_g {9.81f};
    float m_height {1.0f};
};

class LIP3D {
public:
    LIP3D() = default;
    LIP3D(float x_position, float y_position, float x_velocity=0.0f, float y_velocity=0.0f);
    
    void integrate(float dt);
    
    float getXPosition() const;
    void setXPosition(float x_position);
    
    float getXVelocity() const;
    void setXVelocity(float x_velocity);
    
    float getYPosition() const;
    void setYPosition(float y_position);
    
    float getYVelocity() const;
    void setYVelocity(float y_velocity);
    
    float getZPosition() const;
    
    float getHeight() const;
    void setHeight(float height);
    
    float getXOrigin() const;
    void setXOrigin(float x_origin);
    
    float getYOrigin() const;
    void setYOrigin(float y_origin);
    
    void setPlaneConstraint(float x_slope, float y_slope, float zc);
    
    float getTau();
private:
    float calculateZFromPlaneConstraint() const;
    
    LIP1D m_x_lip, m_y_lip;
    float m_x_slope {0.0f}, m_y_slope {0.0f};
    float m_z_position {m_x_lip.getHeight()};
};

}

#endif
