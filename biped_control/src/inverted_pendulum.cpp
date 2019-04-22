
#include <biped_control/inverted_pendulum.h>

#include "biped_control/inverted_pendulum.h"

using namespace biped_control;

LIP1D::LIP1D(float position, float velocity)
    : m_position(position), m_velocity(velocity)
{
}

void LIP1D::integrate(float t)
{
    float tau = getTau();
    float csh = coshf(t/tau);
    float snh = sinhf(t/tau);
    
    float m_new_position = m_position * csh + tau * m_velocity * snh;
    float m_new_velocity = (m_position / tau) * snh + m_velocity * csh;
    
    setPosition(m_new_position);
    setVelocity(m_new_velocity);
}

float LIP1D::getPosition() const
{
    return m_position;
}

void LIP1D::setPosition(float position)
{
    m_position = position;
}

float LIP1D::getVelocity() const
{
    return m_velocity;
}

void LIP1D::setVelocity(float velocity)
{
    m_velocity = velocity;
}

float LIP1D::getHeight() const {
    return m_height;
}

void LIP1D::setHeight(float height) {
    m_height = height;
}

float LIP1D::getTau() {
    return sqrtf(m_height/m_g);
}
