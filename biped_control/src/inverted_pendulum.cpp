
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

LIP2D::LIP2D(float x_position, float y_position, float x_velocity, float y_velocity)
    : m_x_lip(x_position, x_velocity), m_y_lip(y_position, y_velocity)
{
}

void LIP2D::integrate(float t) {
    m_x_lip.integrate(t);
    m_y_lip.integrate(t);
    m_z_position = calculateZFromPlaneConstraint();
}

float LIP2D::getXPosition() const {
    return m_x_lip.getPosition();
}

void LIP2D::setXPosition(float x_position) {
    m_x_lip.setPosition(x_position);
}

float LIP2D::getXVelocity() const {
    return m_x_lip.getVelocity();
}

void LIP2D::setXVelocity(float x_velocity) {
    m_x_lip.setVelocity(x_velocity);
}

float LIP2D::getYPosition() const {
    return m_y_lip.getPosition();
}

void LIP2D::setYPosition(float y_position) {
    m_y_lip.setPosition(y_position);
}

float LIP2D::getYVelocity() const {
    return m_y_lip.getVelocity();
}

void LIP2D::setYVelocity(float y_velocity) {
    m_y_lip.setVelocity(y_velocity);
}

float LIP2D::getZPosition() const {
    return m_z_position;
}

float LIP2D::getHeight() const {
    return m_x_lip.getHeight();
}

void LIP2D::setHeight(float height) {
    m_x_lip.setHeight(height);
    m_y_lip.setHeight(height);
}

void LIP2D::setPlaneConstraint(float x_slope, float y_slope, float zc) {
    m_x_slope = x_slope;
    m_y_slope = y_slope;
    setHeight(zc);
}

float LIP2D::getTau() {
    return m_x_lip.getTau();
}

float LIP2D::calculateZFromPlaneConstraint() const {
    float x = m_x_lip.getPosition();
    float y = m_y_lip.getPosition();
    float zc = m_x_lip.getHeight();
    
    return m_x_slope * x + m_y_slope * y + zc;
}
