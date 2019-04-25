
#include <biped_control/inverted_pendulum.h>

#include "biped_control/inverted_pendulum.h"

using namespace biped_control;

LIP1D::LIP1D(float position, float velocity)
    : m_position(position), m_velocity(velocity)
{
}

void LIP1D::integrate(float dt)
{
    float tau = getTau();
    float csh = coshf(dt/tau);
    float snh = sinhf(dt/tau);
    
    float pos = m_position - m_origin;
    
    float new_position = pos * csh + tau * m_velocity * snh + m_origin;
    float new_velocity = (pos / tau) * snh + m_velocity * csh;
    
    setPosition(new_position);
    setVelocity(new_velocity);
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

float LIP1D::getOrigin() const {
    return m_origin;
}

void LIP1D::setOrigin(float origin) {
    m_origin = origin;
}

float LIP1D::getTau() {
    return sqrtf(m_height/m_g);
}

LIP3D::LIP3D(float x_position, float y_position, float x_velocity, float y_velocity)
    : m_x_lip(x_position, x_velocity), m_y_lip(y_position, y_velocity)
{
}

void LIP3D::integrate(float dt) {
    m_x_lip.integrate(dt);
    m_y_lip.integrate(dt);
    m_z_position = calculateZFromPlaneConstraint();
}

float LIP3D::getXPosition() const {
    return m_x_lip.getPosition();
}

void LIP3D::setXPosition(float x_position) {
    m_x_lip.setPosition(x_position);
}

float LIP3D::getXVelocity() const {
    return m_x_lip.getVelocity();
}

void LIP3D::setXVelocity(float x_velocity) {
    m_x_lip.setVelocity(x_velocity);
}

float LIP3D::getYPosition() const {
    return m_y_lip.getPosition();
}

void LIP3D::setYPosition(float y_position) {
    m_y_lip.setPosition(y_position);
}

float LIP3D::getYVelocity() const {
    return m_y_lip.getVelocity();
}

void LIP3D::setYVelocity(float y_velocity) {
    m_y_lip.setVelocity(y_velocity);
}

float LIP3D::getZPosition() const {
    return m_z_position;
}

float LIP3D::getHeight() const {
    return m_x_lip.getHeight();
}

void LIP3D::setHeight(float height) {
    m_x_lip.setHeight(height);
    m_y_lip.setHeight(height);
}

float LIP3D::getXOrigin() const {
    return m_x_lip.getOrigin();
}

void LIP3D::setXOrigin(float x_origin) {
    m_x_lip.setOrigin(x_origin);
}

float LIP3D::getYOrigin() const {
    return m_y_lip.getOrigin();
}

void LIP3D::setYOrigin(float y_origin) {
    m_y_lip.setOrigin(y_origin);
}

void LIP3D::setPlaneConstraint(float x_slope, float y_slope, float zc) {
    m_x_slope = x_slope;
    m_y_slope = y_slope;
    setHeight(zc);
}

float LIP3D::getTau() {
    return m_x_lip.getTau();
}

float LIP3D::calculateZFromPlaneConstraint() const {
    float x = m_x_lip.getPosition();
    float y = m_y_lip.getPosition();
    float zc = m_x_lip.getHeight();
    
    return m_x_slope * x + m_y_slope * y + zc;
}
