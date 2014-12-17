/**
 * spaint: Camera.cpp
 */

#include "cameras/Camera.h"

#include "maths/MathUtil.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

Camera::Camera(const Eigen::Vector3f& position, const Eigen::Vector3f& look, const Eigen::Vector3f& up)
: m_n(look), m_position(position), m_v(up)
{
  m_n.normalize();
  m_v.normalize();

  m_u = m_v.cross(m_n);
  m_u.normalize();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Eigen::Vector3f& Camera::get_n() const
{
  return m_n;
}

const Eigen::Vector3f& Camera::get_position() const
{
  return m_position;
}

const Eigen::Vector3f& Camera::get_u() const
{
  return m_u;
}

const Eigen::Vector3f& Camera::get_v() const
{
  return m_v;
}

Camera& Camera::move_n(float delta)
{
  m_position += delta * m_n;
  return *this;
}

Camera& Camera::move_u(float delta)
{
  m_position += delta * m_u;
  return *this;
}

Camera& Camera::move_v(float delta)
{
  m_position += delta * m_v;
  return *this;
}

Camera& Camera::rotate(const Eigen::Vector3f& axis, float angle)
{
  m_n = MathUtil::rotate_about_axis(m_n, angle, axis);
  m_u = MathUtil::rotate_about_axis(m_u, angle, axis);
  m_v = MathUtil::rotate_about_axis(m_v, angle, axis);
  return *this;
}

}
