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
  // TODO
  return *this;
}

}
