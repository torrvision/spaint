/**
 * rigging: DerivedCamera.cpp
 */

#include "DerivedCamera.h"

namespace rigging {

//#################### CONSTRUCTORS ####################

DerivedCamera::DerivedCamera(const Camera_CPtr& baseCamera, const Eigen::Matrix3f& rot, const Eigen::Vector3f& trans)
: m_baseCamera(baseCamera), m_rot(rot), m_trans(trans)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

Eigen::Vector3f DerivedCamera::n() const
{
  return make_world_space_rotation() * m_baseCamera->n();
}

Eigen::Vector3f DerivedCamera::p() const
{
  return m_baseCamera->p() + m_trans.x() * m_baseCamera->u() + m_trans.y() * m_baseCamera->v() + m_trans.z() * m_baseCamera->n();
}

Eigen::Vector3f DerivedCamera::u() const
{
  return make_world_space_rotation() * m_baseCamera->u();
}

Eigen::Vector3f DerivedCamera::v() const
{
  return make_world_space_rotation() * m_baseCamera->v();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

Eigen::Matrix3f DerivedCamera::make_world_space_rotation() const
{
  Eigen::Vector3f n = m_baseCamera->n(), u = m_baseCamera->u(), v = m_baseCamera->v();

  // Construct a matrix that can transform (free) vectors from camera space into world space. For example, m * (1,0,0)^T = u.
  Eigen::Matrix3f m;
  m(0,0) = u.x(); m(1,0) = v.x(); m(2,0) = n.x();
  m(0,1) = u.y(); m(1,1) = v.y(); m(2,1) = n.y();
  m(0,2) = u.z(); m(1,2) = v.z(); m(2,2) = n.z();

  // Use it to turn our camera-space rotation matrix into a world-space one.
  return m * m_rot * m.inverse();
}

}
