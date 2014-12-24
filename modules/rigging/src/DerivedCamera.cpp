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
  return m_rot * m_baseCamera->n();
}

Eigen::Vector3f DerivedCamera::p() const
{
  return m_baseCamera->p() + m_trans.x() * m_baseCamera->u() + m_trans.y() * m_baseCamera->v() + m_trans.z() * m_baseCamera->n();
}

Eigen::Vector3f DerivedCamera::u() const
{
  return m_rot * m_baseCamera->u();
}

Eigen::Vector3f DerivedCamera::v() const
{
  return m_rot * m_baseCamera->v();
}

}
