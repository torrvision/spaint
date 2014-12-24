/**
 * spaint: CompositeCamera.cpp
 */

#include "cameras/CompositeCamera.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

CompositeCamera::CompositeCamera(const MoveableCamera_Ptr& primaryCamera)
: m_primaryCamera(primaryCamera)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CompositeCamera::add_secondary_camera(const std::string& name, const Camera_CPtr& camera)
{
  // TODO
  throw 23;
}

const Camera_CPtr& CompositeCamera::get_secondary_camera(const std::string& name) const
{
  // TODO
  throw 23;
}

CompositeCamera& CompositeCamera::move_n(float delta)
{
  m_primaryCamera->move_n(delta);
  return *this;
}

CompositeCamera& CompositeCamera::move_u(float delta)
{
  m_primaryCamera->move_u(delta);
  return *this;
}

CompositeCamera& CompositeCamera::move_v(float delta)
{
  m_primaryCamera->move_v(delta);
  return *this;
}

Eigen::Vector3f CompositeCamera::n() const
{
  return m_primaryCamera->n();
}

Eigen::Vector3f CompositeCamera::p() const
{
  return m_primaryCamera->p();
}

void CompositeCamera::remove_secondary_camera(const std::string& name)
{
  // TODO
  throw 23;
}

CompositeCamera& CompositeCamera::rotate(const Eigen::Vector3f& axis, float angle)
{
  m_primaryCamera->rotate(axis, angle);
  return *this;
}

Eigen::Vector3f CompositeCamera::u() const
{
  return m_primaryCamera->u();
}

Eigen::Vector3f CompositeCamera::v() const
{
  return m_primaryCamera->v();
}

}
