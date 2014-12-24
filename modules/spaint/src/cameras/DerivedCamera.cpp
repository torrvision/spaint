/**
 * spaint: DerivedCamera.cpp
 */

#include "cameras/DerivedCamera.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

DerivedCamera::DerivedCamera(const Camera_CPtr& baseCamera, const Eigen::Matrix4f& transformation)
: m_baseCamera(baseCamera), m_transformation(transformation)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

Eigen::Vector3f DerivedCamera::n() const
{
  // TODO
  throw 23;
}

Eigen::Vector3f DerivedCamera::p() const
{
  // TODO
  throw 23;
}

Eigen::Vector3f DerivedCamera::u() const
{
  // TODO
  throw 23;
}

Eigen::Vector3f DerivedCamera::v() const
{
  // TODO
  throw 23;
}

}
