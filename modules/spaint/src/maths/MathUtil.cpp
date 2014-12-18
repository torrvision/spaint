/**
 * spaint: MathUtil.cpp
 */

#include "maths/MathUtil.h"

#include <cmath>
#include <stdexcept>

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

Eigen::Vector3f MathUtil::rotate_about_axis(const Eigen::Vector3f& v, float angle, const Eigen::Vector3f& axis)
{
  // Enforce the precondition.
  if(fabs(axis.size() - 1.0f) < 1e-3f)
  {
    throw std::runtime_error("Vector too small to be used as a rotation axis");
  }

  float cosAngle = cos(angle);
  float sinAngle = sin(angle);
  Eigen::Vector3f aCROSSv = axis.cross(v);

  // The rotated vector is v cos angle + (axis x v) sin angle + axis(axis . v)(1 - cos angle)
  // See Mathematics for 3D Game Programming and Computer Graphics, p.62, for details of why this is (it's not very hard).
  return v * cosAngle + aCROSSv * sinAngle + axis * axis.dot(v) * (1.0f - cosAngle);
}

}
