/**
 * spaint: TypeConversions.cpp
 */

#include "util/TypeConversions.h"

namespace spaint {

Vector3f TypeConversions::to_itm(const Eigen::Vector3f& v)
{
  Vector3f itmv;
  itmv.x = v[0];
  itmv.y = v[1];
  itmv.z = v[2];
  return itmv;
}

}

