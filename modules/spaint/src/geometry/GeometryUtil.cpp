/**
 * spaint: GeometryUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "geometry/GeometryUtil.h"

#include <ITMLib/Utils/ITMMath.h>

#include "geometry/DualQuaternion.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

bool GeometryUtil::poses_are_similar(const ORUtils::SE3Pose& pose1, const ORUtils::SE3Pose& pose2, double rotThreshold, float transThreshold)
{
  Vector3f r1, t1, r2, t2;
  pose1.GetParams(t1, r1);
  pose2.GetParams(t2, r2);

  double rot = DualQuatd::angle_between_rotations(DualQuatd::from_rotation(r1), DualQuatd::from_rotation(r2));
  float trans = length(t1 - t2);

  return rot <= rotThreshold && trans <= transThreshold;
}

}
