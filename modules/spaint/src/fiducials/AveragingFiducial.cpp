/**
 * spaint: AveragingFiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/AveragingFiducial.h"

#include <orx/geometry/GeometryUtil.h>
using namespace orx;

namespace spaint {

//#################### CONSTRUCTORS ####################

AveragingFiducial::AveragingFiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: Fiducial(id, pose)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void AveragingFiducial::integrate_sub(const FiducialMeasurement& measurement)
{
  // Convert the existing fiducial pose and the measurement pose to dual quaternions.
  DualQuatd dq = GeometryUtil::pose_to_dual_quat<double>(m_pose);
  DualQuatd newDQ = GeometryUtil::pose_to_dual_quat<double>(*measurement.pose_world());

  // Compute a confidence-weighted average of the two poses using dual quaternion interpolation
  // and set this as the new pose for the fiducial.
  DualQuatd avgDQ = DualQuatd::sclerp(newDQ, dq, m_confidence / (m_confidence + 1.0f));
  m_pose = GeometryUtil::dual_quat_to_pose(avgDQ);
}

}
