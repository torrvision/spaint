/**
 * spaint: AveragingFiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/AveragingFiducial.h"

#include <ITMLib/Utils/ITMMath.h>

#include "geometry/DualQuaternion.h"
using namespace spaint;

namespace spaint {

//#################### CONSTRUCTORS ####################

AveragingFiducial::AveragingFiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: Fiducial(id, pose)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void AveragingFiducial::integrate_sub(const FiducialMeasurement& measurement)
{
  // Get the rotation and translation components of both the existing fiducial pose and the measurement pose.
  Vector3f r, t, newR, newT;
  m_pose.GetParams(t, r);
  measurement.pose_world()->GetParams(newT, newR);

  // Convert the two poses to dual quaternions.
  DualQuatf q = DualQuatf::from_translation(t) * DualQuatf::from_rotation(r);
  DualQuatf newQ = DualQuatf::from_translation(newT) * DualQuatf::from_rotation(newR);

  // Compute a confidence-weighted average of the two poses using dual quaternion interpolation
  // and set this as the new pose for the fiducial.
  DualQuatf avgQ = DualQuatf::sclerp(newQ, q, m_confidence / (m_confidence + 1.0f));
  m_pose.SetFrom(avgQ.get_translation(), avgQ.get_rotation());
}

}
