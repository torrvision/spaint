/**
 * spaint: AveragingFiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/AveragingFiducial.h"

#include <ITMLib/Utils/ITMMath.h>

#include <ORUtils/DualQuaternion.h>
using ORUtils::DualQuatf;

namespace spaint {

//#################### CONSTRUCTORS ####################

AveragingFiducial::AveragingFiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: Fiducial(id, pose)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void AveragingFiducial::integrate_sub(const FiducialMeasurement& measurement)
{
  Vector3f R, t, newR, newT;
  m_pose.GetParams(t, R);
  measurement.pose_world()->GetParams(newT, newR);

  // TODO: Rotations.
  DualQuatf q = DualQuatf::from_translation(t);
  DualQuatf newQ = DualQuatf::from_translation(newT);

  // TODO: Calculate the correct interpolation parameter (use the confidence of the fiducial).
  DualQuatf avgQ = DualQuatf::sclerp(q, newQ, 0.5f);

  // TODO: Determine avgR and avgT from avgQ.
  Vector3f avgR, avgT;
  // TODO

  m_pose.SetFrom(avgT, avgR);
}

}
