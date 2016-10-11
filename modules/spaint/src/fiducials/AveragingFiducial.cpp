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
  // TODO: Comment.
  Vector3f r, t, newR, newT;
  m_pose.GetParams(t, r);
  measurement.pose_world()->GetParams(newT, newR);

  // TODO: Comment.
  DualQuatf q = DualQuatf::from_translation(t) * DualQuatf::from_rotation(r);
  DualQuatf newQ = DualQuatf::from_translation(newT) * DualQuatf::from_rotation(newR);

  // TODO: Calculate the correct interpolation parameter (use the confidence of the fiducial).
  DualQuatf avgQ = DualQuatf::sclerp(newQ, q, m_confidence / (m_confidence + confidence_step()));

  // TODO: Determine avgR and avgT from avgQ.
  Vector3f avgR, avgT;
  // TODO

  m_pose.SetFrom(avgT, avgR);
}

}
