/**
 * spaint: SimpleFiducial.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/SimpleFiducial.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SimpleFiducial::SimpleFiducial(const std::string& id, const ORUtils::SE3Pose& pose)
: Fiducial(id, pose)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SimpleFiducial::integrate_sub(const FiducialMeasurement& measurement)
{
  m_pose = *measurement.pose_world();
}

}
