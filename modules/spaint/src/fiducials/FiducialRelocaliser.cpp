/**
 * spaint: FiducialRelocaliser.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/FiducialRelocaliser.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

boost::optional<ORUtils::SE3Pose> FiducialRelocaliser::estimate_pose(const std::map<std::string,Fiducial_Ptr>& fiducials,
                                                                     const std::map<std::string,FiducialMeasurement>& measurements)
{
  // TODO
  return boost::none;
}

}
