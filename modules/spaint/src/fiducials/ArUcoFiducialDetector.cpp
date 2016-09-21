/**
 * spaint: ArUcoFiducialDetector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "fiducials/ArUcoFiducialDetector.h"

#include <opencv2/aruco.hpp>

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::map<std::string,Fiducial_Ptr> ArUcoFiducialDetector::detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& pose) const
{
  // TODO
  throw 23;
}

}
