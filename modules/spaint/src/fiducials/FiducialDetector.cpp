/**
 * spaint: FiducialDetector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "fiducials/FiducialDetector.h"

#include <itmx/util/CameraPoseConverter.h>
using namespace itmx;
using namespace rigging;

namespace spaint {

//#################### PROTECTED STATIC MEMBER FUNCTIONS ####################

boost::optional<ORUtils::SE3Pose> FiducialDetector::make_pose_from_corners(const boost::optional<Vector3f>& v0,
                                                                           const boost::optional<Vector3f>& v1,
                                                                           const boost::optional<Vector3f>& v2)
{
  boost::optional<ORUtils::SE3Pose> pose;

  if(v0 && v1 && v2)
  {
    Vector3f xp = (*v1 - *v0).normalised();
    Vector3f yp = (*v2 - *v0).normalised();
    Vector3f zp = ORUtils::cross(xp, yp);
    yp = ORUtils::cross(zp, xp);

    SimpleCamera cam(
      Eigen::Vector3f(v0->x, v0->y, v0->z),
      Eigen::Vector3f(zp.x, zp.y, zp.z),
      Eigen::Vector3f(-yp.x, -yp.y, -yp.z)
    );

    pose = CameraPoseConverter::camera_to_pose(cam);
  }

  return pose;
}

}
