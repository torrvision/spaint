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

FiducialMeasurement
FiducialDetector::make_measurement_from_eye_pose(const std::string& fiducialID,
                                                 const boost::optional<ORUtils::SE3Pose>& fiducialPoseEye,
                                                 const boost::optional<ORUtils::SE3Pose>& cameraPoseWorld)
{
  boost::optional<ORUtils::SE3Pose> fiducialPoseWorld;
  if(fiducialPoseEye && cameraPoseWorld) fiducialPoseWorld.reset(fiducialPoseEye->GetM() * cameraPoseWorld->GetM());
  return FiducialMeasurement(fiducialID, fiducialPoseEye, fiducialPoseWorld);
}

FiducialMeasurement
FiducialDetector::make_measurement_from_world_pose(const std::string& fiducialID,
                                                   const boost::optional<ORUtils::SE3Pose>& fiducialPoseWorld,
                                                   const boost::optional<ORUtils::SE3Pose>& cameraPoseWorld)
{
  boost::optional<ORUtils::SE3Pose> fiducialPoseEye;
  if(fiducialPoseWorld && cameraPoseWorld) fiducialPoseEye.reset(fiducialPoseWorld->GetM() * cameraPoseWorld->GetInvM());
  return FiducialMeasurement(fiducialID, fiducialPoseEye, fiducialPoseWorld);
}

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
