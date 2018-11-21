/**
 * spaint: ViconFiducialDetector.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "fiducials/ViconFiducialDetector.h"
using namespace itmx;
using namespace ViconDataStreamSDK::CPP;

#include <orx/geometry/GeometryUtil.h>
using namespace orx;

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### CONSTRUCTORS ####################

ViconFiducialDetector::ViconFiducialDetector(const ViconInterface_CPtr& vicon, const FiducialDetector_CPtr& calibratingDetector)
: m_calibratingDetector(calibratingDetector), m_vicon(vicon)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::map<std::string,FiducialMeasurement>
ViconFiducialDetector::detect_fiducials(const View_CPtr& view, const ORUtils::SE3Pose& depthPose) const
try
{
  /**
   * The primary purpose of the Vicon fiducial detector is to calibrate the Vicon system with respect to world space
   * (i.e. the coordinate space associated with the original pose of the RGB-D camera). For this, we use a printed
   * (square) ArUco fiducial with Vicon markers stuck to three of its corners. The method itself is as follows:
   *
   * 1) Detect the Vicon markers using the Vicon system, and use them to determine a transformation fTv from Vicon
   *    space (v) to another coordinate space (fv) corresponding to the fiducial.
   * 2) Detect the ArUco fiducial itself in the RGB-D image, and use it to determine a transformation fTw from
   *    Original Camera ("world") space (w) to another coordinate space (fc) corresponding to the fiducial.
   * 3) Assert that because of the way in which we stuck the Vicon markers to the ArUco fiducial, fv = fc.
   *    (If not, move the markers.) From now on, call that space f (thereby justifying the transformation names).
   * 4) Finally, compute the relative transformation from world space to Vicon space as vTw = fTv^-1 * fTw.
   *
   * Each call to this function yields a single measurement of this relative transformation. Multiple measurements
   * may later be blended by the fiducial fusion process, depending on the type of fiducial we're using, giving a
   * more stable estimate of the relative transformation between the two coordinate spaces. The final pose of the
   * fiducial is the pose of the Vicon origin in world space (when rendered, the fiducial will be at the origin).
   */

  std::map<std::string,FiducialMeasurement> result;

  // Attempt to detect the Vicon markers and determine the transformation from Vicon space to fiducial space.
  const std::string subjectName = "coords";
  boost::optional<std::map<std::string,Eigen::Vector3f> > maybeMarkerPositions = m_vicon->try_get_marker_positions(subjectName);
  if(!maybeMarkerPositions) return std::map<std::string,FiducialMeasurement>();

  const std::map<std::string,Eigen::Vector3f>& markerPositions = *maybeMarkerPositions;
  const Vector3f v0 = GeometryUtil::to_itm(MapUtil::lookup(markerPositions, "main1"));
  const Vector3f v1 = GeometryUtil::to_itm(MapUtil::lookup(markerPositions, "main2"));
  const Vector3f v2 = GeometryUtil::to_itm(MapUtil::lookup(markerPositions, "main3"));
  const boost::optional<ORUtils::SE3Pose> fTv = make_pose_from_corners(v0, v1, v2);

  if(fTv)
  {
    // Attempt to detect the ArUco fiducial in the RGB-D image and determine the transformation from world space to fiducial space.
    std::map<std::string,FiducialMeasurement> calibrationResults = m_calibratingDetector->detect_fiducials(view, depthPose);
    if(!calibrationResults.empty())
    {
      const FiducialMeasurement& calibrationMeasurement = calibrationResults.begin()->second;
      const boost::optional<ORUtils::SE3Pose>& fTw = calibrationMeasurement.pose_world();
      if(fTw)
      {
        // Compute the relative transformation from world space to Vicon space, and add a measurement of the Vicon origin based on it.
        // Note that fTv^-1 * fTw = vTf * fTw = vTw.
        ORUtils::SE3Pose vTw(fTv->GetInvM() * fTw->GetM());
        FiducialMeasurement measurement = make_measurement_from_world_pose("vicon", vTw, boost::none);
        result.insert(std::make_pair(measurement.id(), measurement));
      }
    }
  }

  return result;
}
catch(std::exception&)
{
  return std::map<std::string,FiducialMeasurement>();
}

}
