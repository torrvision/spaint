/**
 * itmx: ViconTracker.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "trackers/ViconTracker.h"
using namespace ITMLib;
using namespace ViconDataStreamSDK::CPP;

#include <fstream>
#include <iomanip>
#include <iostream>

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

#include "util/CameraPoseConverter.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

ViconTracker::ViconTracker(const ViconInterface_CPtr& vicon, const std::string& subjectName)
: m_subjectName(subjectName), m_vicon(vicon)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool ViconTracker::requiresColourRendering() const
{
  return false;
}

bool ViconTracker::requiresDepthReliability() const
{
  return false;
}

bool ViconTracker::requiresPointCloudRendering() const
{
  return false;
}

void ViconTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
#define DEBUG_OUTPUT 0

  std::ostream& fs = std::cout;

#if DEBUG_OUTPUT
  // Output the current frame number for debugging purposes.
  fs << "\n#####\n";
  fs << "Frame " << m_vicon->get_frame_number() << "\n\n";
#endif

  // Attempt to get the marker positions for the camera subject.
  boost::optional<std::map<std::string,Eigen::Vector3f> > maybeMarkerPositions = m_vicon->try_get_marker_positions(m_subjectName);
  m_lostTracking = !maybeMarkerPositions;
  if(m_lostTracking) return;
  const std::map<std::string,Eigen::Vector3f>& markerPositions = *maybeMarkerPositions;

  // Check that all of the essential markers are present. If not, early out.
  const std::string centreMarker = "centre", frontMarker = "front", leftMarker = "left", rightMarker = "right";
  if(!MapUtil::contains(markerPositions, centreMarker) || !MapUtil::contains(markerPositions, frontMarker) ||
     !MapUtil::contains(markerPositions, leftMarker) || !MapUtil::contains(markerPositions, rightMarker))
  {
    return;
  }

#if DEBUG_OUTPUT
  // Output the marker positions for debugging purposes.
  for(std::map<std::string,Eigen::Vector3f>::const_iterator it = markerPositions.begin(), iend = markerPositions.end(); it != iend; ++it)
  {
    fs << it->first << ": " << it->second.transpose() << '\n';
  }
  fs << '\n';
#endif

  // Calculate the camera's v axis using the positions of the markers on top of the camera.
  const Eigen::Vector3f& c = MapUtil::lookup(markerPositions, centreMarker);
  const Eigen::Vector3f& l = MapUtil::lookup(markerPositions, leftMarker);
  const Eigen::Vector3f& r = MapUtil::lookup(markerPositions, rightMarker);
  Eigen::Vector3f v = (r - c).cross(l - c).normalized();

#if DEBUG_OUTPUT
  // Output the v axis for debugging purposes.
  fs << "V: " << v.transpose() << '\n';
#endif

  // Calculate the camera's n axis by projecting a vector from the right marker to the front marker into the plane defined by the markers on top of the camera.
  const Eigen::Vector3f& f = MapUtil::lookup(markerPositions, frontMarker);
  Eigen::Vector3f n = f - r;
  n = (n - ((n.dot(v)) * v)).normalized();

#if DEBUG_OUTPUT
  // Output the n axis for debugging purposes.
  fs << "N: " << n.transpose() << '\n';
#endif

  // Create the camera and determine its pose.
  rigging::SimpleCamera cam(c, n, v);
  Matrix4f globalPose = CameraPoseConverter::camera_to_pose(cam).GetM();

  // If this is the first pass, record the inverse of the camera's initial pose.
  static bool firstPass = true;
  static Matrix4f invInitialPose;
  if(firstPass)
  {
    globalPose.inv(invInitialPose);
    firstPass = false;
  }

  // Compute the camera's InfiniTAM pose (this is the relative transformation between the camera's initial pose and its current pose).
  Matrix4f M = globalPose * invInitialPose;
  trackingState->pose_d->SetM(M);

#if DEBUG_OUTPUT
  // Output the pose for debugging purposes.
  fs << "\nM:\n";
  fs << std::fixed << std::setprecision(1) << M << '\n';
#endif

  fs.flush();

#undef DEBUG_OUTPUT
}

}
