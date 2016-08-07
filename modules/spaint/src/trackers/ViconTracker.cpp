/**
 * spaint: ViconTracker.cpp
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

namespace spaint {

//#################### CONSTRUCTORS ####################

ViconTracker::ViconTracker(const std::string& host, const std::string& subjectName)
: m_subjectName(subjectName)
{
  // Connect to the Vicon system.
  if(m_vicon.Connect(host).Result != Result::Success || !m_vicon.IsConnected().Connected)
  {
    throw std::runtime_error("Could not connect to the Vicon system");
  }

  // Set up the Vicon client.
  m_vicon.EnableMarkerData();
  m_vicon.EnableSegmentData();
  m_vicon.EnableUnlabeledMarkerData();
  m_vicon.SetAxisMapping(Direction::Right, Direction::Down, Direction::Forward);
  m_vicon.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
}

//#################### DESTRUCTOR ####################

ViconTracker::~ViconTracker()
{
  m_vicon.DisableMarkerData();
  m_vicon.DisableSegmentData();
  m_vicon.DisableUnlabeledMarkerData();
  m_vicon.Disconnect();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool ViconTracker::requiresColourRendering() const
{
  return false;
}

bool ViconTracker::requiresDepthReliability() const
{
  return false;
}

void ViconTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  // If there's no frame currently available, early out.
  if(m_vicon.GetFrame().Result != Result::Success) return;

#define DEBUG_OUTPUT 0

  std::ostream& fs = std::cout;

#if DEBUG_OUTPUT
  // Output the current frame number for debugging purposes.
  fs << "\n#####\n";
  fs << "Frame " << m_vicon.GetFrameNumber().FrameNumber << "\n\n";
#endif

  // Attempt to get the marker positions for the camera subject.
  boost::optional<std::map<std::string,Eigen::Vector3f> > maybeMarkerPositions = try_get_marker_positions(m_subjectName);
  m_lostTracking = !maybeMarkerPositions;
  if(m_lostTracking) return;
  const std::map<std::string,Eigen::Vector3f>& markerPositions = *maybeMarkerPositions;

#if DEBUG_OUTPUT
  // Output the marker positions for debugging purposes.
  for(std::map<std::string,Eigen::Vector3f>::const_iterator it = markerPositions.begin(), iend = markerPositions.end(); it != iend; ++it)
  {
    fs << it->first << ": " << it->second.transpose() << '\n';
  }
  fs << '\n';
#endif

  // Calculate the camera's v axis using the positions of the markers on top of the camera.
  const Eigen::Vector3f& c = MapUtil::lookup(markerPositions, "centre");
  const Eigen::Vector3f& l = MapUtil::lookup(markerPositions, "left");
  const Eigen::Vector3f& r = MapUtil::lookup(markerPositions, "right");
  Eigen::Vector3f v = (r - c).cross(l - c).normalized();

#if DEBUG_OUTPUT
  // Output the v axis for debugging purposes.
  fs << "V: " << v.transpose() << '\n';
#endif

  // Calculate the camera's n axis by projecting a vector from the right marker to the front marker into the plane defined by the markers on top of the camera.
  const Eigen::Vector3f& f = MapUtil::lookup(markerPositions, "front");
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

//#################### PRIVATE MEMBER FUNCTIONS ####################

boost::optional<std::map<std::string,Eigen::Vector3f> > ViconTracker::try_get_marker_positions(const std::string& subjectName) const
{
  std::map<std::string,Eigen::Vector3f> result;

  unsigned int markerCount = m_vicon.GetMarkerCount(subjectName).MarkerCount;
  for(unsigned int i = 0; i < markerCount; ++i)
  {
    // Get the name of the marker and its position in the Vicon coordinate system.
    std::string markerName = m_vicon.GetMarkerName(subjectName, i).MarkerName;
    Output_GetMarkerGlobalTranslation trans = m_vicon.GetMarkerGlobalTranslation(subjectName, markerName);

    // If we can't currently get the position of the marker:
    if(trans.Occluded)
    {
      // If the marker is essential, early out; if not, just skip it.
      if(markerName == "centre" || markerName == "front" || markerName == "left" || markerName == "right") return boost::none;
      else continue;
    }

    // Transform the marker position from the Vicon coordinate system to our one (the Vicon coordinate system is in mm, whereas ours is in metres).
    Eigen::Vector3f pos(
      static_cast<float>(trans.Translation[0] / 1000),
      static_cast<float>(trans.Translation[1] / 1000),
      static_cast<float>(trans.Translation[2] / 1000)
    );

    // Record the position in the map.
    result.insert(std::make_pair(markerName, pos));
  }

  return result;
}

}
