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

ViconTracker::ViconTracker(const ViconInterface_CPtr& vicon, const std::string& sceneID, const std::string& subjectName, TrackingMode trackingMode)
: m_sceneID(sceneID), m_subjectName(subjectName), m_trackingMode(trackingMode), m_vicon(vicon)
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

  /**
   * There are a number of different coordinate spaces involved when tracking using the Vicon:
   *
   * Original Marker (a)
   * Current Marker (b)
   * Original Camera (w) - a.k.a. "world" space
   * Current Camera (c)
   * Vicon (v)
   *
   * To track using the Vicon, we detect markers on top of the Vicon and use them to determine bTv,
   * the transformation from Vicon space to Current Marker space. Having stored the inverse of the
   * initial value of bTv (i.e. aTv^-1 = vTa), we can then compute bTa = bTv . vTa, i.e. the
   * transformation from Original Marker space to Current Marker space. In relative tracking mode,
   * we then use this directly as the InfiniTAM pose.
   *
   * In absolute tracking mode, we use cTw rather than bTa as the InfiniTAM pose. This can be derived
   * as follows, in which we note that wTa = cTb, since the camera and the markers are rigidly attached
   * to each other:
   *
   * cTw
   * = cTb . bTw
   * = wTa . bTw
   * = (wTv . vTa) . (bTa . aTv . vTw)
   * = (wTv . vTa) . bTa . (aTv . vTw)
   * = aTw^-1 . bTa . aTw
   */

#if DEBUG_OUTPUT
  // Output the current frame number for debugging purposes.
  fs << "\n#####\n";
  fs << "Frame " << m_vicon->get_frame_number() << "\n\n";
#endif

  // Step 1: Attempt to get the marker positions for the subject being tracked.
  boost::optional<std::map<std::string,Eigen::Vector3f> > maybeMarkerPositions = m_vicon->try_get_marker_positions(m_subjectName);
  m_lostTracking = !maybeMarkerPositions;
  if(m_lostTracking) return;
  const std::map<std::string,Eigen::Vector3f>& markerPositions = *maybeMarkerPositions;

  // Step 2: Check that all of the essential markers are present. If not, early out.
  const std::string centreMarker = "centre", leftMarker = "left", rightMarker = "right";
  if(!MapUtil::contains(markerPositions, centreMarker) ||
     !MapUtil::contains(markerPositions, leftMarker) ||
     !MapUtil::contains(markerPositions, rightMarker))
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

  // Step 3: Determine the relative transformation from Vicon space to Current Marker space. For simplicity, we do this by determining
  //         the axis vectors for a SimpleCamera representation of the transformation, and then convert this to a matrix.

  // Calculate the SimpleCamera's v axis using the positions of the markers on top of the camera.
  const Eigen::Vector3f& c = MapUtil::lookup(markerPositions, centreMarker);
  const Eigen::Vector3f& l = MapUtil::lookup(markerPositions, leftMarker);
  const Eigen::Vector3f& r = MapUtil::lookup(markerPositions, rightMarker);
  Eigen::Vector3f v = (r - c).cross(l - c).normalized();

#if DEBUG_OUTPUT
  // Output the v axis for debugging purposes.
  fs << "V: " << v.transpose() << '\n';
#endif

  // Calculate a draft version of the SimpleCamera's u axis (the actual u axis will be calculated later as v x n).
  Eigen::Vector3f u = (l - r).normalized();

  // Calculate the SimpleCamera's n axis as u x v (note that the normalisation here is defensive only - n should already be unit length).
  Eigen::Vector3f n = u.cross(v).normalized();

#if DEBUG_OUTPUT
  // Output the n axis for debugging purposes.
  fs << "N: " << n.transpose() << '\n';
#endif

  // Construct the SimpleCamera and convert it to a matrix representation of the relative transformation.
  rigging::SimpleCamera cam(c, n, v);
  Matrix4f bTv = CameraPoseConverter::camera_to_pose(cam).GetM();

#if DEBUG_OUTPUT
  // Output the relative transformation for debugging purposes.
  std::cout << "bTv:\n" << bTv << "\n\n";
#endif

  // Step 4: If we haven't yet recorded the relative transformation from Original Marker space to Vicon space, record it now.
  //         (Note that the first time we get here, Original Marker space is simply Current Marker space.)
  if(!m_vTa) m_vTa = ORUtils::SE3Pose(bTv).GetInvM();

  // Step 5: Compute the relative transformation from Original Marker space to Current Marker space, and use this to update the tracking state.
  Matrix4f bTa = bTv * *m_vTa;

  // Step 6: Depending on which tracking mode we're using, determine the final pose to use for tracking, and update the tracking state accordingly.
  switch(m_trackingMode)
  {
    case TM_ABSOLUTE:
    {
      // If we're in absolute tracking mode, first look to see if the relative transformation from Original Camera
      // ("world") space to Vicon space has been determined yet.
      const boost::optional<Matrix4f>& vTw = m_vicon->get_world_to_vicon_transform(m_sceneID);
      if(vTw)
      {
        // If it has, compute and set the final pose (cTw), and set the tracking quality to good.
        if(!m_aTw)
        {
          m_aTw = ORUtils::SE3Pose(*m_vTa).GetInvM() * *vTw;
          m_wTa = ORUtils::SE3Pose(*m_aTw).GetInvM();
        }

        Matrix4f cTw = *m_wTa * bTa * *m_aTw;
        trackingState->pose_d->SetM(cTw);
        trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
      }
      else
      {
        // If not, set the tracking quality to failed.
        trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;
      }
      break;
    }
    case TM_RELATIVE:
    {
      // If we're in relative tracking mode, use the relative transformation from Original Marker space to Current Marker space
      // as the pose, and set the tracking quality to good.
      trackingState->pose_d->SetM(bTa);
      trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
      break;
    }
  }

#if DEBUG_OUTPUT
  // Output the inverse of the final pose we computed (for debugging purposes).
  fs << "\ninvM:\n";
  fs << std::fixed << std::setprecision(1) << trackingState->pose_d->GetInvM() << '\n';
#endif

  fs.flush();

#undef DEBUG_OUTPUT
}

}
