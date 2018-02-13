/**
 * itmx: RemoteTracker.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "trackers/RemoteTracker.h"
using namespace ITMLib;

namespace itmx {

//#################### CONSTRUCTORS ####################

RemoteTracker::RemoteTracker(const MappingServer_Ptr& mappingServer, int clientID)
: m_clientID(clientID), m_mappingServer(mappingServer)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool RemoteTracker::requiresColourRendering() const
{
  return false;
}

bool RemoteTracker::requiresDepthReliability() const
{
  return false;
}

bool RemoteTracker::requiresPointCloudRendering() const
{
  return false;
}

void RemoteTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  m_mappingServer->get_pose(m_clientID, *trackingState->pose_d);
  trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
}

}
