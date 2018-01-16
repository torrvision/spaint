/**
 * itmx: ZedTracker.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "trackers/ZedTracker.h"
using namespace ITMLib;

namespace itmx {

//#################### CONSTRUCTORS ####################

ZedTracker::ZedTracker(const ZedCamera_Ptr& camera)
: m_camera(camera)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool ZedTracker::requiresColourRendering() const
{
  return false;
}

bool ZedTracker::requiresDepthReliability() const
{
  return false;
}

bool ZedTracker::requiresPointCloudRendering() const
{
  return false;
}

void ZedTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  m_camera->get_tracking_state(trackingState);
}

}
