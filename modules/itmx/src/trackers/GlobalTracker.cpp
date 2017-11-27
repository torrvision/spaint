/**
 * itmx: GlobalTracker.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "trackers/GlobalTracker.h"
using namespace ITMLib;
using namespace ORUtils;

namespace itmx {

//#################### CONSTRUCTORS ####################

GlobalTracker::GlobalTracker(const Tracker_Ptr& tracker, const SE3Pose& initialPose)
: m_initialPose(initialPose), m_tracker(tracker)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool GlobalTracker::CanKeepTracking() const
{
  return m_tracker->CanKeepTracking();
}

bool GlobalTracker::requiresColourRendering() const
{
  return m_tracker->requiresColourRendering();
}

bool GlobalTracker::requiresDepthReliability() const
{
  return m_tracker->requiresDepthReliability();
}

bool GlobalTracker::requiresPointCloudRendering() const
{
  return m_tracker->requiresPointCloudRendering();
}

void GlobalTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  m_tracker->TrackCamera(trackingState, view);

  trackingState->pose_d->SetM(trackingState->pose_d->GetM() * m_initialPose.GetM());
  trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
}

void GlobalTracker::UpdateInitialPose(ITMTrackingState *trackingState)
{
  trackingState->pose_d->SetM(m_initialPose.GetM());
  trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;
}

}
