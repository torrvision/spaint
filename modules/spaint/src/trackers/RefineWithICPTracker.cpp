/**
 * spaint: RefineWithICPTracker.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "trackers/RefineWithICPTracker.h"

#include <ITMLib/Engine/ITMTrackerFactory.h>
using namespace ITMLib::Engine;

namespace spaint {

//#################### CONSTRUCTORS ####################

RefineWithICPTracker::RefineWithICPTracker(ITMLib::Engine::ITMTracker *baseTracker, const Vector2i& trackedImageSize, const Settings_CPtr& settings,
                                           const LowLevelEngine_CPtr& lowLevelEngine, const Scene_Ptr& scene)
: m_baseTracker(baseTracker), m_icpSucceeded(false)
{
  m_icpTracker.reset(ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().Make(
    trackedImageSize, settings.get(), lowLevelEngine.get(), NULL, scene.get()
  ));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool RefineWithICPTracker::lost_tracking() const
{
  return !m_icpSucceeded;
}

void RefineWithICPTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  // Obtain a coarse pose using the base tracker.
  m_baseTracker->TrackCamera(trackingState, view);

  // Record the pose matrix so that it can be restored if ICP fails.
  Matrix4f baseM = trackingState->pose_d->GetM();

  // Attempt to refine the pose using ICP.
  m_icpTracker->TrackCamera(trackingState, view);

  // Check whether ICP succeeded or not.
  m_icpSucceeded = true; // TEMPORARY

  // If ICP failed, restore the pose from the base tracker.
  if(!m_icpSucceeded) trackingState->pose_d->SetM(baseM);
}

void RefineWithICPTracker::UpdateInitialPose(ITMTrackingState *trackingState)
{
  m_baseTracker->UpdateInitialPose(trackingState);
}

}
