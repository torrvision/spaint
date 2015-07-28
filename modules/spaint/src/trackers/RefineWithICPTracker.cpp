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
: m_baseTracker(baseTracker)
{
  m_icpTracker.reset(ITMTrackerFactory<SpaintVoxel,ITMVoxelIndex>::Instance().Make(
    trackedImageSize, settings.get(), lowLevelEngine.get(), NULL, scene.get()
  ));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

bool RefineWithICPTracker::lost_tracking() const
{
  // TODO
  throw 23;
}

void RefineWithICPTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  // TODO
  throw 23;
}

void RefineWithICPTracker::UpdateInitialPose(ITMTrackingState *trackingState)
{
  // TODO
  throw 23;
}

}
