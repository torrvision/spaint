/**
 * spaint: RefineWithICPTracker.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "trackers/RefineWithICPTracker.h"

#include <ITMLib/Engine/ITMTrackerFactory.h>
using namespace ITMLib::Engine;

#include "util/CameraPoseConverter.h"
using namespace rigging;

namespace spaint {

//#################### CONSTRUCTORS ####################

RefineWithICPTracker::RefineWithICPTracker(ITMLib::Engine::ITMTracker *baseTracker, const Vector2i& trackedImageSize, const Settings_CPtr& settings,
                                           const LowLevelEngine_CPtr& lowLevelEngine, const Scene_Ptr& scene)
: m_baseTracker(baseTracker), m_icpSucceeded(true)
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
  Matrix4f initialM = trackingState->pose_d->GetM();
  SimpleCamera initialCam = CameraPoseConverter::pose_to_camera(*trackingState->pose_d);

  // Obtain a coarse pose using the base tracker.
  m_baseTracker->TrackCamera(trackingState, view);

  // Record the pose matrix so that it can be restored if ICP fails.
  Matrix4f baseM = trackingState->pose_d->GetM();
  SimpleCamera baseCam = CameraPoseConverter::pose_to_camera(*trackingState->pose_d);

  // Attempt to refine the pose using ICP.
  m_icpTracker->TrackCamera(trackingState, view);

  // Check whether ICP succeeded or not.
  SimpleCamera icpCam = CameraPoseConverter::pose_to_camera(*trackingState->pose_d);
  m_icpSucceeded = poses_are_similar(initialCam, icpCam, 0.1) & poses_are_similar(baseCam, icpCam, 0.1);
  std::cout << m_icpSucceeded << '\n';

  // If ICP failed, restore the pose from the base tracker.
  if(!m_icpSucceeded) trackingState->pose_d->SetM(baseM);
}

void RefineWithICPTracker::UpdateInitialPose(ITMTrackingState *trackingState)
{
  m_baseTracker->UpdateInitialPose(trackingState);
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

double RefineWithICPTracker::angle_between(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
  return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

bool RefineWithICPTracker::poses_are_similar(const SimpleCamera& cam1, const SimpleCamera& cam2, double distanceThreshold)
{
  double pDist = (cam1.p() - cam2.p()).norm();
  double nAngle = angle_between(cam1.n(), cam2.n());
  double uAngle = angle_between(cam1.u(), cam2.u());
  double vAngle = angle_between(cam1.v(), cam2.v());

  std::cout << pDist << ' ' << nAngle << ' ' << uAngle << ' ' << vAngle << '\n';

  // TODO: Set appropriate thresholds.
  return pDist < distanceThreshold;

  //return true;
}

}
