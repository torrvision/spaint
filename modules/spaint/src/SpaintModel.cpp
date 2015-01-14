/**
 * spaint: SpaintModel.cpp
 */

#include "SpaintModel.h"

#include <ITMLib/Engine/ITMTrackerFactory.h>

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintModel::SpaintModel(const ITMLibSettings& settings, const ImageSourceEngine_Ptr& imageSourceEngine)
: m_settings(settings)
{
  // Make sure that we're not trying to run on the GPU if CUDA support isn't enabled.
#ifndef WITH_CUDA
  std::cerr << "[spaint] CUDA support unavailable, reverting to the CPU implementation of InfiniTAM\n";
  m_settings.useGPU = false;
#endif

  // Determine the RGB and depth image sizes.
  Vector2i rgbImageSize = imageSourceEngine->getRGBImageSize(), depthImageSize = imageSourceEngine->getDepthImageSize();
  if(depthImageSize.x == -1 || depthImageSize.y == -1) depthImageSize = rgbImageSize;

  // Set up the scene.
  m_scene.reset(new Scene(&m_settings.sceneParams, m_settings.useSwapping, m_settings.useGPU));

  // Set up the initial tracking state.
  m_trackingState.reset(ITMTrackerFactory::MakeTrackingState(m_settings, rgbImageSize, depthImageSize));
  m_trackingState->pose_d->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // Set up the scene view.
  m_view.reset(new ITMView(imageSourceEngine->calib, rgbImageSize, depthImageSize, m_settings.useGPU));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const ITMIntrinsics& SpaintModel::get_intrinsics() const
{
  return m_view->calib->intrinsics_d;
}

const ITMPose& SpaintModel::get_pose() const
{
  return *m_trackingState->pose_d;
}

SpaintModel::Scene_CPtr SpaintModel::get_scene() const
{
  return m_scene;
}

SpaintModel::TrackingState_CPtr SpaintModel::get_tracking_state() const
{
  return m_trackingState;
}

SpaintModel::View_CPtr SpaintModel::get_view() const
{
  return m_view;
}

}
