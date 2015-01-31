/**
 * spaint: SpaintModel.cpp
 */

#include "core/SpaintModel.h"

//#include <ITMLib/Engine/ITMTrackerFactory.h>

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintModel::SpaintModel(const ITMLibSettings& settings, const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const TrackingController_CPtr& trackingController)
: m_depthImageSize(depthImageSize), m_rgbImageSize(rgbImageSize), m_settings(settings)
{
  // Set up the scene.
  MemoryDeviceType memoryType = settings.deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
  m_scene.reset(new Scene(&m_settings.sceneParams, settings.useSwapping, memoryType));

  // Set up the initial tracking state.
  m_trackingState.reset(trackingController->BuildTrackingState());
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& SpaintModel::get_depth_image_size() const
{
  return m_depthImageSize;
}

const ITMIntrinsics& SpaintModel::get_intrinsics() const
{
  return m_view->calib->intrinsics_d;
}

const ITMPose& SpaintModel::get_pose() const
{
  return *m_trackingState->pose_d;
}

const Vector2i& SpaintModel::get_rgb_image_size() const
{
  return m_rgbImageSize;
}

const SpaintModel::Scene_Ptr& SpaintModel::get_scene()
{
  return m_scene;
}

SpaintModel::Scene_CPtr SpaintModel::get_scene() const
{
  return m_scene;
}

const ITMLibSettings& SpaintModel::get_settings() const
{
  return m_settings;
}

const SpaintModel::TrackingState_Ptr& SpaintModel::get_tracking_state()
{
  return m_trackingState;
}

SpaintModel::TrackingState_CPtr SpaintModel::get_tracking_state() const
{
  return m_trackingState;
}

const SpaintModel::View_Ptr& SpaintModel::get_view()
{
  return m_view;
}

SpaintModel::View_CPtr SpaintModel::get_view() const
{
  return m_view;
}

}
