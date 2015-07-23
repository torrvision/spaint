/**
 * spaint: SpaintModel.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/SpaintModel.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintModel::SpaintModel(const Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const TrackingState_Ptr& trackingState,
                         const Settings_CPtr& settings)
: m_depthImageSize(depthImageSize),
  m_labelManager(new LabelManager(10)),
  m_rgbImageSize(rgbImageSize),
  m_scene(scene),
  m_settings(settings),
  m_trackingState(trackingState)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& SpaintModel::get_depth_image_size() const
{
  return m_depthImageSize;
}

const ITMIntrinsics& SpaintModel::get_intrinsics() const
{
  return m_view->calib->intrinsics_d;
}

const LabelManager_Ptr& SpaintModel::get_label_manager()
{
  return m_labelManager;
}

LabelManager_CPtr SpaintModel::get_label_manager() const
{
  return m_labelManager;
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

const SpaintModel::Settings_CPtr& SpaintModel::get_settings() const
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

void SpaintModel::set_view(ITMView *view)
{
  if(m_view.get() != view) m_view.reset(view);
}

}
