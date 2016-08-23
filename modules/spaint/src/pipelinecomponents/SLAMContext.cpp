/**
 * spaint: SLAMContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMContext.h"
using namespace ITMLib;
using namespace ORUtils;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& SLAMContext::get_depth_image_size() const
{
  return m_inputRawDepthImage->noDims;
}

const ITMShortImage_Ptr& SLAMContext::get_input_raw_depth_image()
{
  return m_inputRawDepthImage;
}

const ITMUChar4Image_Ptr& SLAMContext::get_input_rgb_image()
{
  return m_inputRGBImage;
}

const ITMIntrinsics& SLAMContext::get_intrinsics() const
{
  return m_view->calib->intrinsics_d;
}

const SLAMContext::RenderState_Ptr& SLAMContext::get_live_render_state()
{
  return m_liveRenderState;
}

const SE3Pose& SLAMContext::get_pose() const
{
  return *m_trackingState->pose_d;
}

const Vector2i& SLAMContext::get_rgb_image_size() const
{
  return m_inputRGBImage->noDims;
}

const SpaintScene_Ptr& SLAMContext::get_scene()
{
  return m_scene;
}

SpaintScene_CPtr SLAMContext::get_scene() const
{
  return m_scene;
}

const SLAMContext::TrackingState_Ptr& SLAMContext::get_tracking_state()
{
  return m_trackingState;
}

SLAMContext::TrackingState_CPtr SLAMContext::get_tracking_state() const
{
  return m_trackingState;
}

const SLAMContext::View_Ptr& SLAMContext::get_view()
{
  return m_view;
}

SLAMContext::View_CPtr SLAMContext::get_view() const
{
  return m_view;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void SLAMContext::set_input_raw_depth_image(ITMShortImage *inputRawDepthImage)
{
  if(m_inputRawDepthImage.get() != inputRawDepthImage) m_inputRawDepthImage.reset(inputRawDepthImage);
}

void SLAMContext::set_input_rgb_image(ITMUChar4Image *inputRGBImage)
{
  if(m_inputRGBImage.get() != inputRGBImage) m_inputRGBImage.reset(inputRGBImage);
}

void SLAMContext::set_live_render_state(ITMLib::ITMRenderState *liveRenderState)
{
  if(m_liveRenderState.get() != liveRenderState) m_liveRenderState.reset(liveRenderState);
}

void SLAMContext::set_scene(SpaintScene *scene)
{
  if(m_scene.get() != scene) m_scene.reset(scene);
}

void SLAMContext::set_tracking_state(ITMLib::ITMTrackingState *trackingState)
{
  if(m_trackingState.get() != trackingState) m_trackingState.reset(trackingState);
}

void SLAMContext::set_view(ITMView *view)
{
  if(m_view.get() != view) m_view.reset(view);
}

}
