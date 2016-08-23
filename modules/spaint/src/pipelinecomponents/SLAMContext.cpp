/**
 * spaint: SLAMContext.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "pipelinecomponents/SLAMContext.h"
using namespace ITMLib;
using namespace ORUtils;

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& SLAMContext::get_depth_image_size(const std::string& sceneID) const
{
  return get_scene_context(sceneID).m_inputRawDepthImage->noDims;
}

const ITMShortImage_Ptr& SLAMContext::get_input_raw_depth_image(const std::string& sceneID)
{
  return get_scene_context(sceneID).m_inputRawDepthImage;
}

const ITMUChar4Image_Ptr& SLAMContext::get_input_rgb_image(const std::string& sceneID)
{
  return get_scene_context(sceneID).m_inputRGBImage;
}

const ITMIntrinsics& SLAMContext::get_intrinsics(const std::string& sceneID) const
{
  return get_scene_context(sceneID).m_view->calib->intrinsics_d;
}

const SLAMContext::RenderState_Ptr& SLAMContext::get_live_render_state(const std::string& sceneID)
{
  return get_scene_context(sceneID).m_liveRenderState;
}

const SE3Pose& SLAMContext::get_pose(const std::string& sceneID) const
{
  return *get_scene_context(sceneID).m_trackingState->pose_d;
}

const Vector2i& SLAMContext::get_rgb_image_size(const std::string& sceneID) const
{
  return get_scene_context(sceneID).m_inputRGBImage->noDims;
}

const SpaintScene_Ptr& SLAMContext::get_scene(const std::string& sceneID)
{
  return get_scene_context(sceneID).m_scene;
}

SpaintScene_CPtr SLAMContext::get_scene(const std::string& sceneID) const
{
  return get_scene_context(sceneID).m_scene;
}

const SLAMContext::TrackingState_Ptr& SLAMContext::get_tracking_state(const std::string& sceneID)
{
  return get_scene_context(sceneID).m_trackingState;
}

SLAMContext::TrackingState_CPtr SLAMContext::get_tracking_state(const std::string& sceneID) const
{
  return get_scene_context(sceneID).m_trackingState;
}

const SLAMContext::View_Ptr& SLAMContext::get_view(const std::string& sceneID)
{
  return get_scene_context(sceneID).m_view;
}

SLAMContext::View_CPtr SLAMContext::get_view(const std::string& sceneID) const
{
  return get_scene_context(sceneID).m_view;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

const SLAMContext::SceneContext& SLAMContext::get_scene_context(const std::string& sceneID) const
{
  return MapUtil::lookup(m_sceneContexts, sceneID);
}

void SLAMContext::set_input_raw_depth_image(const std::string& sceneID, ITMShortImage *inputRawDepthImage)
{
  set_if_different(m_sceneContexts[sceneID].m_inputRawDepthImage, inputRawDepthImage);
}

void SLAMContext::set_input_rgb_image(const std::string& sceneID, ITMUChar4Image *inputRGBImage)
{
  set_if_different(m_sceneContexts[sceneID].m_inputRGBImage, inputRGBImage);
}

void SLAMContext::set_live_render_state(const std::string& sceneID, ITMLib::ITMRenderState *liveRenderState)
{
  set_if_different(m_sceneContexts[sceneID].m_liveRenderState, liveRenderState);
}

void SLAMContext::set_scene(const std::string& sceneID, SpaintScene *scene)
{
  set_if_different(m_sceneContexts[sceneID].m_scene, scene);
}

void SLAMContext::set_tracking_state(const std::string& sceneID, ITMLib::ITMTrackingState *trackingState)
{
  set_if_different(m_sceneContexts[sceneID].m_trackingState, trackingState);
}

void SLAMContext::set_view(const std::string& sceneID, ITMView *view)
{
  set_if_different(m_sceneContexts[sceneID].m_view, view);
}

}
