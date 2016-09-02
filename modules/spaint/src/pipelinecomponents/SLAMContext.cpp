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
  return get_single_scene_context(sceneID).m_inputRawDepthImage->noDims;
}

const ITMShortImage_Ptr& SLAMContext::get_input_raw_depth_image(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_inputRawDepthImage;
}

ITMShortImage_Ptr SLAMContext::get_input_raw_depth_image_copy(const std::string& sceneID) const
{
  ITMShortImage_CPtr inputRawDepthImage = get_single_scene_context(sceneID).m_inputRawDepthImage;
  ITMShortImage_Ptr copy(new ITMShortImage(inputRawDepthImage->noDims, true, false));
  copy->SetFrom(inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

const ITMUChar4Image_Ptr& SLAMContext::get_input_rgb_image(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_inputRGBImage;
}

ITMUChar4Image_Ptr SLAMContext::get_input_rgb_image_copy(const std::string& sceneID) const
{
  ITMUChar4Image_CPtr inputRGBImage = get_single_scene_context(sceneID).m_inputRGBImage;
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(inputRGBImage->noDims, true, false));
  copy->SetFrom(inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

const ITMIntrinsics& SLAMContext::get_intrinsics(const std::string& sceneID) const
{
  return get_single_scene_context(sceneID).m_view->calib->intrinsics_d;
}

const SurfelRenderState_Ptr& SLAMContext::get_live_surfel_render_state(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_liveSurfelRenderState;
}

const VoxelRenderState_Ptr& SLAMContext::get_live_voxel_render_state(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_liveVoxelRenderState;
}

const SE3Pose& SLAMContext::get_pose(const std::string& sceneID) const
{
  return *get_single_scene_context(sceneID).m_trackingState->pose_d;
}

const Vector2i& SLAMContext::get_rgb_image_size(const std::string& sceneID) const
{
  return get_single_scene_context(sceneID).m_inputRGBImage->noDims;
}

const SpaintSurfelScene_Ptr& SLAMContext::get_surfel_scene(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_surfelScene;
}

SpaintSurfelScene_CPtr SLAMContext::get_surfel_scene(const std::string& sceneID) const
{
  return get_single_scene_context(sceneID).m_surfelScene;
}

const TrackingState_Ptr& SLAMContext::get_tracking_state(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_trackingState;
}

TrackingState_CPtr SLAMContext::get_tracking_state(const std::string& sceneID) const
{
  return get_single_scene_context(sceneID).m_trackingState;
}

const View_Ptr& SLAMContext::get_view(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_view;
}

View_CPtr SLAMContext::get_view(const std::string& sceneID) const
{
  return get_single_scene_context(sceneID).m_view;
}

const SpaintVoxelScene_Ptr& SLAMContext::get_voxel_scene(const std::string& sceneID)
{
  return get_single_scene_context(sceneID).m_voxelScene;
}

SpaintVoxelScene_CPtr SLAMContext::get_voxel_scene(const std::string& sceneID) const
{
  return get_single_scene_context(sceneID).m_voxelScene;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

const SLAMContext::SingleSceneContext& SLAMContext::get_single_scene_context(const std::string& sceneID) const
{
  return MapUtil::lookup(m_singleSceneContexts, sceneID);
}

void SLAMContext::set_input_raw_depth_image(const std::string& sceneID, ITMShortImage *inputRawDepthImage)
{
  set_if_different(m_singleSceneContexts[sceneID].m_inputRawDepthImage, inputRawDepthImage);
}

void SLAMContext::set_input_rgb_image(const std::string& sceneID, ITMUChar4Image *inputRGBImage)
{
  set_if_different(m_singleSceneContexts[sceneID].m_inputRGBImage, inputRGBImage);
}

void SLAMContext::set_live_surfel_render_state(const std::string& sceneID, ITMLib::ITMSurfelRenderState *liveSurfelRenderState)
{
  set_if_different(m_singleSceneContexts[sceneID].m_liveSurfelRenderState, liveSurfelRenderState);
}

void SLAMContext::set_live_voxel_render_state(const std::string& sceneID, ITMLib::ITMRenderState *liveVoxelRenderState)
{
  set_if_different(m_singleSceneContexts[sceneID].m_liveVoxelRenderState, liveVoxelRenderState);
}

void SLAMContext::set_surfel_scene(const std::string& sceneID, SpaintSurfelScene *surfelScene)
{
  set_if_different(m_singleSceneContexts[sceneID].m_surfelScene, surfelScene);
}

void SLAMContext::set_tracking_state(const std::string& sceneID, ITMLib::ITMTrackingState *trackingState)
{
  set_if_different(m_singleSceneContexts[sceneID].m_trackingState, trackingState);
}

void SLAMContext::set_view(const std::string& sceneID, ITMView *view)
{
  set_if_different(m_singleSceneContexts[sceneID].m_view, view);
}

void SLAMContext::set_voxel_scene(const std::string& sceneID, SpaintVoxelScene *voxelScene)
{
  set_if_different(m_singleSceneContexts[sceneID].m_voxelScene, voxelScene);
}

}
