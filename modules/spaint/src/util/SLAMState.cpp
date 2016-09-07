/**
 * spaint: SLAMState.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "util/SLAMState.h"
using namespace ITMLib;
using namespace ORUtils;

namespace spaint {

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& SLAMState::get_depth_image_size() const
{
  return m_inputRawDepthImage->noDims;
}

ITMUCharImage_CPtr SLAMState::get_input_mask() const
{
  return m_inputMask;
}

const ITMShortImage_Ptr& SLAMState::get_input_raw_depth_image()
{
  return m_inputRawDepthImage;
}

ITMShortImage_Ptr SLAMState::get_input_raw_depth_image_copy() const
{
  ITMShortImage_Ptr copy(new ITMShortImage(m_inputRawDepthImage->noDims, true, false));
  copy->SetFrom(m_inputRawDepthImage.get(), ORUtils::MemoryBlock<short>::CPU_TO_CPU);
  return copy;
}

const ITMUChar4Image_Ptr& SLAMState::get_input_rgb_image()
{
  return m_inputRGBImage;
}

ITMUChar4Image_Ptr SLAMState::get_input_rgb_image_copy() const
{
  ITMUChar4Image_Ptr copy(new ITMUChar4Image(m_inputRGBImage->noDims, true, false));
  copy->SetFrom(m_inputRGBImage.get(), ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  return copy;
}

const ITMIntrinsics& SLAMState::get_intrinsics() const
{
  return m_view->calib.intrinsics_d;
}

const SurfelRenderState_Ptr& SLAMState::get_live_surfel_render_state()
{
  return m_liveSurfelRenderState;
}

const VoxelRenderState_Ptr& SLAMState::get_live_voxel_render_state()
{
  return m_liveVoxelRenderState;
}

const SE3Pose& SLAMState::get_pose() const
{
  return *m_trackingState->pose_d;
}

const Vector2i& SLAMState::get_rgb_image_size() const
{
  return m_inputRGBImage->noDims;
}

const SpaintSurfelScene_Ptr& SLAMState::get_surfel_scene()
{
  return m_surfelScene;
}

SpaintSurfelScene_CPtr SLAMState::get_surfel_scene() const
{
  return m_surfelScene;
}

const TrackingState_Ptr& SLAMState::get_tracking_state()
{
  return m_trackingState;
}

TrackingState_CPtr SLAMState::get_tracking_state() const
{
  return m_trackingState;
}

const View_Ptr& SLAMState::get_view()
{
  return m_view;
}

View_CPtr SLAMState::get_view() const
{
  return m_view;
}

const SpaintVoxelScene_Ptr& SLAMState::get_voxel_scene()
{
  return m_voxelScene;
}

SpaintVoxelScene_CPtr SLAMState::get_voxel_scene() const
{
  return m_voxelScene;
}

void SLAMState::set_input_mask(const ITMUCharImage_Ptr& inputMask)
{
  m_inputMask = inputMask;
}

void SLAMState::set_input_raw_depth_image(const ITMShortImage_Ptr& inputRawDepthImage)
{
  m_inputRawDepthImage = inputRawDepthImage;
}

void SLAMState::set_input_rgb_image(const ITMUChar4Image_Ptr& inputRGBImage)
{
  m_inputRGBImage = inputRGBImage;
}

void SLAMState::set_live_surfel_render_state(const SurfelRenderState_Ptr& liveSurfelRenderState)
{
  m_liveSurfelRenderState = liveSurfelRenderState;
}

void SLAMState::set_live_voxel_render_state(const VoxelRenderState_Ptr& liveVoxelRenderState)
{
  m_liveVoxelRenderState = liveVoxelRenderState;
}

void SLAMState::set_pose(const ORUtils::SE3Pose& pose)
{
  *m_trackingState->pose_d = pose;
}

void SLAMState::set_surfel_scene(const SpaintSurfelScene_Ptr& surfelScene)
{
  m_surfelScene = surfelScene;
}

void SLAMState::set_tracking_state(const TrackingState_Ptr& trackingState)
{
  m_trackingState = trackingState;
}

void SLAMState::set_view(ITMLib::ITMView *view)
{
  if(m_view.get() != view) m_view.reset(view);
}

void SLAMState::set_voxel_scene(const SpaintVoxelScene_Ptr& voxelScene)
{
  m_voxelScene = voxelScene;
}

}
