/**
 * spaint: SpaintModel.cpp
 */

#include "core/SpaintModel.h"

#include "marking/cpu/VoxelMarker_CPU.h"
#ifdef WITH_CUDA
#include "marking/cuda/VoxelMarker_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintModel::SpaintModel(const Scene_Ptr& scene, const Vector2i& rgbImageSize, const Vector2i& depthImageSize, const TrackingState_Ptr& trackingState,
                         const Settings_CPtr& settings)
: m_depthImageSize(depthImageSize), m_rgbImageSize(rgbImageSize), m_scene(scene), m_settings(settings), m_trackingState(trackingState)
{
  // Set up the voxel marker.
  if(settings->deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    // Use the CUDA implementation.
    m_voxelMarker.reset(new VoxelMarker_CUDA);
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    // Use the CPU implementation.
    m_voxelMarker.reset(new VoxelMarker_CPU);
  }
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

void SpaintModel::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label)
{
  m_voxelMarker->mark_voxels(voxelLocationsMB, label, m_scene.get());
}

void SpaintModel::set_view(ITMView *view)
{
  if(m_view.get() != view) m_view.reset(view);
}

}
