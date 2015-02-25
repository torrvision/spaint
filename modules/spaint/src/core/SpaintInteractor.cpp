/**
 * spaint: SpaintInteractor.cpp
 */

#include "core/SpaintInteractor.h"

#include "markers/cpu/VoxelMarker_CPU.h"
#ifdef WITH_CUDA
#include "markers/cuda/VoxelMarker_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintInteractor::SpaintInteractor(const SpaintModel_Ptr& model)
: m_brushRadius(2), m_model(model), m_semanticLabel(1)
{
  // Set up the voxel marker.
  if(model->get_settings()->deviceType == ITMLibSettings::DEVICE_CUDA)
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

int SpaintInteractor::get_brush_radius() const
{
  return m_brushRadius;
}

const boost::optional<Eigen::Vector3f>& SpaintInteractor::get_pick_point() const
{
  return m_pickPoint;
}

unsigned char SpaintInteractor::get_semantic_label() const
{
  return m_semanticLabel;
}

void SpaintInteractor::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label)
{
  m_voxelMarker->mark_voxels(voxelLocationsMB, label, m_model->get_scene().get());
}

void SpaintInteractor::set_brush_radius(int brushRadius)
{
  m_brushRadius = brushRadius;
}

void SpaintInteractor::set_pick_point(const boost::optional<Eigen::Vector3f>& pickPoint)
{
  m_pickPoint = pickPoint;
}

void SpaintInteractor::set_semantic_label(unsigned char semanticLabel)
{
  m_semanticLabel = semanticLabel;
}

}
