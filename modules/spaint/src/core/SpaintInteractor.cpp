/**
 * spaint: SpaintInteractor.cpp
 */

#include "core/SpaintInteractor.h"

#include "marking/cpu/VoxelMarker_CPU.h"
#ifdef WITH_CUDA
#include "marking/cuda/VoxelMarker_CUDA.h"
#endif

namespace spaint {

//#################### CONSTRUCTORS ####################

SpaintInteractor::SpaintInteractor(const SpaintModel_Ptr& model)
: m_model(model)
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

void SpaintInteractor::mark_voxels(const ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, unsigned char label)
{
  m_voxelMarker->mark_voxels(voxelLocationsMB, label, m_model->get_scene().get());
}

}
