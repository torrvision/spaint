/**
 * spaint: VoxelSamplerFactory.cpp
 */

#include "sampling/VoxelSamplerFactory.h"

#include "sampling/cpu/VoxelSampler_CPU.h"

#ifdef WITH_CUDA
#include "sampling/cuda/VoxelSampler_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

VoxelSampler_CPtr VoxelSamplerFactory::make(int labelCount, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed, ITMLibSettings::DeviceType deviceType)
{
  VoxelSampler_CPtr sampler;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    sampler.reset(new VoxelSampler_CUDA(labelCount, maxVoxelsPerLabel, raycastResultSize, seed));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    sampler.reset(new VoxelSampler_CPU(labelCount, maxVoxelsPerLabel, raycastResultSize, seed));
  }

  return sampler;
}

}
