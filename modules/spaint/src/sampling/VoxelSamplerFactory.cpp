/**
 * spaint: VoxelSamplerFactory.cpp
 */

#include "sampling/VoxelSamplerFactory.h"

#include "sampling/cpu/PerLabelVoxelSampler_CPU.h"
#include "sampling/cpu/UniformVoxelSampler_CPU.h"

#ifdef WITH_CUDA
#include "sampling/cuda/PerLabelVoxelSampler_CUDA.h"
#include "sampling/cuda/UniformVoxelSampler_CUDA.h"
#endif

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PerLabelVoxelSampler_CPtr VoxelSamplerFactory::make_per_label_sampler(size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize, unsigned int seed,
                                                                      ITMLibSettings::DeviceType deviceType)
{
  PerLabelVoxelSampler_CPtr sampler;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    sampler.reset(new PerLabelVoxelSampler_CUDA(maxLabelCount, maxVoxelsPerLabel, raycastResultSize, seed));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    sampler.reset(new PerLabelVoxelSampler_CPU(maxLabelCount, maxVoxelsPerLabel, raycastResultSize, seed));
  }

  return sampler;
}

UniformVoxelSampler_CPtr VoxelSamplerFactory::make_uniform_sampler(int raycastResultSize, unsigned int seed, ITMLibSettings::DeviceType deviceType)
{
  UniformVoxelSampler_CPtr sampler;

  if(deviceType == ITMLibSettings::DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    sampler.reset(new UniformVoxelSampler_CUDA(raycastResultSize, seed));
#else
    // This should never happen as things stand - we set deviceType to DEVICE_CPU to false if CUDA support isn't available.
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    sampler.reset(new UniformVoxelSampler_CPU(raycastResultSize, seed));
  }

  return sampler;
}

}
