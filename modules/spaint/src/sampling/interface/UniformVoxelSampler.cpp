/**
 * spaint: UniformVoxelSampler.cpp
 */

#include "sampling/interface/UniformVoxelSampler.h"

#include <tvgutil/RandomNumberGenerator.h>

namespace spaint {

//#################### CONSTRUCTORS ####################

UniformVoxelSampler::UniformVoxelSampler(int raycastResultSize, unsigned int seed)
: m_raycastResultSize(raycastResultSize),
  m_rng(new tvgutil::RandomNumberGenerator(seed)),
  m_sampledVoxelIndicesMB(raycastResultSize, true, true)
{}

//#################### DESTRUCTOR ####################

UniformVoxelSampler::~UniformVoxelSampler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void UniformVoxelSampler::sample_voxels(const ITMFloat4Image *raycastResult, size_t voxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  // Choose which voxels to sample from the raycast result.
  choose_voxels_to_sample(voxelsToSample);

  // Write the sampled voxel locations into the sampled voxel locations array.
  write_sampled_voxel_locations(raycastResult, voxelsToSample, sampledVoxelLocationsMB);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void UniformVoxelSampler::choose_voxels_to_sample(size_t voxelsToSample) const
{
  int *sampledVoxelIndices = m_sampledVoxelIndicesMB.GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0; i < voxelsToSample; ++i)
  {
    sampledVoxelIndices[i] = m_rng->generate_int_from_uniform(0, m_raycastResultSize - 1);
  }

  m_sampledVoxelIndicesMB.UpdateDeviceFromHost();
}

}
