/**
 * spaint: UniformVoxelSampler.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "sampling/interface/UniformVoxelSampler.h"

#include <tvgutil/RandomNumberGenerator.h>

#include "util/MemoryBlockFactory.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

UniformVoxelSampler::UniformVoxelSampler(int raycastResultSize, unsigned int seed)
: m_raycastResultSize(raycastResultSize),
  m_rng(new tvgutil::RandomNumberGenerator(seed)),
  m_sampledVoxelIndicesMB(MemoryBlockFactory::instance().make_block<int>(raycastResultSize))
{}

//#################### DESTRUCTOR ####################

UniformVoxelSampler::~UniformVoxelSampler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void UniformVoxelSampler::sample_voxels(const ITMFloat4Image *raycastResult, size_t numVoxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  // Choose which voxels to sample from the raycast result.
  int *sampledVoxelIndices = m_sampledVoxelIndicesMB->GetData(MEMORYDEVICE_CPU);
  for(size_t i = 0; i < numVoxelsToSample; ++i)
  {
    sampledVoxelIndices[i] = m_rng->generate_int_from_uniform(0, m_raycastResultSize - 1);
  }
  m_sampledVoxelIndicesMB->UpdateDeviceFromHost();

  // Write the sampled voxel locations into the sampled voxel locations array.
  write_sampled_voxel_locations(raycastResult, numVoxelsToSample, sampledVoxelLocationsMB);
}

}
