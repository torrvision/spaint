/**
 * spaint: UniformVoxelSampler_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "sampling/cpu/UniformVoxelSampler_CPU.h"

#include "sampling/shared/UniformVoxelSampler_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

UniformVoxelSampler_CPU::UniformVoxelSampler_CPU(int raycastResultSize, unsigned int seed)
: UniformVoxelSampler(raycastResultSize, seed)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void UniformVoxelSampler_CPU::write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t sampledVoxelCount,
                                                            ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  const Vector4f *raycastResultData = raycastResult->GetData(MEMORYDEVICE_CPU);
  const int *sampledVoxelIndices = m_sampledVoxelIndicesMB->GetData(MEMORYDEVICE_CPU);
  Vector3s *sampledVoxelLocations = sampledVoxelLocationsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int tid = 0; tid < static_cast<int>(sampledVoxelCount); ++tid)
  {
    write_sampled_voxel_location(tid, raycastResultData, sampledVoxelIndices, sampledVoxelLocations);
  }
}

}
