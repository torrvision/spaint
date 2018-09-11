/**
 * spaint: UniformVoxelSampler_CUDA.cu
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "sampling/cuda/UniformVoxelSampler_CUDA.h"

#include "sampling/shared/UniformVoxelSampler_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_write_sampled_voxel_locations(int voxelsToSample, const Vector4f *raycastResultData, const int *sampledVoxelIndices, Vector3s *sampledVoxelLocations)
{
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if(tid < voxelsToSample)
  {
    write_sampled_voxel_location(tid, raycastResultData, sampledVoxelIndices, sampledVoxelLocations);
  }
}

//#################### CONSTRUCTORS ####################

UniformVoxelSampler_CUDA::UniformVoxelSampler_CUDA(int raycastResultSize, unsigned int seed)
: UniformVoxelSampler(raycastResultSize, seed)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void UniformVoxelSampler_CUDA::write_sampled_voxel_locations(const ORFloat4Image *raycastResult, size_t sampledVoxelCount,
                                                             ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  int threadsPerBlock = 256;
  int numBlocks = (static_cast<int>(sampledVoxelCount) + threadsPerBlock - 1) / threadsPerBlock;

  ck_write_sampled_voxel_locations<<<numBlocks,threadsPerBlock>>>(
    static_cast<int>(sampledVoxelCount),
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    m_sampledVoxelIndicesMB->GetData(MEMORYDEVICE_CUDA),
    sampledVoxelLocationsMB.GetData(MEMORYDEVICE_CUDA)
  );
}

}
