/**
 * spaint: VoxelSampler_CUDA.cu
 */

#include "sampling/cuda/VoxelSampler_CUDA.h"

#include <thrust/device_ptr.h>
#include <thrust/scan.h>

#include "sampling/shared/VoxelSampler_Shared.h"

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_voxel_masks(const Vector4f *raycastResult, int raycastResultSize,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                         int labelCount, unsigned char *voxelMasks)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < raycastResultSize)
  {
    update_masks_for_voxel(voxelIndex, raycastResult, raycastResultSize, voxelData, indexData, labelCount, voxelMasks);
  }
}

__global__ void ck_set_voxel_counts(int raycastResultSize, const unsigned int *voxelMaskPrefixSums, unsigned int *voxelCountsForLabels)
{
  int label = threadIdx.x + blockDim.x * blockIdx.x;
  set_voxel_count(label, raycastResultSize, voxelMaskPrefixSums, voxelCountsForLabels);
}

__global__ void ck_write_voxel_locations(const Vector4f *raycastResult, int raycastResultSize,
                                         const unsigned char *voxelMasks, const unsigned int *voxelMaskPrefixSums,
                                         int labelCount, Vector3s *voxelLocations)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < raycastResultSize)
  {
    write_voxel_location(voxelIndex, raycastResult, raycastResultSize, voxelMasks, voxelMaskPrefixSums, labelCount, voxelLocations);
  }
}

//#################### CONSTRUCTORS ####################

VoxelSampler_CUDA::VoxelSampler_CUDA(int labelCount, int maxVoxelsPerLabel, int raycastResultSize)
: VoxelSampler(labelCount, maxVoxelsPerLabel, raycastResultSize, MEMORYDEVICE_CUDA)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VoxelSampler_CUDA::calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                                         ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB) const
{
  const unsigned char *voxelMasks = voxelMasksMB.GetData(MEMORYDEVICE_CUDA);
  unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CUDA);

  // For each label:
  const int stride = m_raycastResultSize + 1;
  for(int k = 0; k < m_labelCount; ++k)
  {
    // Calculate the prefix sum of the voxel mask (this can be used to determine the locations in the voxel array for this label into which voxels should be written).
    thrust::device_ptr<const unsigned char> voxelMask(voxelMasks + k * stride);
    thrust::device_ptr<unsigned int> voxelMaskPrefixSum(voxelMaskPrefixSums + k * stride);

    thrust::exclusive_scan(
      voxelMask,
      voxelMask + stride,
      voxelMaskPrefixSum
    );
  }

#if 1
  // FOR DEBUGGING
  voxelMaskPrefixSumsMB.UpdateHostFromDevice();
#endif
}

void VoxelSampler_CUDA::calculate_voxel_masks(const ITMFloat4Image *raycastResult,
                                              const SpaintVoxel *voxelData,
                                              const ITMVoxelIndex::IndexData *indexData,
                                              ORUtils::MemoryBlock<unsigned char>& voxelMasksMB) const
{
  int threadsPerBlock = 256;
  int numBlocks = (m_raycastResultSize + threadsPerBlock - 1) / threadsPerBlock;
  ck_calculate_voxel_masks<<<numBlocks,threadsPerBlock>>>(
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    m_raycastResultSize,
    voxelData,
    indexData,
    m_labelCount,
    voxelMasksMB.GetData(MEMORYDEVICE_CUDA)
  );

#if 1
  // FOR DEBUGGING
  voxelMasksMB.UpdateHostFromDevice();
#endif
}

void VoxelSampler_CUDA::set_voxel_counts(const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                         ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  ck_set_voxel_counts<<<1,m_labelCount>>>(
    m_raycastResultSize,
    voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CUDA)
  );

  voxelCountsForLabelsMB.UpdateHostFromDevice();
}

void VoxelSampler_CUDA::write_voxel_locations(const ITMFloat4Image *raycastResult,
                                              const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                              const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                              ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB) const
{
  int threadsPerBlock = 256;
  int numBlocks = (m_raycastResultSize + threadsPerBlock - 1) / threadsPerBlock;
  ck_write_voxel_locations<<<numBlocks,threadsPerBlock>>>(
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    m_raycastResultSize,
    voxelMasksMB.GetData(MEMORYDEVICE_CUDA),
    voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CUDA),
    m_labelCount,
    voxelLocationsMB.GetData(MEMORYDEVICE_CUDA)
  );

#if 1
  // FOR DEBUGGING
  voxelLocationsMB.UpdateHostFromDevice();
#endif
}

}
