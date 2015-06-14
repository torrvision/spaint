/**
 * spaint: PerLabelVoxelSampler_CUDA.cu
 */

#include "sampling/cuda/PerLabelVoxelSampler_CUDA.h"

#include <cassert>

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced when including the Thrust headers.
  #pragma warning(disable:4267)
#endif

#include <thrust/device_ptr.h>
#include <thrust/scan.h>

#ifdef _MSC_VER
  // Reenable the suppressed warnings for the rest of the translation unit.
  #pragma warning(default:4267)
#endif

#include "sampling/shared/PerLabelVoxelSampler_Shared.h"

#define DEBUGGING 0

namespace spaint {

//#################### CUDA KERNELS ####################

__global__ void ck_calculate_voxel_masks(const Vector4f *raycastResult, int raycastResultSize,
                                         const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData,
                                         size_t maxLabelCount, unsigned char *voxelMasks)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < raycastResultSize)
  {
    update_masks_for_voxel(voxelIndex, raycastResult, raycastResultSize, voxelData, indexData, maxLabelCount, voxelMasks);
  }
}

__global__ void ck_copy_sampled_voxel_locations(const bool *labelMask, size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize,
                                                const Vector3s *voxelLocationsByClass, const int *randomVoxelIndices,
                                                Vector3s *sampledVoxelLocations)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < maxVoxelsPerLabel)
  {
    copy_sampled_voxel_locations(voxelIndex, labelMask, maxLabelCount, maxVoxelsPerLabel, raycastResultSize, voxelLocationsByClass, randomVoxelIndices,
                                 sampledVoxelLocations);
  }
}

__global__ void ck_write_candidate_voxel_counts(int raycastResultSize, const bool *labelMask, const unsigned int *voxelMaskPrefixSums,
                                                unsigned int *voxelCountsForLabels)
{
  int label = threadIdx.x + blockDim.x * blockIdx.x;
  write_candidate_voxel_count(label, raycastResultSize, labelMask, voxelMaskPrefixSums, voxelCountsForLabels);
}

__global__ void ck_write_candidate_voxel_locations(const Vector4f *raycastResult, int raycastResultSize,
                                                   const unsigned char *voxelMasks, const unsigned int *voxelMaskPrefixSums,
                                                   size_t maxLabelCount, Vector3s *voxelLocations)
{
  int voxelIndex = threadIdx.x + blockDim.x * blockIdx.x;
  if(voxelIndex < raycastResultSize)
  {
    write_candidate_voxel_location(voxelIndex, raycastResult, raycastResultSize, voxelMasks, voxelMaskPrefixSums, maxLabelCount, voxelLocations);
  }
}

//#################### CONSTRUCTORS ####################

VoxelSampler_CUDA::VoxelSampler_CUDA(size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize, unsigned int seed)
: VoxelSampler(maxLabelCount, maxVoxelsPerLabel, raycastResultSize, seed)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VoxelSampler_CUDA::calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<bool>& labelMaskMB) const
{
  const bool *labelMask = labelMaskMB.GetData(MEMORYDEVICE_CPU);
  const unsigned char *voxelMasks = m_voxelMasksMB.GetData(MEMORYDEVICE_CUDA);
  unsigned int *voxelMaskPrefixSums = m_voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CUDA);

  // For each possible label:
  const int stride = m_raycastResultSize + 1;
  for(size_t k = 0; k < m_maxLabelCount; ++k)
  {
    // If the label is not currently in use, continue.
    if(!labelMask[k]) continue;

    // Calculate the prefix sum of the voxel mask.
    thrust::device_ptr<const unsigned char> voxelMask(voxelMasks + k * stride);
    thrust::device_ptr<unsigned int> voxelMaskPrefixSum(voxelMaskPrefixSums + k * stride);

    thrust::exclusive_scan(
      voxelMask,
      voxelMask + stride,
      voxelMaskPrefixSum
    );
  }

#if DEBUGGING
  m_voxelMaskPrefixSumsMB.UpdateHostFromDevice();
#endif
}

void VoxelSampler_CUDA::calculate_voxel_masks(const ITMFloat4Image *raycastResult,
                                              const SpaintVoxel *voxelData,
                                              const ITMVoxelIndex::IndexData *indexData) const
{
  int threadsPerBlock = 256;
  int numBlocks = (m_raycastResultSize + threadsPerBlock - 1) / threadsPerBlock;
  ck_calculate_voxel_masks<<<numBlocks,threadsPerBlock>>>(
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    m_raycastResultSize,
    voxelData,
    indexData,
    m_maxLabelCount,
    m_voxelMasksMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_voxelMasksMB.UpdateHostFromDevice();
#endif
}

void VoxelSampler_CUDA::write_candidate_voxel_counts(const ORUtils::MemoryBlock<bool>& labelMaskMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  // If the label count starts getting too large, we should consider splitting this into multiple thread blocks.
  int maxLabelCount = static_cast<int>(m_maxLabelCount);
  assert(maxLabelCount <= 256);

  ck_write_candidate_voxel_counts<<<1,maxLabelCount>>>(
    m_raycastResultSize,
    labelMaskMB.GetData(MEMORYDEVICE_CUDA),
    m_voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CUDA),
    voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CUDA)
  );

  voxelCountsForLabelsMB.UpdateHostFromDevice();
}

void VoxelSampler_CUDA::write_candidate_voxel_locations(const ITMFloat4Image *raycastResult) const
{
  int threadsPerBlock = 256;
  int numBlocks = (m_raycastResultSize + threadsPerBlock - 1) / threadsPerBlock;
  ck_write_candidate_voxel_locations<<<numBlocks,threadsPerBlock>>>(
    raycastResult->GetData(MEMORYDEVICE_CUDA),
    m_raycastResultSize,
    m_voxelMasksMB.GetData(MEMORYDEVICE_CUDA),
    m_voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CUDA),
    m_maxLabelCount,
    m_candidateVoxelLocationsMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  m_candidateVoxelLocationsMB.UpdateHostFromDevice();
#endif
}

void VoxelSampler_CUDA::write_sampled_voxel_locations(const ORUtils::MemoryBlock<bool>& labelMaskMB, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  int threadsPerBlock = 256;
  int numBlocks = (static_cast<int>(m_maxVoxelsPerLabel) + threadsPerBlock - 1) / threadsPerBlock;
  ck_copy_sampled_voxel_locations<<<numBlocks,threadsPerBlock>>>(
    labelMaskMB.GetData(MEMORYDEVICE_CUDA),
    m_maxLabelCount,
    m_maxVoxelsPerLabel,
    m_raycastResultSize,
    m_candidateVoxelLocationsMB.GetData(MEMORYDEVICE_CUDA),
    m_candidateVoxelIndicesMB.GetData(MEMORYDEVICE_CUDA),
    sampledVoxelLocationsMB.GetData(MEMORYDEVICE_CUDA)
  );

#if DEBUGGING
  sampledVoxelLocationsMB.UpdateHostFromDevice();
#endif
}

}
