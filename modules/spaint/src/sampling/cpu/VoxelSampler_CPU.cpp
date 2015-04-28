/**
 * spaint: VoxelSampler_CPU.cpp
 */

#include "sampling/cpu/VoxelSampler_CPU.h"

#include "sampling/shared/VoxelSampler_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelSampler_CPU::VoxelSampler_CPU(int maxLabelCount, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed)
: VoxelSampler(maxLabelCount, maxVoxelsPerLabel, raycastResultSize, seed)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VoxelSampler_CPU::calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                                        ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB) const
{
  const unsigned char *voxelMasks = voxelMasksMB.GetData(MEMORYDEVICE_CPU);
  unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CPU);

  // For each label:
  const int stride = m_raycastResultSize + 1;
  for(int k = 0; k < m_maxLabelCount; ++k)
  {
    // Calculate the prefix sum of the voxel mask.
    voxelMaskPrefixSums[k * stride] = 0;
    for(int i = 1; i < stride; ++i)
    {
      voxelMaskPrefixSums[k * stride + i] = voxelMaskPrefixSums[k * stride + (i-1)] + voxelMasks[k * stride + (i-1)];
    }
  }
}

void VoxelSampler_CPU::calculate_voxel_masks(const ITMFloat4Image *raycastResult,
                                             const SpaintVoxel *voxelData,
                                             const ITMVoxelIndex::IndexData *indexData,
                                             ORUtils::MemoryBlock<unsigned char>& voxelMasksMB) const
{
  unsigned char *voxelMasks = voxelMasksMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < m_raycastResultSize; ++voxelIndex)
  {
    // Update the voxel masks based on the contents of the voxel.
    update_masks_for_voxel(
      voxelIndex,
      raycastResult->GetData(MEMORYDEVICE_CPU),
      m_raycastResultSize,
      voxelData,
      indexData,
      m_maxLabelCount,
      voxelMasks
    );
  }
}

void VoxelSampler_CPU::write_candidate_voxel_counts(const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                                    ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  const unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CPU);
  unsigned int *voxelCountsForLabels = voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int k = 0; k < m_maxLabelCount; ++k)
  {
    write_candidate_voxel_count(k, m_raycastResultSize, voxelMaskPrefixSums, voxelCountsForLabels);
  }
}

void VoxelSampler_CPU::write_candidate_voxel_locations(const ITMFloat4Image *raycastResult,
                                                       const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                                       const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                                       ORUtils::MemoryBlock<Vector3s>& candidateVoxelLocationsMB) const
{
  const unsigned char *voxelMasks = voxelMasksMB.GetData(MEMORYDEVICE_CPU);
  const unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *candidateVoxelLocations = candidateVoxelLocationsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < m_raycastResultSize; ++voxelIndex)
  {
    write_candidate_voxel_location(
      voxelIndex,
      raycastResult->GetData(MEMORYDEVICE_CPU),
      m_raycastResultSize,
      voxelMasks,
      voxelMaskPrefixSums,
      m_maxLabelCount,
      candidateVoxelLocations
    );
  }
}

void VoxelSampler_CPU::write_sampled_voxel_locations(ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const
{
  const Vector3s *candidateVoxelLocations = m_candidateVoxelLocationsMB.GetData(MEMORYDEVICE_CPU);
  const int *candidateVoxelIndices = m_candidateVoxelIndicesMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *sampledVoxelLocations = sampledVoxelLocationsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < m_maxVoxelsPerLabel; ++voxelIndex)
  {
    copy_sampled_voxel_locations(
      voxelIndex,
      m_maxLabelCount,
      m_maxVoxelsPerLabel,
      m_raycastResultSize,
      candidateVoxelLocations,
      candidateVoxelIndices,
      sampledVoxelLocations
    );
  }
}

}
