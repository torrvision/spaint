/**
 * spaint: VoxelSampler_CPU.cpp
 */

#include "sampling/cpu/VoxelSampler_CPU.h"

#include "sampling/shared/VoxelSampler_Shared.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelSampler_CPU::VoxelSampler_CPU(int labelCount, int raycastResultSize)
: VoxelSampler(labelCount, raycastResultSize, MEMORYDEVICE_CPU)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void VoxelSampler_CPU::calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                                        ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB) const
{
  const unsigned char *voxelMasks = voxelMasksMB.GetData(MEMORYDEVICE_CPU);
  unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CPU);

  // For each label:
  const int stride = m_raycastResultSize + 1;
  for(int k = 0; k < m_labelCount; ++k)
  {
    // Calculate the prefix sum of the voxel mask (this can be used to determine the locations in the voxel array for this label into which voxels should be written).
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
  //#pragma omp parallel for
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
      m_labelCount,
      voxelMasks
    );
  }
}

void VoxelSampler_CPU::set_voxel_counts(const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                        ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  const unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CPU);
  unsigned int *voxelCountsForLabels = voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int k = 0; k < m_labelCount; ++k)
  {
    set_voxel_count(k, m_raycastResultSize, voxelMaskPrefixSums, voxelCountsForLabels);
  }
}

void VoxelSampler_CPU::write_voxel_locations(const ITMFloat4Image *raycastResult,
                                             const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                             const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                             ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB) const
{
  const unsigned char *voxelMasks = voxelMasksMB.GetData(MEMORYDEVICE_CPU);
  const unsigned int *voxelMaskPrefixSums = voxelMaskPrefixSumsMB.GetData(MEMORYDEVICE_CPU);
  Vector3s *voxelLocations = voxelLocationsMB.GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  //#pragma omp parallel for
#endif
  for(int voxelIndex = 0; voxelIndex < m_raycastResultSize; ++voxelIndex)
  {
    write_voxel_location(
      voxelIndex,
      raycastResult->GetData(MEMORYDEVICE_CPU),
      m_raycastResultSize,
      voxelMasks,
      voxelMaskPrefixSums,
      m_labelCount,
      voxelLocations
    );
  }
}

}
