/**
 * spaint: VoxelSampler.cpp
 */

#include "sampling/interface/VoxelSampler.h"

#include <tvgutil/RandomNumberGenerator.h>

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelSampler::VoxelSampler(int maxLabelCount, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed)
: m_candidateVoxelIndicesMB(maxLabelCount * maxVoxelsPerLabel, true, true),
  m_candidateVoxelLocationsMB(maxLabelCount * raycastResultSize, true, true),
  m_maxLabelCount(maxLabelCount),
  m_maxVoxelsPerLabel(maxVoxelsPerLabel),
  m_raycastResultSize(raycastResultSize),
  m_rng(new tvgutil::RandomNumberGenerator(seed)),
  m_voxelMaskPrefixSumsMB(maxLabelCount * (raycastResultSize + 1), true, true),
  m_voxelMasksMB(maxLabelCount * (raycastResultSize + 1), true, true)
{}

//#################### DESTRUCTOR ####################

VoxelSampler::~VoxelSampler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelSampler::sample_voxels(const ITMFloat4Image *raycastResult,
                                 const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                 const ORUtils::MemoryBlock<bool>& labelMaskMB,
                                 ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB,
                                 ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  // Calculate the voxel masks for all labels (these indicate which voxels could serve as examples of each label).
  // Note that we calculate masks even for unused labels to avoid unnecessary branching - these will always be empty.
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_voxel_masks(raycastResult, voxelData, indexData);

  // Calculate the prefix sums of the voxel masks for the used labels (these can be used to determine the locations in
  // the candidate voxel locations array into which candidate voxels should be written).
  calculate_voxel_mask_prefix_sums(labelMaskMB);

  // Based on the voxel masks and the prefix sums, write the candidate voxel locations into the candidate voxel locations array.
  // Note that we do not need to explicitly use the label mask when writing candidate voxel locations, since the voxel mask for
  // an unused label will be empty in any case.
  write_candidate_voxel_locations(raycastResult);

  // Write the candidate voxel counts for the used labels into the voxel counts array.
  write_candidate_voxel_counts(labelMaskMB, voxelCountsForLabelsMB);

  // Randomly choose candidate voxel locations to sample for each used label.
  // FIXME: It might be a good idea to implement this on both the CPU and GPU to avoid the memory transfer.
  const bool *labelMask = labelMaskMB.GetData(MEMORYDEVICE_CPU);
  unsigned int *voxelCountsForLabels = voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CPU);
  int *candidateVoxelIndices = m_candidateVoxelIndicesMB.GetData(MEMORYDEVICE_CPU);
  for(int k = 0; k < m_maxLabelCount; ++k)
  {
    if(!labelMask[k]) continue;

    for(int i = 0; i < m_maxVoxelsPerLabel; ++i)
    {
      candidateVoxelIndices[k * m_maxVoxelsPerLabel + i] = i < voxelCountsForLabels[k] ? m_rng->generate_int_from_uniform(0, voxelCountsForLabels[k] - 1) : -1;
    }
  }
  m_candidateVoxelIndicesMB.UpdateDeviceFromHost();

  // Write the sampled voxel locations into the sampled voxel locations array.
  write_sampled_voxel_locations(labelMaskMB, sampledVoxelLocationsMB);

  // Update the voxel counts for the different labels to reflect the number of voxels sampled.
  for(int k = 0; k < m_maxLabelCount; ++k)
  {
    if(labelMask[k])
    {
      if(voxelCountsForLabels[k] > m_maxVoxelsPerLabel) voxelCountsForLabels[k] = m_maxVoxelsPerLabel;
    }
    else voxelCountsForLabels[k] = 0;
  }
  voxelCountsForLabelsMB.UpdateDeviceFromHost();
}

}
