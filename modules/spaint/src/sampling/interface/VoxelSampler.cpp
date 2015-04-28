/**
 * spaint: VoxelSampler.cpp
 */

#include "sampling/interface/VoxelSampler.h"

#include <tvgutil/RandomNumberGenerator.h>

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelSampler::VoxelSampler(const LabelManager_CPtr& labelManager, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed)
: m_candidateVoxelIndicesMB(labelManager->get_max_label_count() * maxVoxelsPerLabel, true, true),
  m_candidateVoxelLocationsMB(labelManager->get_max_label_count() * raycastResultSize, true, true),
  m_labelManager(labelManager),
  m_maxLabelCount(labelManager->get_max_label_count()),
  m_maxVoxelsPerLabel(maxVoxelsPerLabel),
  m_raycastResultSize(raycastResultSize),
  m_rng(new tvgutil::RandomNumberGenerator(seed)),
  m_voxelMaskPrefixSumsMB(labelManager->get_max_label_count() * (raycastResultSize + 1), true, true),
  m_voxelMasksMB(labelManager->get_max_label_count() * (raycastResultSize + 1), true, true)
{}

//#################### DESTRUCTOR ####################

VoxelSampler::~VoxelSampler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelSampler::sample_voxels(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                 ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  // Calculate the voxel masks for the various labels (these indicate which voxels could serve as examples of each label).
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_voxel_masks(raycastResult, voxelData, indexData, m_voxelMasksMB);

  // Calculate the prefix sums of the voxel masks (these can be used to determine the locations in the candidate voxel locations array
  // into which candidate voxels should be written).
  calculate_voxel_mask_prefix_sums(m_voxelMasksMB, m_voxelMaskPrefixSumsMB);

  // Based on the voxel masks and the prefix sums, write the candidate voxel locations into the candidate voxel locations array.
  write_candidate_voxel_locations(raycastResult, m_voxelMasksMB, m_voxelMaskPrefixSumsMB, m_candidateVoxelLocationsMB);

  // Write the candidate voxel counts for the different labels into the voxel counts array.
  write_candidate_voxel_counts(m_voxelMaskPrefixSumsMB, voxelCountsForLabelsMB);

  // Randomly choose candidate voxel locations to sample for each label.
  // FIXME: It might be a good idea to implement this on both the CPU and GPU to avoid the memory transfer.
  unsigned int *voxelCountsForLabels = voxelCountsForLabelsMB.GetData(MEMORYDEVICE_CPU);
  int *candidateVoxelIndices = m_candidateVoxelIndicesMB.GetData(MEMORYDEVICE_CPU);
  for(int k = 0; k < m_maxLabelCount; ++k)
  {
    for(int i = 0; i < m_maxVoxelsPerLabel; ++i)
    {
      candidateVoxelIndices[k * m_maxVoxelsPerLabel + i] = i < voxelCountsForLabels[k] ? m_rng->generate_int_from_uniform(0, voxelCountsForLabels[k] - 1) : -1;
    }
  }
  m_candidateVoxelIndicesMB.UpdateDeviceFromHost();

  // Write the sampled voxel locations into the sampled voxel locations array.
  write_sampled_voxel_locations(sampledVoxelLocationsMB);

  // Update the voxel counts for the different labels to reflect the number of voxels sampled.
  for(int k = 0; k < m_maxLabelCount; ++k)
  {
    if(voxelCountsForLabels[k] > m_maxVoxelsPerLabel) voxelCountsForLabels[k] = m_maxVoxelsPerLabel;
  }
  voxelCountsForLabelsMB.UpdateDeviceFromHost();
}

}
