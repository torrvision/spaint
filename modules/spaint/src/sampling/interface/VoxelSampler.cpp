/**
 * spaint: VoxelSampler.cpp
 */

#include "sampling/interface/VoxelSampler.h"

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelSampler::VoxelSampler(int labelCount, int raycastResultSize, MemoryDeviceType memoryDeviceType)
: m_labelCount(labelCount),
  m_raycastResultSize(raycastResultSize),
  m_voxelMaskPrefixSumsMB(m_labelCount * (raycastResultSize + 1), true, true/*memoryDeviceType*/),
  m_voxelMasksMB(m_labelCount * (raycastResultSize + 1), true, true/*memoryDeviceType*/)
{}

//#################### DESTRUCTOR ####################

VoxelSampler::~VoxelSampler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelSampler::sample_voxels(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                                 ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  // Calculate the voxel masks for the various labels (these indicate which voxels could serve as examples of each label).
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  this->calculate_voxel_masks(raycastResult, voxelData, indexData, m_voxelMasksMB);

  // Calculate the prefix sums of the voxel masks (these can be used to determine the locations in the voxel array into which voxels should be written).
  this->calculate_voxel_mask_prefix_sums(m_voxelMasksMB, m_voxelMaskPrefixSumsMB);

  // Based on the voxel masks and the prefix sums, write the voxel locations into the voxel location array.
  this->write_voxel_locations(raycastResult, m_voxelMasksMB, m_voxelMaskPrefixSumsMB, voxelLocationsMB);

  // Sets the voxel counts for the different labels.
  this->set_voxel_counts(m_voxelMaskPrefixSumsMB, voxelCountsForLabelsMB);
}

}
