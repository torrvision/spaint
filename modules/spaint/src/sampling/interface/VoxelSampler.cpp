/**
 * spaint: VoxelSampler.cpp
 */

#include "sampling/interface/VoxelSampler.h"

#include <tvgutil/random/RandomNumberGenerator.h>

namespace {

//#################### LOCAL VARIABLES ####################

/** The random number generator. */
tvgutil::RandomNumberGenerator rng(12345);

}

namespace spaint {

//#################### CONSTRUCTORS ####################

VoxelSampler::VoxelSampler(int labelCount, int raycastResultSize, MemoryDeviceType memoryDeviceType)
: m_labelCount(labelCount),
  m_raycastResultSize(raycastResultSize),
  m_voxelLocationsByClassMB(m_labelCount * raycastResultSize, true, true/*memoryDeviceType*/),
  m_voxelMaskPrefixSumsMB(m_labelCount * (raycastResultSize + 1), true, true/*memoryDeviceType*/),
  m_voxelMasksMB(m_labelCount * (raycastResultSize + 1), true, true/*memoryDeviceType*/)
{}

//#################### DESTRUCTOR ####################

VoxelSampler::~VoxelSampler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void VoxelSampler::sample_voxels(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene, int maxVoxelsPerLabel,
                                 ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const
{
  // Calculate the voxel masks for the various labels (these indicate which voxels could serve as examples of each label).
  const SpaintVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const ITMVoxelIndex::IndexData *indexData = scene->index.getIndexData();
  calculate_voxel_masks(raycastResult, voxelData, indexData, m_voxelMasksMB);

  // Calculate the prefix sums of the voxel masks (these can be used to determine the locations in the voxel array into which voxels should be written).
  calculate_voxel_mask_prefix_sums(m_voxelMasksMB, m_voxelMaskPrefixSumsMB);

  // Based on the voxel masks and the prefix sums, write the voxel locations into the voxel location array.
  write_voxel_locations(raycastResult, m_voxelMasksMB, m_voxelMaskPrefixSumsMB, m_voxelLocationsByClassMB);

  // Sets the voxel counts for the different labels.
  set_voxel_counts(m_voxelMaskPrefixSumsMB, voxelCountsForLabelsMB);

  // Randomly sample from the voxel locations for each class.
  // TODO

  // Write the sampled voxel locations into the sampled voxel location array.
  // TODO
}

}
