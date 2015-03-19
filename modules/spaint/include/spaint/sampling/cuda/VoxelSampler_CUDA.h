/**
 * spaint: VoxelSampler_CUDA.h
 */

#ifndef H_SPAINT_VOXELSAMPLER_CUDA
#define H_SPAINT_VOXELSAMPLER_CUDA

#include "../interface/VoxelSampler.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to sample voxels for which features should be calculated using CUDA.
 */
class VoxelSampler_CUDA : public VoxelSampler
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based voxel sampler.
   *
   * \param labelCount        The number of semantic labels that are in use.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result image (in pixels).
   */
  VoxelSampler_CUDA(int labelCount, int maxVoxelsPerLabel, int raycastResultSize);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                                ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB) const;

  /** Override */
  virtual void calculate_voxel_masks(const ITMFloat4Image *raycastResult,
                                     const SpaintVoxel *voxelData,
                                     const ITMVoxelIndex::IndexData *indexData,
                                     ORUtils::MemoryBlock<unsigned char>& voxelMasksMB) const;

  /** Override */
  virtual void set_voxel_counts(const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;

  /** Override */
  virtual void write_sampled_voxel_locations(ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;

  /** Override */
  virtual void write_voxel_locations(const ITMFloat4Image *raycastResult,
                                     const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                     const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                     ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB) const;
};

}

#endif
