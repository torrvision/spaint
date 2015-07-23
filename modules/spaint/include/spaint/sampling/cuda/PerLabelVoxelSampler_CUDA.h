/**
 * spaint: PerLabelVoxelSampler_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_PERLABELVOXELSAMPLER_CUDA
#define H_SPAINT_PERLABELVOXELSAMPLER_CUDA

#include "../interface/PerLabelVoxelSampler.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to sample voxels for each currently-used label from a scene using CUDA.
 */
class PerLabelVoxelSampler_CUDA : public PerLabelVoxelSampler
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based per-label voxel sampler.
   *
   * \param maxLabelCount     The maximum number of labels that can be in use.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result image (in pixels).
   * \param seed              The seed for the random number generator.
   */
  PerLabelVoxelSampler_CUDA(size_t maxLabelCount, size_t maxVoxelsPerLabel, int raycastResultSize, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<bool>& labelMaskMB) const;

  /** Override */
  virtual void calculate_voxel_masks(const ITMFloat4Image *raycastResult, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const;

  /** Override */
  virtual void write_candidate_voxel_counts(const ORUtils::MemoryBlock<bool>& labelMaskMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;

  /** Override */
  virtual void write_candidate_voxel_locations(const ITMFloat4Image *raycastResult) const;

  /** Override */
  virtual void write_sampled_voxel_locations(const ORUtils::MemoryBlock<bool>& labelMaskMB, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

}

#endif
