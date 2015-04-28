/**
 * spaint: VoxelSampler_CPU.h
 */

#ifndef H_SPAINT_VOXELSAMPLER_CPU
#define H_SPAINT_VOXELSAMPLER_CPU

#include "../interface/VoxelSampler.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to sample voxels from a scene using the CPU.
 */
class VoxelSampler_CPU : public VoxelSampler
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based voxel sampler.
   *
   * \param labelManager      The label manager.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result image (in pixels).
   * \param seed              The seed for the random number generator.
   */
  VoxelSampler_CPU(const LabelManager_CPtr& labelManager, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void calculate_voxel_mask_prefix_sums() const;

  /** Override */
  virtual void calculate_voxel_masks(const ITMFloat4Image *raycastResult, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const;

  /** Override */
  virtual void write_candidate_voxel_counts(ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;

  /** Override */
  virtual void write_candidate_voxel_locations(const ITMFloat4Image *raycastResult) const;

  /** Override */
  virtual void write_sampled_voxel_locations(ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

}

#endif
