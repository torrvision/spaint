/**
 * spaint: VoxelSampler.h
 */

#ifndef H_SPAINT_VOXELSAMPLER
#define H_SPAINT_VOXELSAMPLER

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to sample voxels for which features should be calculated.
 */
class VoxelSampler
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The number of semantic labels that are in use. */
  const int m_labelCount;

  /** The size of the raycast result image (in pixels). */
  const int m_raycastResultSize;

  /**
   * A memory block in which to store the prefix sums for the voxel masks. These are used
   * to determine the locations in the output array into which to write sample voxels.
   */
  mutable ORUtils::MemoryBlock<unsigned int> m_voxelMaskPrefixSumsMB;

  /**
   * A memory block in which to store voxel masks indicating which voxels may be used as examples of which semantic labels.
   * The masks for the different labels are concatenated into a single 1D array.
   */
  mutable ORUtils::MemoryBlock<unsigned char> m_voxelMasksMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a voxel sampler.
   *
   * \param labelCount        The number of semantic labels that are in use.
   * \param raycastResultSize The size of the raycast result image (in pixels).
   * \param memoryDeviceType  The type of memory device on which to allocate the internal memory blocks (i.e. CPU or CUDA).
   */
  VoxelSampler(int labelCount, int raycastResultSize, MemoryDeviceType memoryDeviceType);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the voxel sampler.
   */
  virtual ~VoxelSampler();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the prefix sums for the voxel masks.
   *
   * \param voxelMasksMB          A memory block containing the voxel masks indicating which voxels may be used as examples of which semantic labels.
   * \param voxelMaskPrefixSumsMB A memory block into which to write the prefix sums for the voxel masks.
   */
  virtual void calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                                ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB) const = 0;

  /**
   * \brief Calculates the voxel masks.
   *
   * \param raycastResult The current raycast result.
   * \param voxelData     The scene's voxel data.
   * \param indexData     The scene's index data.
   * \param voxelMasksMB  A memory block into which to write the voxel masks indicating which voxels may be used as examples of which semantic labels.
   */
  virtual void calculate_voxel_masks(const ITMFloat4Image *raycastResult,
                                     const SpaintVoxel *voxelData,
                                     const ITMVoxelIndex::IndexData *indexData,
                                     ORUtils::MemoryBlock<unsigned char>& voxelMasksMB) const = 0;

  /**
   * \brief Sets the voxel counts for the various labels.
   *
   * \param voxelMaskPrefixSumsMB   A memory block containing the prefix sums for the voxel masks.
   * \param voxelCountsForLabelsMB  A memory block into which to write the numbers of voxels sampled for each label.
   */
  virtual void set_voxel_counts(const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const = 0;

  /**
   * \brief Writes the locations of the sampled voxels into the output memory block.
   *
   * \param raycastRaycast        The current raycast result.
   * \param voxelMasksMB          A memory block containing the voxel masks indicating which voxels may be used as examples of which semantic labels.
   * \param voxelMaskPrefixSumsMB A memory block containing the prefix sums for the voxel masks.
   * \param voxelLocationsMB      A memory block into which to write the locations of the sampled voxels.
   */
  virtual void write_voxel_locations(const ITMFloat4Image *raycastResult,
                                     const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                     const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                     ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Samples voxels from the current raycast result.
   *
   * \param raycastResult           The current raycast result.
   * \param scene                   The scene.
   * \param voxelLocationsMB        A memory block into which to write the locations of the sampled voxels.
   * \param voxelCountsForLabelsMB  A memory block into which to write the numbers of voxels sampled for each label.
   */
  void sample_voxels(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                     ORUtils::MemoryBlock<Vector3s>& voxelLocationsMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;
};

}

#endif
