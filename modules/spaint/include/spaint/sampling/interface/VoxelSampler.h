/**
 * spaint: VoxelSampler.h
 */

#ifndef H_SPAINT_VOXELSAMPLER
#define H_SPAINT_VOXELSAMPLER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace tvgutil {

//#################### FORWARD DECLARATIONS ####################

class RandomNumberGenerator;

}

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to sample voxels for which features should be calculated.
 */
class VoxelSampler
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A memory block in which to store random indices when sampling from the candidate voxels for each class. */
  mutable ORUtils::MemoryBlock<int> m_candidateVoxelIndicesMB;

  /** A memory block in which to store the locations of candidate voxels in the raycast result, grouped by semantic class. */
  mutable ORUtils::MemoryBlock<Vector3s> m_candidateVoxelLocationsMB;

  /** The number of semantic labels that are in use. */
  const int m_labelCount;

  /** The maximum number of voxels to sample for each label. */
  const int m_maxVoxelsPerLabel;

  /** The size of the raycast result (in pixels). */
  const int m_raycastResultSize;

  /** A random number generator. */
  boost::shared_ptr<tvgutil::RandomNumberGenerator> m_rng;

  /**
   * A memory block in which to store the prefix sums for the voxel masks. These are used to determine the locations in the
   * candidate voxel locations array into which to write candidate voxels.
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
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   */
  VoxelSampler(int labelCount, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed);

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
   * \brief Writes the number of candidate voxels that are available for each label into the voxel counts for labels memory block.
   *
   * \param voxelMaskPrefixSumsMB   A memory block containing the prefix sums for the voxel masks.
   * \param voxelCountsForLabelsMB  A memory block into which to write voxel counts for each label.
   */
  virtual void write_candidate_voxel_counts(const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                            ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const = 0;

  /**
   * \brief Writes the locations of the candidate voxels into the candidate voxel locations memory block.
   *
   * \param raycastRaycast            The current raycast result.
   * \param voxelMasksMB              A memory block containing the voxel masks indicating which voxels may be used as examples of which semantic labels.
   * \param voxelMaskPrefixSumsMB     A memory block containing the prefix sums for the voxel masks.
   * \param candidateVoxelLocationsMB A memory block into which to write the locations of the candidate voxels.
   */
  virtual void write_candidate_voxel_locations(const ITMFloat4Image *raycastResult,
                                               const ORUtils::MemoryBlock<unsigned char>& voxelMasksMB,
                                               const ORUtils::MemoryBlock<unsigned int>& voxelMaskPrefixSumsMB,
                                               ORUtils::MemoryBlock<Vector3s>& candidateVoxelLocationsMB) const = 0;

  /**
   * \brief Writes the locations of the sampled voxels into the sampled voxel locations memory block.
   *
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   */
  virtual void write_sampled_voxel_locations(ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Samples voxels from the current raycast result.
   *
   * \param raycastResult           The current raycast result.
   * \param scene                   The scene.
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   * \param voxelCountsForLabelsMB  A memory block into which to write the numbers of voxels sampled for each label.
   */
  void sample_voxels(const ITMFloat4Image *raycastResult, const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                     ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;
};

}

#endif
