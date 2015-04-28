/**
 * spaint: VoxelSampler.h
 */

#ifndef H_SPAINT_VOXELSAMPLER
#define H_SPAINT_VOXELSAMPLER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/LabelManager.h"

namespace tvgutil {

//#################### FORWARD DECLARATIONS ####################

class RandomNumberGenerator;

}

namespace spaint {

/**
 * \brief An instance of a class deriving from this one can be used to sample voxels from a scene.
 */
class VoxelSampler
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** A memory block in which to store random indices when sampling from the candidate voxels for each class. */
  mutable ORUtils::MemoryBlock<int> m_candidateVoxelIndicesMB;

  /** A memory block in which to store the locations of candidate voxels in the raycast result, grouped by semantic label. */
  mutable ORUtils::MemoryBlock<Vector3s> m_candidateVoxelLocationsMB;

  /** The label manager. */
  LabelManager_CPtr m_labelManager;

  /** The maximum number of labels that can be in use. */
  const int m_maxLabelCount;

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
   * \param labelManager      The label manager.
   * \param maxVoxelsPerLabel The maximum number of voxels to sample for each label.
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   */
  VoxelSampler(const LabelManager_CPtr& labelManager, int maxVoxelsPerLabel, int raycastResultSize, unsigned int seed);

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
   * \param labelMaskMB A memory block containing a mask specifying which labels are currently in use.
   */
  virtual void calculate_voxel_mask_prefix_sums(const ORUtils::MemoryBlock<bool>& labelMaskMB) const = 0;

  /**
   * \brief Calculates the voxel masks.
   *
   * \param raycastResult The current raycast result.
   * \param voxelData     The scene's voxel data.
   * \param indexData     The scene's index data.
   */
  virtual void calculate_voxel_masks(const ITMFloat4Image *raycastResult, const SpaintVoxel *voxelData, const ITMVoxelIndex::IndexData *indexData) const = 0;

  /**
   * \brief Writes the number of candidate voxels that are available for each label into the voxel counts for labels memory block.
   *
   * \param labelMaskMB             A memory block containing a mask specifying which labels are currently in use.
   * \param voxelCountsForLabelsMB  A memory block into which to write voxel counts for each label.
   */
  virtual void write_candidate_voxel_counts(const ORUtils::MemoryBlock<bool>& labelMaskMB, ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const = 0;

  /**
   * \brief Writes the locations of the candidate voxels into the candidate voxel locations memory block.
   *
   * \param raycastResult The current raycast result.
   */
  virtual void write_candidate_voxel_locations(const ITMFloat4Image *raycastResult) const = 0;

  /**
   * \brief Writes the locations of the sampled voxels into the sampled voxel locations memory block.
   *
   * \param labelMaskMB             A memory block containing a mask specifying which labels are currently in use.
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   */
  virtual void write_sampled_voxel_locations(const ORUtils::MemoryBlock<bool>& labelMaskMB, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Samples voxels from the current raycast result.
   *
   * \param raycastResult           The current raycast result.
   * \param scene                   The scene.
   * \param labelMaskMB             A memory block containing a mask specifying which labels are currently in use.
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   * \param voxelCountsForLabelsMB  A memory block into which to write the numbers of voxels sampled for each label.
   */
  void sample_voxels(const ITMFloat4Image *raycastResult,
                     const ITMLib::Objects::ITMScene<SpaintVoxel,ITMVoxelIndex> *scene,
                     const ORUtils::MemoryBlock<bool>& labelMaskMB,
                     ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB,
                     ORUtils::MemoryBlock<unsigned int>& voxelCountsForLabelsMB) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const VoxelSampler> VoxelSampler_CPtr;

}

#endif
