/**
 * spaint: UniformVoxelSampler.h
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER
#define H_SPAINT_UNIFORMVOXELSAMPLER

#include <boost/shared_ptr.hpp>

#include <ITMLib/Objects/ITMScene.h>

#include "../../util/SpaintVoxel.h"

namespace tvgutil {

//#################### FORWARD DECLARATIONS ####################

class RandomNumberGenerator;

}

namespace spaint {

/**
 * \brief TODO
 */
class UniformVoxelSampler
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The size of the raycast result (in pixels). */
  const int m_raycastResultSize;

  /** A random number generator. */
  boost::shared_ptr<tvgutil::RandomNumberGenerator> m_rng;

  /** A memory block in which to store the indices of the voxels to be sampled from the raycast result. */
  mutable ORUtils::MemoryBlock<int> m_sampledVoxelIndicesMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief TODO
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   */
  UniformVoxelSampler(int raycastResultSize, unsigned int seed);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the voxel sampler.
   */
  virtual ~UniformVoxelSampler();

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  virtual void write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t voxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Samples voxels uniformly from the current raycast result.
   *
   * Note that this sampler does not guarantee that the voxels it produces are valid, so client code must account for this.
   *
   * \param raycastResult           The current raycast result.
   * \param voxelsToSample          The number of voxels to sample.
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   */
  void sample_voxels(const ITMFloat4Image *raycastResult, size_t voxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief TODO
   */
  void choose_voxels_to_sample(size_t maxVoxelsToSample) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const UniformVoxelSampler> UniformVoxelSampler_CPtr;

}

#endif
