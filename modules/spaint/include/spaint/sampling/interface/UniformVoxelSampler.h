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
 * \brief An instance of a class deriving from this one can be used to uniformly sample voxels from a scene.
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
  boost::shared_ptr<ORUtils::MemoryBlock<int> > m_sampledVoxelIndicesMB;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a uniform voxel sampler.
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
   * \brief Writes the locations of the voxels to be sampled from the raycast result into the sampled voxel locations memory block.
   *
   * \param raycastResult           The current raycast result.
   * \param sampledVoxelCount       The number of sampled voxels.
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   */
  virtual void write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t sampledVoxelCount, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Uniformly samples the specified number of voxels from the current raycast result.
   *
   * Note that this sampler does not guarantee that the voxels it produces are valid, so client code must account for this.
   *
   * \param raycastResult           The current raycast result.
   * \param numVoxelsToSample       The number of voxels to sample.
   * \param sampledVoxelLocationsMB A memory block into which to write the locations of the sampled voxels.
   */
  void sample_voxels(const ITMFloat4Image *raycastResult, size_t numVoxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<const UniformVoxelSampler> UniformVoxelSampler_CPtr;

}

#endif
