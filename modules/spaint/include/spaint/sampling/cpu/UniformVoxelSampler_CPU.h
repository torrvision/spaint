/**
 * spaint: UniformVoxelSampler_CPU.h
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER_CPU
#define H_SPAINT_UNIFORMVOXELSAMPLER_CPU

#include "../interface/UniformVoxelSampler.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to uniformly sample voxels from a scene using the CPU.
 */
class UniformVoxelSampler_CPU : public UniformVoxelSampler
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based uniform voxel sampler.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   */
  UniformVoxelSampler_CPU(int raycastResultSize, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t sampledVoxelCount, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

}

#endif
