/**
 * spaint: UniformVoxelSampler_CPU.h
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER_CPU
#define H_SPAINT_UNIFORMVOXELSAMPLER_CPU

#include "../interface/UniformVoxelSampler.h"

namespace spaint {

/**
 * \brief TODO
 */
class UniformVoxelSampler_CPU : public UniformVoxelSampler
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  UniformVoxelSampler_CPU(int raycastResultSize, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t voxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

}

#endif
