/**
 * spaint: UniformVoxelSampler_CUDA.h
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER_CUDA
#define H_SPAINT_UNIFORMVOXELSAMPLER_CUDA

#include "../interface/UniformVoxelSampler.h"

namespace spaint {

/**
 * \brief TODO
 */
class UniformVoxelSampler_CUDA : public UniformVoxelSampler
{
  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t voxelsToSample, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

}

#endif
