/**
 * spaint: UniformVoxelSampler_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_UNIFORMVOXELSAMPLER_CUDA
#define H_SPAINT_UNIFORMVOXELSAMPLER_CUDA

#include "../interface/UniformVoxelSampler.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to uniformly sample voxels from a scene using CUDA.
 */
class UniformVoxelSampler_CUDA : public UniformVoxelSampler
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based uniform voxel sampler.
   *
   * \param raycastResultSize The size of the raycast result (in pixels).
   * \param seed              The seed for the random number generator.
   */
  UniformVoxelSampler_CUDA(int raycastResultSize, unsigned int seed);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void write_sampled_voxel_locations(const ITMFloat4Image *raycastResult, size_t sampledVoxelCount, ORUtils::MemoryBlock<Vector3s>& sampledVoxelLocationsMB) const;
};

}

#endif
