/**
 * spaint: GPURansac_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPURANSACCUDA
#define H_SPAINT_GPURANSACCUDA

#include "randomforest/interface/GPURansac.h"

#include <curand_kernel.h>

namespace spaint {
class GPURansac_CUDA : public GPURansac
{
public:
  typedef curandState_t RandomState;
  typedef ORUtils::MemoryBlock<RandomState> RandomStateMemoryBlock;
  typedef boost::shared_ptr<RandomStateMemoryBlock> RandomStateMemoryBlock_Ptr;
  typedef boost::shared_ptr<const RandomStateMemoryBlock> RandomStateMemoryBlock_CPtr;

  GPURansac_CUDA();

protected:
  virtual void generate_pose_candidates();
  virtual void compute_and_sort_energies();

private:
  RandomStateMemoryBlock_Ptr m_randomStates;
  uint32_t m_rngSeed;

  void init_random();
};

}

#endif
