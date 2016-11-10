/**
 * spaint: GPURansac_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPURANSACCUDA
#define H_SPAINT_GPURANSACCUDA

#include "randomforest/interface/GPURansac.h"

#include "tvgutil/numbers/SimpleRandomNumberGenerator_CUDA.h"

namespace spaint {
class GPURansac_CUDA : public GPURansac
{
public:
  GPURansac_CUDA();

protected:
  virtual void generate_pose_candidates();
  virtual void compute_and_sort_energies();

private:
  tvgutil::CUDARNGMemoryBlock_Ptr m_randomGenerators;
  uint32_t m_rngSeed;

  ITMIntImage_Ptr m_nbPoseCandidates_device;

  void init_random();
  void compute_candidate_pose_kabsch();
};

}

#endif
