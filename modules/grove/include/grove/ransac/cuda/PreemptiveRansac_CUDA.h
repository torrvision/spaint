/**
 * grove: PreemptiveRansac_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSACCUDA
#define H_GROVE_PREEMPTIVERANSACCUDA

#include "../interface/PreemptiveRansac.h"
#include "../../numbers/CUDARNG.h"

namespace grove {

class PreemptiveRansac_CUDA: public PreemptiveRansac
{
public:
  PreemptiveRansac_CUDA();

protected:
  virtual void generate_pose_candidates();
  virtual void compute_and_sort_energies();
  virtual void sample_inlier_candidates(bool useMask);
  virtual void update_candidate_poses();

private:
  CUDARNGMemoryBlock_Ptr m_randomGenerators;
  uint32_t m_rngSeed;

  ITMIntImage_Ptr m_nbPoseCandidates_device;
  ITMIntImage_Ptr m_nbSampledInliers_device;

  void init_random();
};

}

#endif
