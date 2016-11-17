/**
 * spaint: PreemptiveRansac_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PREEMPTIVERANSACCUDA
#define H_SPAINT_PREEMPTIVERANSACCUDA

#include "randomforest/interface/PreemptiveRansac.h"

#include "tvgutil/numbers/SimpleRandomNumberGenerator_CUDA.h"

namespace spaint
{
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
  tvgutil::CUDARNGMemoryBlock_Ptr m_randomGenerators;
  uint32_t m_rngSeed;

  ITMIntImage_Ptr m_nbPoseCandidates_device;
  ITMIntImage_Ptr m_nbSampledInliers_device;

  void init_random();
  void compute_candidate_pose_kabsch();
};

}

#endif
