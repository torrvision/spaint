/**
 * grove: PreemptiveRansac_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSACCPU
#define H_GROVE_PREEMPTIVERANSACCPU

#include "../interface/PreemptiveRansac.h"

#include <tvgutil/numbers/SimpleRandomNumberGenerator_CPU.h>

namespace grove {

class PreemptiveRansac_CPU : public PreemptiveRansac
{
public:
  PreemptiveRansac_CPU();

protected:
  virtual void generate_pose_candidates();
  virtual void compute_and_sort_energies();
  virtual void sample_inlier_candidates(bool useMask);
  virtual void update_candidate_poses();

private:
  tvgutil::CPURNGMemoryBlock_Ptr m_randomGenerators;
  uint32_t m_rngSeed;

  void init_random();
  void compute_pose_energy(PoseCandidate &candidate) const;
};

}

#endif
