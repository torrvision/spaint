/**
 * grove: PreemptiveRansac_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSACCPU
#define H_GROVE_PREEMPTIVERANSACCPU

#include "../interface/PreemptiveRansac.h"
#include "../../numbers/CPURNG.h"

namespace grove {

/**
 * \brief An instance of this class allows the estimation of a 6DOF pose from a set of
 *        3D Keypoints and associated ScoreForest predictions, using the CPU.
 *
 *        This technique is based on the Preemptive-RANSAC algorithm, details can be found in:
 *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
 *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*, Julien Valentin,
 *        Luigi Di Stefano and Philip H. S. Torr
 */
class PreemptiveRansac_CPU : public PreemptiveRansac
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The random number generators used during the P-RANSAC process. */
  CPURNGMemoryBlock_Ptr m_rngs;

  /** The seed used to initialise the random number generators. */
  uint32_t m_rngSeed;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of PreemptiveRansac_CPU.
   *
   * \param settings A pointer to the settings used to configure the algorithm.
   */
  PreemptiveRansac_CPU(const tvgutil::SettingsContainer_CPtr& settings);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void compute_energies_and_sort();

  /** Override */
  virtual void generate_pose_candidates();

  /** Override */
  virtual void prepare_inliers_for_optimisation();

  /** Override */
  virtual void sample_inlier_candidates(bool useMask);

  /** Override */
  virtual void update_candidate_poses();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Compute the energy of a single pose candidate.
   *
   * \param candidate The pose candidate to evaluate.
   */
  void compute_pose_energy(PoseCandidate& candidate) const;

  /**
   * \brief Initialises the random number generators in a deterministic manner.
   */
  void init_random();
};

}

#endif
