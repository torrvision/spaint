/**
 * grove: PreemptiveRansac_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSAC_CPU
#define H_GROVE_PREEMPTIVERANSAC_CPU

#include "../interface/PreemptiveRansac.h"
#include "../../numbers/CPURNG.h"

namespace grove {

/**
 * \brief An instance of this class allows the estimation of a 6DOF pose from a set of
 *        3D keypoints and their associated SCoRe forest predictions using the CPU.
 *
 * The technique used is based on preemptive RANSAC, as described in:
 *
 * "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" (Cavallari et al., CVPR 2017)
 *
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
   * \param settings           A pointer to the settings used to configure the algorithm.
   * \param settingsNamespace  The namespace used to read settings from the SettingsContainer.
   */
  PreemptiveRansac_CPU(const tvgutil::SettingsContainer_CPtr& settings, const std::string& settingsNamespace);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void compute_energies_and_sort();

  /** Override */
  virtual void generate_pose_candidates();

  /** Override */
  virtual void prepare_inliers_for_optimisation();

  /** Override */
  virtual void sample_inliers(bool useMask);

  /** Override */
  virtual void update_candidate_poses();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes the energy of a single pose candidate.
   *
   * \param candidate The pose candidate whose energy we want to compute.
   */
  void compute_pose_energy(PoseCandidate& candidate) const;

  /**
   * \brief Initialises the random number generators in a deterministic manner.
   */
  void init_random();
};

}

#endif
