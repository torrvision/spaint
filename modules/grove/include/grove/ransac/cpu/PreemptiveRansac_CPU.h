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
 *
 */
class PreemptiveRansac_CPU : public PreemptiveRansac
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of PreemptiveRansac_CUDA.
   */
  PreemptiveRansac_CPU();

  //#################### PROTECTED VIRTUAL MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Compute the energy associated to each remaining pose hypothesis ans rerank them by increasing energy.
   */
  virtual void compute_and_sort_energies();

  /**
   * \brief Generate a certain number of camera pose hypotheses according to the method described in the paper.
   */
  virtual void generate_pose_candidates();

  /**
   * \brief Sample a certain number of keypoints from the input image. Those keypoints will be used for the subsequent
   *        energy computation.
   *
   * \param useMask Whether or not to store in a persistent mask the location of already sampled keypoints.
   */
  virtual void sample_inlier_candidates(bool useMask = false);

  /**
   * \brief Perform the continuous optimisation step described in the paper to update each remaining pose hypothesis.
   */
  virtual void update_candidate_poses();

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The random number generators used during the P-RANSAC process. */
  CPURNGMemoryBlock_Ptr m_randomGenerators;

  /** The seed used to initialise the random number generators. */
  uint32_t m_rngSeed;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Compute the energy of a single pose hypothesis.
   *
   * \param candidate The pose hypothesis to evaluate.
   */
  void compute_pose_energy(PoseCandidate &candidate) const;

  /**
   * \brief Initialises the random number generators in a deterministic manner.
   */
  void init_random();
};

} // namespace grove

#endif // H_GROVE_PREEMPTIVERANSACCPU
