/**
 * grove: PreemptiveRansac_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSACCUDA
#define H_GROVE_PREEMPTIVERANSACCUDA

#include "../interface/PreemptiveRansac.h"
#include "../../numbers/CUDARNG.h"

namespace grove {

/**
 * \brief An instance of this class allows the estimation of a 6DOF pose from a set of
 *        3D Keypoints and associated ScoreForest predictions, using the GPU.
 *
 *        This technique is based on the Preemptive-RANSAC algorithm, details can be found in:
 *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
 *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*, Julien Valentin,
 *        Luigi Di Stefano and Philip H. S. Torr
 *
 */
class PreemptiveRansac_CUDA : public PreemptiveRansac
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of pose candidates currently sampled. Resides in device memory. */
  ITMIntMemoryBlock_Ptr m_nbPoseCandidates_device;

  /** The number of currently sampled inliers. Resides in device memory. */
  ITMIntMemoryBlock_Ptr m_nbSampledInliers_device;

  /** The random number generators used during the P-RANSAC process. */
  CUDARNGMemoryBlock_Ptr m_randomGenerators;

  /** The seed used to initialise the random number generators. */
  uint32_t m_rngSeed;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of PreemptiveRansac_CUDA.
   *
   * \param settings A pointer to the settings used to configure the algorithm.
   */
  PreemptiveRansac_CUDA(const tvgutil::SettingsContainer_CPtr& settings);

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
   * \brief Initialises the random number generators in a deterministic manner.
   */
  void init_random();

  /** Override */
  virtual void update_host_pose_candidates() const;
};

}

#endif
