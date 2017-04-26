/**
 * grove: PreemptiveRansac.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSAC
#define H_GROVE_PREEMPTIVERANSAC

#include <vector>

#include <boost/optional.hpp>

#include <itmx/ITMImagePtrTypes.h>
#include <itmx/ITMMemoryBlockPtrTypes.h>

#include <tvgutil/timing/AverageTimer.h>

#include "../../keypoints/Keypoint3DColour.h"
#include "../../ransac/base/PoseCandidate.h"
#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one allows the estimation of a 6DOF pose from a set of
 *        3D Keypoints and associated ScoreForest predictions.
 *
 *        This technique is based on the Preemptive-RANSAC algorithm, details can be found in:
 *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
 *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*, Julien Valentin,
 *        Luigi Di Stefano and Philip H. S. Torr
 *
 */
class PreemptiveRansac
{
  //#################### TYPEDEFS ####################
public:
  typedef tvgutil::AverageTimer<boost::chrono::nanoseconds> AverageTimer;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an instance of PreemptiveRansac.
   */
  PreemptiveRansac();

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys an instance of PreemptiveRansac.
   */
  virtual ~PreemptiveRansac();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Estimates a 6DOF pose from a set of 3D keypoints and their associated ScoreForest predictions
   *        using a Preemptive-RANSAC technique.
   *
   * \note  For details on the pose estimation process see:
   *        "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" by
   *        Tommaso Cavallari, Stuart Golodetz*, Nicholas A. Lord*, Julien Valentin,
   *        Luigi Di Stefano and Philip H. S. Torr
   *
   * \param keypoints         An image representing 3D keypoints computed from an RGB-D input image pair.
   * \param forestPredictions An image storing ScoreForest predictions for each keypoint in keypoints.
   *
   * \return The estimated pose if successful, an empty optional value otherwise.
   */
  boost::optional<PoseCandidate> estimate_pose(const Keypoint3DColourImage_CPtr &keypoints,
                                               const ScorePredictionsImage_CPtr &forestPredictions);

  /**
   * \brief Returns the best poses estimated by the P-RANSAC algorithm.
   *
   * \param poseCandidates Output array that will be filled with the best poses estimated by P-RANSAC.
   *                       Poses are sorted in descending quality order.
   */
  void get_best_poses(std::vector<PoseCandidate> &poseCandidates) const;

  /**
   * \brief Gets the minimum number of points that have to be valid for the algorithm to attempt the pose estimation.
   *
   * \return The minimum number of points that have to be valid in order to attempt pose estimation.
   */
  int get_min_nb_required_points() const;

  //#################### PROTECTED MEMBER VARIABLES ####################
protected:
  /** The number of points to add to the inlier set after each P-RANSAC iteration. */
  size_t m_batchSizeRansac;

  /**
   * Whether or not to force the sampling of modes having a minimum distance between each other during the pose
   * hyphotesis generation phase.
   */
  bool m_checkMinDistanceBetweenSampledModes;

  /** Whether or not to check for a rigid transformation when sampling modes for the pose hypothesis generation. */
  bool m_checkRigidTransformationConstraint;

  /** An array that stores the indices of the candidate inliers already sampled from the input image. */
  ITMIntMemoryBlock_Ptr m_inliersIndicesBlock;

  /** An image representing a mask for the already sampled inlier points. */
  ITMIntImage_Ptr m_inliersMaskImage;

  /** An image storing the keypoints extracted from the input image during the relocalisation. Not owned by this class.
   */
  Keypoint3DColourImage_CPtr m_keypointsImage;

  /**
   * The minimum distance (squared) between sampled modal clusters when m_checkMinDistanceBetweenSampledModes is
   * enabled.
   */
  float m_minSquaredDistanceBetweenSampledModes;

  /**
   * The maximum number of points that are tested as inliers during the P-RANSAC phase.
   * The actual number starts from m_batchSizeRansac and increases by m_batchSizeRansac each P-RANSAC iteration.
   */
  size_t m_nbMaxInliers;

  /** The initial number of pose hypotheses to generate */
  size_t m_nbMaxPoseCandidates;

  /** The number of points required to hypothesise a pose using the Kabsch algorithm. */
  size_t m_nbPointsForKabschBoostrap;

  /** A memory block storing the pose hypotheses. */
  PoseCandidateMemoryBlock_Ptr m_poseCandidates;

  /**
   * The maximum distance between the estimated world coordinates of a point and its predicted mode to be considered as
   * inlier during the pose optimisation step.
   */
  float m_poseOptimizationInlierThreshold;

  /** Whether or not to optimise the surviving poses after each P-RANSAC iteration. */
  bool m_poseUpdate;

  /** An image storing the forest predictions associated to the keypoints in m_keypointsImage. Not owned by this class.
   */
  ScorePredictionsImage_CPtr m_predictionsImage;

  /**
   * The maximum allowed difference between distances in camera frame and world frame when generating pose hypotheses
   * if m_checkRigidTransformationConstraint is enabled.
   */
  float m_translationErrorMaxForCorrectPose;

  /** Aggressively cull the initial number of pose hypotheses to this amount, keeping only the best ones. */
  size_t m_trimKinitAfterFirstEnergyComputation;

  /** Whether or not to use every modal cluster in the leaves when generating pose hypotheses. */
  bool m_useAllModesPerLeafInPoseHypothesisGeneration;

  /**
   * Whether to use the Mahalanobis distance to measure the energy during the pose optimisation step. If false uses the
   * L2 distance between points.
   */
  bool m_usePredictionCovarianceForPoseOptimization;

  //#################### PROTECTED VIRTUAL ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Compute the energy associated to each remaining pose hypothesis ans rerank them by increasing energy.
   */
  virtual void compute_and_sort_energies() = 0;

  /**
   * \brief Generate a certain number of camera pose hypotheses according to the method described in the paper.
   */
  virtual void generate_pose_candidates() = 0;

  /**
   * \brief Sample a certain number of keypoints from the input image. Those keypoints will be used for the subsequent
   *        energy computation.
   *
   * \param useMask Whether or not to store in a persistent mask the location of already sampled keypoints.
   */
  virtual void sample_inlier_candidates(bool useMask = false) = 0;

  /**
   * \brief Perform the continuous optimisation step described in the paper to update each remaining pose hypothesis.
   */
  virtual void update_candidate_poses() = 0;

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief For every generated pose hypothesis (3 pairs of points in camera/world coordinates) run the Kabsch algorithm
   *        and estimate the rigid transformation matrix.
   *
   * \note  Will probably go away as soon as we implement a proper SVD solver that can run on both CPU and GPU.
   */
  void compute_candidate_poses_kabsch();

  /**
   * \brief Optimise a PoseCandidate pose minimising a non-linear error term depending on the parameters of the class.
   *        See the paper for details.
   *
   * \note  Will probably go away as soon as we implement the optimisation as shared code that can run on both CPU and
   *        GPU.
   *
   * \param poseCandidate The pose candidate to optimise.
   *
   * \return Whether the optimisation succeeded or not.
   */
  bool update_candidate_pose(PoseCandidate &poseCandidate) const;

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** Whether to print a summary of the timings of the various steps of P-RANSAC on destruction. */
  bool m_printTimers;

  /** The timer for the pose hypothesis generation phase. */
  AverageTimer m_timerCandidateGeneration;

  /** The timers for the energy computation phase. One per each RANSAC iteration. */
  std::vector<AverageTimer> m_timerComputeEnergy;

  /** The timer for the first energy computation phase, before the RANSAC iterations. */
  AverageTimer m_timerFirstComputeEnergy;

  /** The timer for the first hypothesis culling. */
  AverageTimer m_timerFirstTrim;

  /** The timers for the inlier sampling phases. One per each RANSAC iteration. */
  std::vector<AverageTimer> m_timerInlierSampling;

  /** The timers for the optimisation phase. One per each RANSAC iteration. */
  std::vector<AverageTimer> m_timerOptimisation;

  /** The timer for the entire P-RANSAC process. */
  AverageTimer m_timerTotal;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<PreemptiveRansac> PreemptiveRansac_Ptr;
typedef boost::shared_ptr<const PreemptiveRansac> PreemptiveRansac_CPtr;

} // namespace grove

#endif // H_GROVE_PREEMPTIVERANSAC
