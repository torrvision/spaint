/**
 * grove: PreemptiveRansac.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSAC
#define H_GROVE_PREEMPTIVERANSAC

#include <vector>

#include <alglib/optimization.h>

#include <boost/optional.hpp>

#include <ORUtils/SE3Pose.h>

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMMemoryBlockPtrTypes.h>

#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/AverageTimer.h>

#include "../shared/PoseCandidate.h"
#include "../../keypoints/Keypoint3DColour.h"
#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one allows the estimation of a 6DOF pose from a set of
 *        3D keypoints and their associated SCoRe forest predictions.
 *
 * The technique used is based on preemptive RANSAC, as described in:
 *
 * "On-the-Fly Adaptation of Regression Forests for Online Camera Relocalisation" (Cavallari et al., CVPR 2017)
 *
 */
class PreemptiveRansac
{
  //#################### TYPEDEFS ####################
public:
  typedef tvgutil::AverageTimer<boost::chrono::nanoseconds> AverageTimer;

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief This struct is used to hold pointers to the data used when computing the residual energy.
   */
  struct PointsForLM
  {
    const Vector4f *cameraPoints;
    const Keypoint3DColourCluster *predictedModes;
    uint32_t nbPoints; // Comes last to avoid padding.
  };

  //#################### PRIVATE VARIABLES ####################
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

  /** The timers for the optimisation preparation phase. One per each RANSAC iteration. */
  std::vector<AverageTimer> m_timerPrepareOptimisation;

  /** The timer for the entire P-RANSAC process. */
  AverageTimer m_timerTotal;

  //#################### PROTECTED VARIABLES ####################
protected:
  /**
   * Whether or not to force the sampling of modes having a minimum distance between each other during the pose
   * hypothesis generation phase.
   */
  bool m_checkMinDistanceBetweenSampledModes;

  /** Whether or not to check for a rigid transformation when sampling modes for the pose hypothesis generation. */
  bool m_checkRigidTransformationConstraint;

  /** A memory block that stores the raster indices of the candidate inliers already sampled from the input image. */
  ITMIntMemoryBlock_Ptr m_inlierRasterIndicesBlock;

  /** An image representing a mask for the already sampled inlier points. */
  ITMIntImage_Ptr m_inliersMaskImage;

  /** An image storing the keypoints extracted from the input image during the relocalisation. Not owned by this class. */
  Keypoint3DColourImage_CPtr m_keypointsImage;

  /** The maximum number of attempts for the generation of a pose candidate. */
  uint32_t m_maxCandidateGenerationIterations;

  /** The initial number of pose hypotheses to generate */
  uint32_t m_maxPoseCandidates;

  /** Aggressively cull the initial number of pose hypotheses to this amount, keeping only the best ones. */
  uint32_t m_maxPoseCandidatesAfterCull;

  /**
   * The maximum allowed difference between distances in camera frame and world frame when generating pose hypotheses
   * if m_checkRigidTransformationConstraint is enabled.
   */
  float m_maxTranslationErrorForCorrectPose;

  /** The minimum distance (squared) between sampled modal clusters when m_checkMinDistanceBetweenSampledModes is enabled. */
  float m_minSquaredDistanceBetweenSampledModes;

  /**
   * The maximum number of points that are tested as inliers during the P-RANSAC phase.
   * The actual number starts from m_ransacInliersPerIteration and increases by m_ransacInliersPerIteration each P-RANSAC iteration.
   */
  size_t m_nbMaxInliers;

  /** A memory block storing the pose hypotheses. */
  PoseCandidateMemoryBlock_Ptr m_poseCandidates;

  /** The actual number of pose candidates that survived the culling process. */
  uint32_t m_poseCandidatesAfterCull;

  /** The camera points used for the pose optimisation step. Each row represents the points for a pose candidate. */
  ITMFloat4MemoryBlock_Ptr m_poseOptimisationCameraPoints;

  /** The minimum value that has to be reached by the energy function during pose optimisation to terminate. */
  double m_poseOptimisationEnergyThreshold;

  /** The minimum value that has to be reached by the gradient's norm during pose optimisation to terminate. */
  double m_poseOptimisationGradientThreshold;

  /**
   * The maximum distance between the estimated world coordinates of a point and its predicted mode to be considered as
   * inlier during the pose optimisation step.
   */
  float m_poseOptimisationInlierThreshold;

  /** The maximum number of Levenberg-Marquardt iterations to perform during pose optimisation. */
  uint32_t m_poseOptimisationMaxIterations;

  /** The modes used for the pose optimisation step. Each row represents the modes for a pose candidate. */
  Keypoint3DColourClusterMemoryBlock_Ptr m_poseOptimisationPredictedModes;

  /** The minimum value that has to be reached by the step's norm during pose optimisation to terminate. */
  double m_poseOptimisationStepThreshold;

  /** Whether or not to optimise the surviving poses after each P-RANSAC iteration. */
  bool m_poseUpdate;

  /** An image storing the forest predictions associated to the keypoints in m_keypointsImage. Not owned by this class. */
  ScorePredictionsImage_CPtr m_predictionsImage;

  /** The number of points to add to the inlier set after each P-RANSAC iteration. */
  uint32_t m_ransacInliersPerIteration;

  /** The settings. */
  tvgutil::SettingsContainer_CPtr m_settings;

  /** Whether or not to use every modal cluster in the leaves when generating pose hypotheses. */
  bool m_useAllModesPerLeafInPoseHypothesisGeneration;

  /**
   * Whether to use the Mahalanobis distance to measure the energy during the pose optimisation step. If false uses the
   * L2 distance between points.
   */
  bool m_usePredictionCovarianceForPoseOptimization;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an instance of PreemptiveRansac.
   *
   * \param settings A pointer to the settings used to configure the algorithm.
   */
  PreemptiveRansac(const tvgutil::SettingsContainer_CPtr& settings);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys an instance of PreemptiveRansac.
   */
  virtual ~PreemptiveRansac();

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Compute the energy associated to each remaining pose hypothesis and rerank them by increasing energy.
   */
  virtual void compute_energies_and_sort() = 0;

  /**
   * \brief Generate a certain number of camera pose hypotheses according to the method described in the paper.
   */
  virtual void generate_pose_candidates() = 0;

  /**
   * \brief Compute the inlier positions in camera space and their associated modes to use during the pose optimisation step.
   */
  virtual void prepare_inliers_for_optimisation() = 0;

  /**
   * \brief Sample a certain number of keypoints from the input image. Those keypoints will be used for the subsequent
   *        energy computation.
   *
   * \param useMask Whether or not to store in a persistent mask the location of already sampled keypoints.
   */
  virtual void sample_inliers(bool useMask = false) = 0;

  /**
   * \brief Perform the continuous optimisation step described in the paper to update each remaining pose hypothesis.
   */
  virtual void update_candidate_poses() = 0;

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
   * \param keypointsImage    An image representing 3D keypoints computed from an RGB-D input image pair.
   * \param predictionsImage  An image storing ScoreForest predictions for each keypoint in keypoints.
   *
   * \return The estimated pose if successful, an empty optional value otherwise.
   */
  boost::optional<PoseCandidate> estimate_pose(const Keypoint3DColourImage_CPtr& keypointsImage, const ScorePredictionsImage_CPtr& predictionsImage);

  /**
   * \brief Gets all of the candidate poses that survived the initial culling process, sorted in non-increasing order
   *        of the number of P-RANSAC iterations they survived.
   *
   * \pre   This function should only be called after a prior call to estimate_pose.
   * \note  The first entry of the vector will be the candidate (if any) returned by estimate_pose.
   *
   * \param poseCandidates An output array that will be filled with the candidate poses that survived the initial culling process.
   */
  void get_best_poses(std::vector<PoseCandidate>& poseCandidates) const;

  /**
   * \brief Gets the minimum number of points that have to be valid for the algorithm to attempt the pose estimation.
   *
   * \return The minimum number of points that have to be valid in order to attempt pose estimation.
   */
  int get_min_nb_required_points() const;

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Runs the Kabsch algorithm on the three camera/world point correspondences of each generated pose hypothesis
   *        to obtain an estimate of the camera pose (a rigid transformation matrix from camera space to world space).
   *
   * \note  This will probably go away as soon as we implement a proper SVD solver that can run on both the CPU and GPU.
   */
  void compute_candidate_poses_kabsch();

  /**
   * \brief Optimise a PoseCandidate pose minimising a non-linear error term depending on the parameters of the class.
   *        See the paper for details.
   *
   * \note  Will probably go away as soon as we implement the optimisation as shared code that can run on both CPU and
   *        GPU.
   *
   * \param candidateIdx The index of the pose candidate to optimise.
   *
   * \return Whether the optimisation succeeded or not.
   */
  bool update_candidate_pose(int candidateIdx) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Make sure that the host version of the pose candidates memory block contains up to date values.
   */
  virtual void update_host_pose_candidates() const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Alglib's diagnostic function. Currently does nothing, but could print stuff.
   */
  static void call_after_each_step(const alglib::real_1d_array& x, double func, void *ptr);

  /**
   * \brief Function that will be called by alglib's optimiser.
   */
  static void Continuous3DOptimizationUsingFullCovariance(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, void *ptr);

  /**
   * \brief Function that will be called by alglib's optimiser (analytic jacobians variant).
   */
  static void Continuous3DOptimizationUsingFullCovarianceJac(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void *ptr);

  /**
   * \brief Function that will be called by alglib's optimiser.
   */
  static void Continuous3DOptimizationUsingL2(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, void *ptr);

  /**
   * \brief Function that will be called by alglib's optimiser (analytic jacobians variant).
   */
  static void Continuous3DOptimizationUsingL2Jac(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void *ptr);

  /**
   * \brief Compute the energy using the Mahalanobis distance.
   */
  static double EnergyForContinuous3DOptimizationUsingFullCovariance(const PointsForLM& pts, const ORUtils::SE3Pose& candidateCameraPose, double *jac = NULL);

  /**
   * \brief Compute the energy using the L2 distance between the points.
   */
  static double EnergyForContinuous3DOptimizationUsingL2(const PointsForLM& pts, const ORUtils::SE3Pose& candidateCameraPose, double *jac = NULL);

  /**
   * \brief Pretty prints a timer value.
   *
   * \param timer The timer to print.
   */
  static void print_timer(const AverageTimer& timer);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<PreemptiveRansac> PreemptiveRansac_Ptr;
typedef boost::shared_ptr<const PreemptiveRansac> PreemptiveRansac_CPtr;

}

#endif
