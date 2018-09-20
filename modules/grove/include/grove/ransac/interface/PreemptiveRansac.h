/**
 * grove: PreemptiveRansac.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSAC
#define H_GROVE_PREEMPTIVERANSAC

#include <boost/optional.hpp>

#include <ORUtils/SE3Pose.h>

#include <orx/base/ORImagePtrTypes.h>
#include <orx/base/ORMemoryBlockPtrTypes.h>

#include <tvgutil/misc/SettingsContainer.h>
#include <tvgutil/timing/AverageTimer.h>

#include "../shared/PoseCandidate.h"
#include "../../keypoints/Keypoint3DColour.h"
#include "../../scoreforests/ScorePrediction.h"

#ifdef WITH_ALGLIB
 //#################### FORWARD DECLARATIONS ####################

// Note: We forward declare these to avoid including the ALGLIB header, which causes NVCC warnings when compiling PreemptiveRansac_CUDA.
namespace alglib {
  class real_1d_array;
  class real_2d_array;
}
#endif

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
   * \brief An instance of this struct holds the "inlier" points needed to compute the energies for candidate camera poses during Levenberg-Marquardt optimisation.
   */
  struct PointsForLM
  {
    /** The positions of the inlier points in camera space. */
    const Vector4f *cameraPoints;

    /** The modes predicted for the inlier points (one mode per point). */
    const Keypoint3DColourCluster *predictedModes;

    /** The number of inlier points (and modes). Comes last to avoid padding. */
    uint32_t nbPoints;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** Whether or not to print a summary of the timings of the various steps of preemptive RANSAC on destruction. */
  bool m_printTimers;

  /** The timer for the pose hypothesis generation phase. */
  AverageTimer m_timerCandidateGeneration;

  /** The timers for the energy computation phase. One per RANSAC iteration. */
  std::vector<AverageTimer> m_timerComputeEnergy;

  /** The timer for the first energy computation phase, before the RANSAC iterations. */
  AverageTimer m_timerFirstComputeEnergy;

  /** The timer for the first hypothesis culling. */
  AverageTimer m_timerFirstTrim;

  /** The timers for the inlier sampling phases. One per RANSAC iteration. */
  std::vector<AverageTimer> m_timerInlierSampling;

  /** The timers for the optimisation phase. One per RANSAC iteration. */
  std::vector<AverageTimer> m_timerOptimisation;

  /** The timers for the optimisation preparation phase. One per RANSAC iteration. */
  std::vector<AverageTimer> m_timerPrepareOptimisation;

  /** The timer for the entire preemptive RANSAC process. */
  AverageTimer m_timerTotal;

  //#################### PROTECTED VARIABLES ####################
protected:
  /**
   * Whether or not to force the sampled modes to have a minimum distance between each other during the pose
   * hypothesis generation phase.
   */
  bool m_checkMinDistanceBetweenSampledModes;

  /** Whether or not to check for a rigid transformation when sampling modes during pose hypothesis generation. */
  bool m_checkRigidTransformationConstraint;

  /** A memory block that stores the raster indices of the candidate inliers already sampled from the input image. */
  ORIntMemoryBlock_Ptr m_inlierRasterIndicesBlock;

  /** A mask recording which inlier points have already been sampled. */
  ORIntImage_Ptr m_inliersMaskImage;

  /** An image storing the keypoints extracted from the input image during relocalisation. Not owned by this class. */
  Keypoint3DColourImage_CPtr m_keypointsImage;

  /** The maximum number of iterations for which to attempt to generate a pose candidate. */
  uint32_t m_maxCandidateGenerationIterations;

  /** The maximum number of pose candidates to generate. */
  uint32_t m_maxPoseCandidates;

  /** Aggressively cull the initial number of pose candidates to this, keeping only the best ones. */
  uint32_t m_maxPoseCandidatesAfterCull;

  /**
   * The maximum allowed difference between distances in camera space and world space when generating pose hypotheses
   * (if m_checkRigidTransformationConstraint is enabled).
   */
  float m_maxTranslationErrorForCorrectPose;

  /** The minimum distance (squared) between sampled modes (if m_checkMinDistanceBetweenSampledModes is enabled). */
  float m_minSquaredDistanceBetweenSampledModes;

  /**
   * The maximum number of points that will be used as inliers during the preemptive RANSAC phase.
   * The actual number of inliers in use starts from m_ransacInliersPerIteration and increases by
   * m_ransacInliersPerIteration on each iteration of preemptive RANSAC.
   */
  size_t m_nbMaxInliers;

  /** A memory block storing the pose candidates. */
  PoseCandidateMemoryBlock_Ptr m_poseCandidates;

  /** The number of pose candidates that survived the culling process. */
  uint32_t m_poseCandidatesAfterCull;

  /** The camera points used for the pose optimisation step. Each row represents the points for a pose candidate. */
  ORFloat4MemoryBlock_Ptr m_poseOptimisationCameraPoints;

  /** The energy value that, if reached, will cause the pose optimisation (which is trying to decrease this value) to terminate. */
  double m_poseOptimisationEnergyThreshold;

  /** The value of the gradient norm that, if reached, will cause the pose optimisation to terminate. */
  double m_poseOptimisationGradientThreshold;

  /**
   * The maximum distance there can be between the estimated world coordinates of a point and its predicted mode
   * for it to be considered as an inlier during the pose optimisation step.
   */
  float m_poseOptimisationInlierThreshold;

  /** The maximum number of Levenberg-Marquardt iterations to perform during pose optimisation. */
  uint32_t m_poseOptimisationMaxIterations;

  /** The modes used for the pose optimisation step. Each row represents the modes for a pose candidate. */
  Keypoint3DColourClusterMemoryBlock_Ptr m_poseOptimisationPredictedModes;

  /** The value of the step norm that, if reached, will cause the pose optimisation to terminate. */
  double m_poseOptimisationStepThreshold;

  /** Whether or not to optimise the surviving poses after each preemptive RANSAC iteration. */
  bool m_poseUpdate;

  /** An image storing the forest predictions associated with the keypoints in m_keypointsImage. Not owned by this class. */
  ScorePredictionsImage_CPtr m_predictionsImage;

  /** The number of points to add to the inlier set after each preemptive RANSAC iteration. */
  uint32_t m_ransacInliersPerIteration;

  /** The settings used to configure the algorithm. */
  tvgutil::SettingsContainer_CPtr m_settings;

  /** Whether or not to use every modal cluster in the leaves when generating pose hypotheses. */
  bool m_useAllModesPerLeafInPoseHypothesisGeneration;

  /** Whether or not to use Mahalanobis (rather than L2) distances to compute the energies during the pose optimisation step. */
  bool m_usePredictionCovarianceForPoseOptimization;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an instance of PreemptiveRansac.
   *
   * \param settings The settings used to configure the algorithm.
   */
  PreemptiveRansac(const tvgutil::SettingsContainer_CPtr& settings);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the instance of PreemptiveRansac.
   */
  virtual ~PreemptiveRansac();

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Computes the energy associated with each remaining pose candidate and reranks them in non-decreasing energy order.
   */
  virtual void compute_energies_and_sort() = 0;

  /**
   * \brief Generates a certain number of camera pose hypotheses using the method described in the paper.
   */
  virtual void generate_pose_candidates() = 0;

  /**
   * \brief Prepares the inliers' positions in camera space and modes for use during pose optimisation.
   */
  virtual void prepare_inliers_for_optimisation() = 0;

  /**
   * \brief Samples a certain number of keypoints from the input image.
   *
   * The sampled keypoints will be used for the subsequent energy computation.
   *
   * \param useMask Whether or not to record the sampled keypoints in a persistent mask (to prevent them being sampled twice).
   */
  virtual void sample_inliers(bool useMask = false) = 0;

  /**
   * \brief Perform the continuous optimisation step described in the paper to update each remaining pose candidate.
   */
  virtual void update_candidate_poses() = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to estimate a 6DOF pose from a set of 3D keypoints and their associated SCoRe forest predictions using a preemptive RANSAC approach.
   *
   * \param keypointsImage    An image containing 3D keypoints computed from an RGB-D input image pair.
   * \param predictionsImage  An image containing SCoRe forest predictions for each keypoint in the keypoints image.
   * \return                  An estimated pose, if possible, or boost::none otherwise.
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
   * \brief Gets the minimum number of points that have to be valid for the algorithm to attempt pose estimation.
   *
   * \return The minimum number of points that have to be valid for the algorithm to attempt pose estimation.
   */
  uint32_t get_min_nb_required_points() const;

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Runs the Kabsch algorithm on the three camera/world point correspondences of each generated pose candidate
   *        to obtain an estimate of the camera pose (a rigid transformation matrix from camera space to world space).
   *
   * \note  This will probably go away as soon as we implement a proper SVD solver that can run on both the CPU and GPU.
   */
  void compute_candidate_poses_kabsch();

  /**
   * \brief Resets the inliers that are used to evaluate camera pose candidates.
   *
   * \param resetMask Whether or not to also reset the inliers mask.
   */
  virtual void reset_inliers(bool resetMask);

  /**
   * \brief Attempts to update the pose of the specified candidate by minimising a non-linear energy using Levenberg-Marquardt.
   *
   * \note  This is currently done on the CPU, although the plan is ultimately to reimplement it as shared code.
   *
   * \param candidateIdx  The index of the candidate whose pose we want to optimise.
   * \return              true, if the optimisation succeeded, or false otherwise.
   */
  bool update_candidate_pose(int candidateIdx) const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Makes sure that the host version of the pose candidates memory block contains up-to-date values.
   */
  virtual void update_host_pose_candidates() const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:

#ifdef WITH_ALGLIB
  /**
   * \brief A function that ALGLIB can call to compute the energy of the candidate camera pose.
   *
   * \note  Calls compute_energy_l2 internally.
   *
   * \param xi  The 6D twist vector corresponding to the candidate camera pose being optimised.
   * \param phi A location into which to store the computed energy value.
   * \param pts The points to use for the energy computation.
   */
  static void alglib_func_l2(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, void *pts);

  /**
   * \brief A function that ALGLIB can call to compute the energy of the candidate camera pose.
   *
   * \note  Calls compute_energy_mahalanobis internally.
   *
   * \param xi  The 6D twist vector corresponding to the candidate camera pose being optimised.
   * \param phi A location into which to store the computed energy value.
   * \param pts The points to use for the energy computation.
   */
  static void alglib_func_mahalanobis(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, void *pts);

  /**
   * \brief A function that ALGLIB can call to compute the energy and Jacobian of the candidate camera pose.
   *
   * \note  Calls compute_energy_l2 internally.
   *
   * \param xi  The 6D twist vector corresponding to the candidate camera pose being optimised.
   * \param phi A location into which to store the computed energy value.
   * \param jac A location into which to store the computed Jacobian.
   * \param pts The points to use for the energy and Jacobian computations.
   */
  static void alglib_jac_l2(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, alglib::real_2d_array& jac, void *pts);

  /**
   * \brief A function that ALGLIB can call to compute the energy and Jacobian of the candidate camera pose.
   *
   * \note  Calls compute_energy_mahalanobis internally.
   *
   * \param xi  The 6D twist vector corresponding to the candidate camera pose being optimised.
   * \param phi A location into which to store the computed energy value.
   * \param jac A location into which to store the computed Jacobian.
   * \param pts The points to use for the energy and Jacobian computations.
   */
  static void alglib_jac_mahalanobis(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, alglib::real_2d_array& jac, void *pts);

  /**
   * \brief A function that is called by ALGLIB after each iteration of the optimisation.
   *
   * \note  This currently does nothing, but could be used for debugging.
   *
   * \param xi  The 6D twist vector corresponding to the candidate camera pose being optimised.
   * \param phi The energy value computed for the pose during the most recent iteration of the optimisation.
   * \param pts The points that were used for the energy computation.
   */
  static void alglib_rep(const alglib::real_1d_array& xi, double phi, void *pts);
#endif

  /**
   * \brief Computes an energy for the specified candidate camera pose based on L2 error terms for a set of points.
   *
   * \param candidateCameraPose The candidate camera pose.
   * \param pts                 The points.
   * \param jac                 An optional location in which to store the Jacobian of the 6D twist vector corresponding to the pose.
   */
  static double compute_energy_l2(const ORUtils::SE3Pose& candidateCameraPose, const PointsForLM& pts, double *jac = NULL);

  /**
   * \brief Computes an energy for the specified candidate camera pose based on Mahalanobis error terms for a set of points.
   *
   * \param candidateCameraPose The candidate camera pose.
   * \param pts                 The points.
   * \param jac                 An optional location in which to store the Jacobian of the 6D twist vector corresponding to the pose.
   */
  static double compute_energy_mahalanobis(const ORUtils::SE3Pose& candidateCameraPose, const PointsForLM& pts, double *jac = NULL);

#ifdef WITH_ALGLIB
  /**
   * \brief Makes an SE3 pose that corresponds to the specified 6D twist vector.
   *
   * \param xi  The 6D twist vector.
   * \return    The corresponding SE3 pose.
   */
  static ORUtils::SE3Pose make_pose_from_twist(const alglib::real_1d_array& xi);

  /**
   * \brief Makes a 6D twist vector that corresponds to the specified SE3 pose.
   *
   * \param pose  The SE3 pose.
   * \return      The corresponding 6D twist vector.
   */
  static alglib::real_1d_array make_twist_from_pose(const ORUtils::SE3Pose& pose);
#endif

  /**
   * \brief Pretty prints the value of a timer.
   *
   * \param timer The timer.
   */
  static void print_timer(const AverageTimer& timer);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<PreemptiveRansac> PreemptiveRansac_Ptr;
typedef boost::shared_ptr<const PreemptiveRansac> PreemptiveRansac_CPtr;

}

#endif
