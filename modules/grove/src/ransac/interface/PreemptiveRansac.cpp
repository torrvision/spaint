/**
 * grove: PreemptiveRansac.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/interface/PreemptiveRansac.h"
using namespace tvgutil;

#include <alglib/optimization.h>

#include <boost/lexical_cast.hpp>
#include <boost/timer/timer.hpp>

#include <Eigen/Dense>

#ifdef WITH_OPENMP
#include <omp.h>
#endif

#include <ORUtils/SE3Pose.h>

#include <itmx/base/MemoryBlockFactory.h>
#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

//#################### MACROS ####################

// Enable/disable the print-out of more detailed timings (very verbose, so disabled by default).
//#define ENABLE_TIMERS

namespace grove {

//#################### CONSTRUCTORS ####################

PreemptiveRansac::PreemptiveRansac(const SettingsContainer_CPtr& settings)
: m_timerCandidateGeneration("Candidate Generation"),
  m_timerFirstComputeEnergy("First Energy Computation"),
  m_timerFirstTrim("First Trim"),
  m_timerTotal("P-RANSAC Total"),
  m_settings(settings)
{
  const std::string settingsNamespace = "PreemptiveRansac.";

  // By default, we set all parameters as in scoreforests.

  // Whether or not to force sampled modes to have a minimum distance between them.
  m_checkMinDistanceBetweenSampledModes = m_settings->get_first_value<bool>(settingsNamespace + "checkMinDistanceBetweenSampledModes", true);

  // Setting it to false speeds up a lot, at the expense of quality.
  m_checkRigidTransformationConstraint = m_settings->get_first_value<bool>(settingsNamespace + "checkRigidTransformationConstraint", true);

  // The maximum number of times we sample three pixel-mode pairs in the attempt to generate a pose candidate.
  m_maxCandidateGenerationIterations = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxCandidateGenerationIterations", 6000);

  // Number of initial pose candidates.
  m_maxPoseCandidates = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxPoseCandidates", 1024);

  // Aggressively cull hypotheses to this number.
  m_maxPoseCandidatesAfterCull = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxPoseCandidatesAfterCull", 64);

  // In m.
  m_maxTranslationErrorForCorrectPose = m_settings->get_first_value<float>(settingsNamespace + "maxTranslationErrorForCorrectPose", 0.05f);

  // In m.
  m_minSquaredDistanceBetweenSampledModes = m_settings->get_first_value<float>(settingsNamespace + "minSquaredDistanceBetweenSampledModes", 0.3f * 0.3f);

  // Optimisation parameters defaulted as in Valentin's paper.

  m_poseOptimisationEnergyThreshold = m_settings->get_first_value<double>(settingsNamespace + "poseOptimisationEnergyThreshold", 0.0);

  // Sets the termination condition for the pose optimisation.
  m_poseOptimisationGradientThreshold = m_settings->get_first_value<double>(settingsNamespace + "poseOptimisationGradientThreshold", 1e-6);

  // In m.
  m_poseOptimisationInlierThreshold = m_settings->get_first_value<float>(settingsNamespace + "poseOptimizationInlierThreshold", 0.2f);

  // Maximum number of LM iterations.
  m_poseOptimisationMaxIterations = m_settings->get_first_value<uint32_t>(settingsNamespace + "poseOptimisationMaxIterations", 100);

  m_poseOptimisationStepThreshold = m_settings->get_first_value<double>(settingsNamespace + "poseOptimisationStepThreshold", 0.0);

  // Whether or not to optimise the poses with LM.
  m_poseUpdate = m_settings->get_first_value<bool>(settingsNamespace + "poseUpdate", true);

  // Whether or not to print the timers for each phase.
  m_printTimers = m_settings->get_first_value<bool>(settingsNamespace + "printTimers", false);

  // The number of inliers sampled in each P-RANSAC iteration.
  m_ransacInliersPerIteration = m_settings->get_first_value<uint32_t>(settingsNamespace + "ransacInliersPerIteration", 500);

  // If false use the first mode only (representing the largest cluster).
  m_useAllModesPerLeafInPoseHypothesisGeneration = m_settings->get_first_value<bool>(settingsNamespace + "useAllModesPerLeafInPoseHypothesisGeneration", true);

  // If false use L2.
  m_usePredictionCovarianceForPoseOptimization = m_settings->get_first_value<bool>(settingsNamespace + "usePredictionCovarianceForPoseOptimization", true);

  // Each ransac iteration after the initial cull adds m_batchSizeRansac inliers to the set, so we allocate enough space for all.
  m_nbMaxInliers = m_ransacInliersPerIteration * static_cast<uint32_t>(std::ceil(log2(m_maxPoseCandidatesAfterCull)));

  const MemoryBlockFactory& mbf = MemoryBlockFactory::instance();

  // Allocate memory.
  m_inliersIndicesBlock = mbf.make_block<int>(m_nbMaxInliers);
  m_inliersMaskImage = mbf.make_image<int>();
  m_poseCandidates = mbf.make_block<PoseCandidate>(m_maxPoseCandidates);

  const uint32_t poseOptimisationBufferSize = m_nbMaxInliers * m_maxPoseCandidates;
  m_poseOptimisationCameraPoints = mbf.make_block<Vector4f>(poseOptimisationBufferSize);
  m_poseOptimisationPredictedModes = mbf.make_block<Keypoint3DColourCluster>(poseOptimisationBufferSize);

#ifdef ENABLE_TIMERS
  // Force the average timers to on as well if we want verbose printing.
  m_printTimers = true;
#endif

  // Setup the remaining timers.
  for(int i = 1; i <= 6; ++i)
  {
    m_timerInlierSampling.push_back(AverageTimer("Inlier Sampling " + boost::lexical_cast<std::string>(i)));
    m_timerPrepareOptimisation.push_back(AverageTimer("Prepare Optimisation " + boost::lexical_cast<std::string>(i)));
    m_timerOptimisation.push_back(AverageTimer("Optimisation " + boost::lexical_cast<std::string>(i)));
    m_timerComputeEnergy.push_back(AverageTimer("Energy Computation " + boost::lexical_cast<std::string>(i)));
  }
}

//#################### DESTRUCTOR ####################

PreemptiveRansac::~PreemptiveRansac()
{
  if(m_printTimers)
  {
    print_timer(m_timerTotal);
    print_timer(m_timerCandidateGeneration);
    print_timer(m_timerFirstTrim);
    print_timer(m_timerFirstComputeEnergy);

    for(size_t i = 0; i < m_timerInlierSampling.size(); ++i)
    {
      print_timer(m_timerInlierSampling[i]);
      print_timer(m_timerComputeEnergy[i]);
      print_timer(m_timerPrepareOptimisation[i]);
      print_timer(m_timerOptimisation[i]);
    }
  }
}

//#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################

// Default implementation of the abstract function.
void PreemptiveRansac::update_candidate_poses()
{
  const int nbPoseCandidates = static_cast<int>(m_poseCandidates->dataSize);

// Update every pose in parallel
#ifdef WITH_OPENMP
  #pragma omp parallel for schedule(dynamic)
#endif
  for(int i = 0; i < nbPoseCandidates; ++i)
  {
    update_candidate_pose(i);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<PoseCandidate> PreemptiveRansac::estimate_pose(const Keypoint3DColourImage_CPtr& keypoints, const ScorePredictionsImage_CPtr& forestPredictions)
{
  // NOTE: In this function and in the virtual functions of the CPU and CUDA subclasses we directly access and write
  // onto the dataSize of several MemoryBlock variables instead of keeping a separate "valid size" variable.
  // This is done on purpose since the Update*From* functions of MemoryBlock/Images only move the first "dataSize"
  // elements of the block. By changing the number to the actual number of elements to move we can gain a slight speed
  // up of the system.

  m_timerTotal.start();

  // Copy keypoints and predictions in the local variables, to avoid explicitely passing them to every function.
  m_keypointsImage = keypoints;
  m_predictionsImage = forestPredictions;

  // 1. Generate the pose hypotheses.
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6, "generating initial candidates: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_timerCandidateGeneration.start();
    generate_pose_candidates();
    m_timerCandidateGeneration.stop();
  }

  // Reset the number of inliers for the new pose estimation.
  m_inliersIndicesBlock->dataSize = 0;

  // 2. If we have to aggressively cull the initial hypotheses to a small number.
  if(m_maxPoseCandidatesAfterCull < m_poseCandidates->dataSize)
  {
    m_timerFirstTrim.start();
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6, "first trim: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    // 2a. First, sample a set of points from the input data to evaluate the hypotheses quality.
    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6, "sample inliers: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      sample_inlier_candidates(false); // no mask for the first pass
    }

    // 2b. Sort the hypotheses by descending quality.
    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6, "compute and sort energies: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      m_timerFirstComputeEnergy.start();
      compute_and_sort_energies();
      m_timerFirstComputeEnergy.stop();
    }

    // 2c. Keep only the best ones.
    m_poseCandidates->dataSize = m_maxPoseCandidatesAfterCull;

    m_timerFirstTrim.stop();
  }

#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6, "ransac: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

  // 3. Reset inlier mask (and inliers that might have been selected in a previous invocation of the method.)
  m_inliersMaskImage->ChangeDims(m_keypointsImage->noDims); // Happens only once, NOP afterwards.
  m_inliersMaskImage->Clear();                              // This and the following happen every time.
  m_inliersIndicesBlock->dataSize = 0;

  int iteration = 0;

  // 4. Main P-RANSAC loop, continue until only a single hypothesis remains.
  while(m_poseCandidates->dataSize > 1)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6, "ransac iteration: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    //    std::cout << candidates.size() << " camera remaining" << std::endl;

    // 4a. Sample a set of keypoints from the input image. Record that thay have been selected in the mask image, to
    // avoid selecting them again.
    m_timerInlierSampling[iteration].start();
    sample_inlier_candidates(true);
    m_timerInlierSampling[iteration].stop();

    // 4b. If the poseUpdate is enabled, optimise all the remaining hypotheses keeping into account the newly selected
    // inliers.
    if(m_poseUpdate)
    {
      m_timerPrepareOptimisation[iteration].start();
      prepare_inliers_for_optimisation();
      m_timerPrepareOptimisation[iteration].stop();

#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6, "continuous optimization: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      m_timerOptimisation[iteration].start();
      update_candidate_poses();
      m_timerOptimisation[iteration].stop();
    }

    // 4c. Compute the energy for each hypothesis and sort them by decreasing quality.
    m_timerComputeEnergy[iteration].start();
    compute_and_sort_energies();
    m_timerComputeEnergy[iteration].stop();

    // 4d. Remove half of the candidates with the worse energies.
    m_poseCandidates->dataSize /= 2;

    ++iteration;
  }

  // If we generated a single pose, the update step above wouldn't have been executed (zero iterations). Force its
  // execution.
  if(m_poseUpdate && iteration == 0 && m_poseCandidates->dataSize == 1)
  {
    // Sample some inliers.
    m_timerInlierSampling[iteration].start();
    sample_inlier_candidates(true);
    m_timerInlierSampling[iteration].stop();

    // Having selected the inlier points, find the best associated modes to use during optimisation.
    m_timerPrepareOptimisation[iteration].start();
    prepare_inliers_for_optimisation();
    m_timerPrepareOptimisation[iteration].stop();

    // Run optimisation.
    m_timerOptimisation[iteration].start();
    update_candidate_poses();
    m_timerOptimisation[iteration].stop();
  }

  m_timerTotal.stop();

  // Make sure the pose candidates available on the host are up to date.
  update_host_pose_candidates();

  // 5. If we have been able to generate at least one candidate hypothesis, return the best one.
  PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  return m_poseCandidates->dataSize > 0 ? candidates[0] : boost::optional<PoseCandidate>();
}

void PreemptiveRansac::get_best_poses(std::vector<PoseCandidate>& poseCandidates) const
{
  // No need to check dataSize since it will likely be 1 after running estimate_pose. If estimate_pose has never run
  // this will probably return garbage.
  const PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

  // Setup output container.
  poseCandidates.clear();
  poseCandidates.reserve(m_maxPoseCandidatesAfterCull);

  // Copy the all the poses that survived after the initial cull. They are "ordered in blocks":
  // the first one is the one returned by estimate_pose, the second is the one removed after the last ransac iteration,
  // the third and fourth are removed in the iteration before (whilst they are not in a specific order, they are worse
  // than those in position 0 and 1), and so on...
  for(uint32_t poseIdx = 0; poseIdx < m_maxPoseCandidatesAfterCull; ++poseIdx)
  {
    poseCandidates.push_back(candidates[poseIdx]);
  }
}

int PreemptiveRansac::get_min_nb_required_points() const
{
  // At least the number of inliers required for a RANSAC iteration.
  return m_ransacInliersPerIteration;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void PreemptiveRansac::compute_candidate_poses_kabsch()
{
  // The assumption is that the data on the CPU memory is up to date. It's the task of the CUDA subclass to make sure of
  // it. This function will probably go away as soon as we implement a shared SVD solver.
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

//  std::cout << "Generated " << nbPoseCandidates << " candidates." << std::endl;

// Process all candidates in parallel.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(size_t candidateIdx = 0; candidateIdx < nbPoseCandidates; ++candidateIdx)
  {
    PoseCandidate& candidate = poseCandidates[candidateIdx];

    // We have to copy the points from the ORUtil Vector types to Eigen Matrices.
    Eigen::Matrix3f localPoints;
    Eigen::Matrix3f worldPoints;

    for(int s = 0; s < PoseCandidate::KABSCH_POINTS; ++s)
    {
      localPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsCamera[s].v);
      worldPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsWorld[s].v);
    }

    // Run Kabsch and store the result in the candidate cameraPose matrix.
    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = GeometryUtil::estimate_rigid_transform(localPoints, worldPoints);
  }
}

namespace {

/**
 * \brief This struct is used to hold pointers to the data used when computing the residual energy.
 */
struct PointsForLM
{
  const Vector4f *cameraPoints;
  const Keypoint3DColourCluster *predictedModes;
  uint32_t nbPoints; // Comes last to avoid padding.
};

/**
 * \brief Compute the energy using the Mahalanobis distance.
 */
static double EnergyForContinuous3DOptimizationUsingFullCovariance(const PointsForLM& pts, const ORUtils::SE3Pose& candidateCameraPose, double *jac = NULL)
{
  double res = 0.0;

  if(jac)
  {
    for(uint32_t i = 0; i < 6; ++i) jac[i] = 0;
  }

  for(uint32_t i = 0; i < pts.nbPoints; ++i)
  {
    if(pts.cameraPoints[i].w == 0.f) continue;

    const Vector3f transformedPt = candidateCameraPose.GetM() * pts.cameraPoints[i].toVector3();
    const Vector3f diff = transformedPt - pts.predictedModes[i].position;
    const Vector3f invCovarianceTimesDiff = pts.predictedModes[i].positionInvCovariance * diff;
//    const float err = sqrtf(dot(diff, invCovarianceTimesDiff)); // Mahalanobis distance
    const float err = dot(diff, invCovarianceTimesDiff); // sqr Mahalanobis distance

    res += err;

    if(jac)
    {
      const Vector3f normGradient = 2 * invCovarianceTimesDiff;
//      const Vector3f normGradient = invCovarianceTimesDiff / err;

      const Vector3f poseGradient[6] = {
          Vector3f(1, 0, 0),
          Vector3f(0, 1, 0),
          Vector3f(0, 0, 1),
          -Vector3f(0, transformedPt.z, -transformedPt.y),
          -Vector3f(-transformedPt.z, 0, transformedPt.x),
          -Vector3f(transformedPt.y, -transformedPt.x, 0),
      };

      for(uint32_t i = 0; i < 6; ++i)
      {
        jac[i] += dot(normGradient, poseGradient[i]);
      }
    }
  }

  return res;
}

/**
 * \brief Function that will be called by alglib's optimiser.
 */
static void Continuous3DOptimizationUsingFullCovariance(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingFullCovariance(*ptsLM, testPose);
}

/**
 * \brief Function that will be called by alglib's optimiser (analytic jacobians variant).
 */
static void Continuous3DOptimizationUsingFullCovarianceJac(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingFullCovariance(*ptsLM, testPose, jac[0]);
}

/***************************************************/
/* Routines to optimize the sum of 3D L2 distances */
/***************************************************/

/**
 * \brief Compute the energy using the L2 distance between the points.
 */
static double EnergyForContinuous3DOptimizationUsingL2(const PointsForLM& pts, const ORUtils::SE3Pose& candidateCameraPose, double *jac = NULL)
{
  double res = 0.0;

  if(jac)
  {
    for(uint32_t i = 0; i < 6; ++i) jac[i] = 0;
  }

  for(uint32_t i = 0; i < pts.nbPoints; ++i)
  {
    if(pts.cameraPoints[i].w == 0.f) continue;

    const Vector3f cameraPoint = pts.cameraPoints[i].toVector3();
    const Vector3f transformedPt = candidateCameraPose.GetM() * cameraPoint;
    const Vector3f diff = transformedPt - pts.predictedModes[i].position;
//    const double err = length(diff); // distance
    const double err = dot(diff, diff); // sqr distance
    res += err;

    if(jac)
    {
      const Vector3f normGradient = 2 * diff;
//      const Vector3f normGradient = diff / err;

      const Vector3f poseGradient[6] = {
          Vector3f(1, 0, 0),
          Vector3f(0, 1, 0),
          Vector3f(0, 0, 1),
          -Vector3f(0, transformedPt.z, -transformedPt.y),
          -Vector3f(-transformedPt.z, 0, transformedPt.x),
          -Vector3f(transformedPt.y, -transformedPt.x, 0),
      };

      for(uint32_t i = 0; i < 6; ++i)
      {
        jac[i] += dot(normGradient, poseGradient[i]);
      }
    }
  }

  return res;
}

/**
 * \brief Function that will be called by alglib's optimiser.
 */
static void Continuous3DOptimizationUsingL2(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingL2(*ptsLM, testPose);
}

/**
 * \brief Function that will be called by alglib's optimiser (analytic jacobians variant).
 */
static void Continuous3DOptimizationUsingL2Jac(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingL2(*ptsLM, testPose, jac[0]);
}

/** Alglib's diagnostic function. Currently does nothing, but could print stuff. */
static void call_after_each_step(const alglib::real_1d_array& x, double func, void *ptr) { return; }

} // anonymous namespace

bool PreemptiveRansac::update_candidate_pose(int candidateIdx) const
{
  // Fill the struct that will be passed to the optimiser.
  PointsForLM ptsForLM;

  // The current number of inlier points.
  ptsForLM.nbPoints = m_inliersIndicesBlock->dataSize;

  // The linearised offset in the pose optimisation buffers.
  const uint32_t candidateOffset = ptsForLM.nbPoints * candidateIdx;

  // Pointers to the data for this candidate.
  ptsForLM.cameraPoints = m_poseOptimisationCameraPoints->GetData(MEMORYDEVICE_CPU) + candidateOffset;
  ptsForLM.predictedModes = m_poseOptimisationPredictedModes->GetData(MEMORYDEVICE_CPU) + candidateOffset;

  // Assumption is that they have been already copied onto the CPU memory, the CUDA subclass should make sure of that.
  // In the long term we will move the whole optimisation step on the GPU (as proper shared code).
  PoseCandidate& poseCandidate = m_poseCandidates->GetData(MEMORYDEVICE_CPU)[candidateIdx];

  // Construct an SE3 pose to optimise from the raw matrix.
  ORUtils::SE3Pose candidateCameraPose(poseCandidate.cameraPose);

  // Convert the 6 parameters to a format that alglib likes.
  alglib::real_1d_array ksi_;
  ksi_.setlength(6);
  for(int i = 0; i < 6; ++i) ksi_[i] = static_cast<double>(candidateCameraPose.GetParams()[i]);

  // Set up the optimiser.
  alglib::minlmstate state;
  const double differentiationStep = 0.0001;
  alglib::minlmcreatev(6, 1, ksi_, differentiationStep, state);
//  alglib::minlmcreatevj(6, 1, ksi_, state);
  alglib::minlmsetcond(state, m_poseOptimisationGradientThreshold, m_poseOptimisationEnergyThreshold, m_poseOptimisationStepThreshold, m_poseOptimisationMaxIterations);

  // Run the optimiser.
  if(m_usePredictionCovarianceForPoseOptimization)
  {
    alglib::minlmoptimize(state, Continuous3DOptimizationUsingFullCovariance, Continuous3DOptimizationUsingFullCovarianceJac, call_after_each_step, &ptsForLM);
  }
  else
  {
    alglib::minlmoptimize(state, Continuous3DOptimizationUsingL2, Continuous3DOptimizationUsingL2Jac, call_after_each_step, &ptsForLM);
  }

  // Extract the results and update the SE3Pose accordingly.
  alglib::minlmreport rep;
  alglib::minlmresults(state, ksi_, rep);
  candidateCameraPose.SetFrom(
    static_cast<float>(ksi_[0]),
    static_cast<float>(ksi_[1]),
    static_cast<float>(ksi_[2]),
    static_cast<float>(ksi_[3]),
    static_cast<float>(ksi_[4]),
    static_cast<float>(ksi_[5])
  );

  // Store the updated pose iff the optimisation succeeded.
  if(rep.terminationtype >= 0)
  {
    poseCandidate.cameraPose = candidateCameraPose.GetM();
    return true;
  }

  // Optimisation failed.
  return false;
}

void PreemptiveRansac::update_host_pose_candidates() const
{
  // NOP by default.
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void PreemptiveRansac::print_timer(const AverageTimer& timer)
{
  std::cout << timer.name() << ": " << timer.count() << " times, avg: " << timer.average_duration() << ".\n";
}

}
