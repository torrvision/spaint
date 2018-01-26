/**
 * grove: PreemptiveRansac.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/interface/PreemptiveRansac.h"
using namespace tvgutil;

#include <boost/lexical_cast.hpp>
#include <boost/timer/timer.hpp>

#include <Eigen/Dense>

#ifdef WITH_OPENMP
#include <omp.h>
#endif

#include <itmx/base/MemoryBlockFactory.h>
#include <itmx/geometry/GeometryUtil.h>
using namespace itmx;

//#################### MACROS ####################

// Enable/disable the print-out of more detailed timings (very verbose, so disabled by default).
//#define ENABLE_TIMERS

namespace grove {

//#################### CONSTRUCTORS ####################

PreemptiveRansac::PreemptiveRansac(const SettingsContainer_CPtr& settings)
: m_poseCandidatesAfterCull(0),
  m_timerCandidateGeneration("Candidate Generation"),
  m_timerFirstComputeEnergy("First Energy Computation"),
  m_timerFirstTrim("First Trim"),
  m_timerTotal("P-RANSAC Total"),
  m_settings(settings)
{
  const std::string settingsNamespace = "PreemptiveRansac.";

  // By default, we set all parameters as in SCoRe forests.
  m_checkMinDistanceBetweenSampledModes = m_settings->get_first_value<bool>(settingsNamespace + "checkMinDistanceBetweenSampledModes", true);                   // Whether or not to force sampled modes to have a minimum distance between them.
  m_checkRigidTransformationConstraint = m_settings->get_first_value<bool>(settingsNamespace + "checkRigidTransformationConstraint", true);                     // Setting this to false speeds things up a lot, at the expense of quality.
  m_maxCandidateGenerationIterations = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxCandidateGenerationIterations", 6000);                     // The maximum number of times we sample three pixel-mode pairs in the attempt to generate a pose candidate.
  m_maxPoseCandidates = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxPoseCandidates", 1024);                                                   // The number of initial pose candidates.
  m_maxPoseCandidatesAfterCull = m_settings->get_first_value<uint32_t>(settingsNamespace + "maxPoseCandidatesAfterCull", 64);                                   // Aggressively cull hypotheses to this number.
  m_maxTranslationErrorForCorrectPose = m_settings->get_first_value<float>(settingsNamespace + "maxTranslationErrorForCorrectPose", 0.05f);                     // In m.
  m_minSquaredDistanceBetweenSampledModes = m_settings->get_first_value<float>(settingsNamespace + "minSquaredDistanceBetweenSampledModes", 0.3f * 0.3f);       // In m.

  // Optimisation parameters defaulted as in Valentin's paper.
  m_poseOptimisationEnergyThreshold = m_settings->get_first_value<double>(settingsNamespace + "poseOptimisationEnergyThreshold", 0.0);                          // Part of the termination condition for the pose optimisation.
  m_poseOptimisationGradientThreshold = m_settings->get_first_value<double>(settingsNamespace + "poseOptimisationGradientThreshold", 1e-6);                     // Part of the termination condition for the pose optimisation.
  m_poseOptimisationInlierThreshold = m_settings->get_first_value<float>(settingsNamespace + "poseOptimizationInlierThreshold", 0.2f);                          // In m.
  m_poseOptimisationMaxIterations = m_settings->get_first_value<uint32_t>(settingsNamespace + "poseOptimisationMaxIterations", 100);                            // Maximum number of LM iterations.
  m_poseOptimisationStepThreshold = m_settings->get_first_value<double>(settingsNamespace + "poseOptimisationStepThreshold", 0.0);                              // Part of the termination condition for the pose optimisation.
  m_poseUpdate = m_settings->get_first_value<bool>(settingsNamespace + "poseUpdate", true);                                                                     // Whether or not to optimise the poses with LM.
  m_printTimers = m_settings->get_first_value<bool>(settingsNamespace + "printTimers", false);                                                                  // Whether or not to print the timers for each phase.
  m_ransacInliersPerIteration = m_settings->get_first_value<uint32_t>(settingsNamespace + "ransacInliersPerIteration", 500);                                    // The number of inliers sampled in each P-RANSAC iteration.
  m_useAllModesPerLeafInPoseHypothesisGeneration = m_settings->get_first_value<bool>(settingsNamespace + "useAllModesPerLeafInPoseHypothesisGeneration", true); // If false, use the first mode only (representing the largest cluster).
  m_usePredictionCovarianceForPoseOptimization = m_settings->get_first_value<bool>(settingsNamespace + "usePredictionCovarianceForPoseOptimization", true);     // If false, use L2.

  // Each RANSAC iteration after the initial cull adds m_ransacInliersPerIteration inliers to the set, so we allocate enough space for all of them up-front.
  m_nbMaxInliers = m_ransacInliersPerIteration * static_cast<uint32_t>(std::ceil(log2(m_maxPoseCandidatesAfterCull)));

  // Allocate memory.
  const MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_inliersIndicesBlock = mbf.make_block<int>(m_nbMaxInliers);
  m_inliersMaskImage = mbf.make_image<int>();
  m_poseCandidates = mbf.make_block<PoseCandidate>(m_maxPoseCandidates);

  const uint32_t poseOptimisationBufferSize = static_cast<uint32_t>(m_nbMaxInliers * m_maxPoseCandidates);
  m_poseOptimisationCameraPoints = mbf.make_block<Vector4f>(poseOptimisationBufferSize);
  m_poseOptimisationPredictedModes = mbf.make_block<Keypoint3DColourCluster>(poseOptimisationBufferSize);

#ifdef ENABLE_TIMERS
  // Force the average timers to on as well if we want verbose printing.
  m_printTimers = true;
#endif

  // Set up the remaining timers.
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

void PreemptiveRansac::update_candidate_poses()
{
  // Note: This is a default implementation of the abstract function - it is intended to be called / overridden by derived classes.

  const int nbPoseCandidates = static_cast<int>(m_poseCandidates->dataSize);

#ifdef WITH_OPENMP
  #pragma omp parallel for schedule(dynamic)
#endif
  for(int i = 0; i < nbPoseCandidates; ++i)
  {
    update_candidate_pose(i);
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<PoseCandidate> PreemptiveRansac::estimate_pose(const Keypoint3DColourImage_CPtr& keypointsImage, const ScorePredictionsImage_CPtr& predictionsImage)
{
  /*
  Note: In this function and in the virtual functions of the CPU and CUDA subclasses, we directly access and overwrite
        the dataSize member of several MemoryBlock variables instead of keeping a separate "valid size" variable.
        This is done on purpose since the Update*From* functions of MemoryBlock/Images only move the first dataSize
        elements of the block. By changing the number to the actual number of elements to move we can gain a slight
        speed-up of the system.
  */

  m_timerTotal.start();

  // Copy the keypoints and predictions images into member variables to avoid explicitly passing them to every function.
  m_keypointsImage = keypointsImage;
  m_predictionsImage = predictionsImage;

  // Step 1: Generate the initial pose candidates.
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

  // Step 2: If necessary, aggressively cull the initial candidates to reduce the computational cost of the remaining steps.
  if(m_poseCandidates->dataSize > m_maxPoseCandidatesAfterCull)
  {
    m_timerFirstTrim.start();
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6, "first trim: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    // Step 2(a): First, sample a set of points from the input data to use to evaluate the quality of each candidate.
    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6, "sample inliers: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      const bool useMask = false; // no mask for the first pass
      sample_inlier_candidates(useMask);
    }

    // Step 2(b): Then, evaluate the candidates and sort them in non-increasing order of quality.
    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6, "compute energies and sort: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      m_timerFirstComputeEnergy.start();
      compute_energies_and_sort();
      m_timerFirstComputeEnergy.stop();
    }

    // Step 2(c): Finally, trim the number of candidates down to the maximum number allowed. Since we previously sorted
    //            the candidates by quality, this has the effect of keeping only the best ones.
    m_poseCandidates->dataSize = m_maxPoseCandidatesAfterCull;

    m_timerFirstTrim.stop();
  }

  m_poseCandidatesAfterCull = static_cast<uint32_t>(m_poseCandidates->dataSize);

#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6, "ransac: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

  // Step 3: Reset the inlier mask (and inliers that might have been selected in a previous invocation of the method).
  m_inliersMaskImage->ChangeDims(m_keypointsImage->noDims); // Happens only once (no-op on subsequent occasions).
  m_inliersMaskImage->Clear();                              // This and the following happen every time.
  m_inliersIndicesBlock->dataSize = 0;

  // Step 4: Run preemptive RANSAC until only a single candidate remains.
  int iteration = 0;
  while(m_poseCandidates->dataSize > 1)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6, "ransac iteration: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

#if 0
    std::cout << candidates.size() << " camera(s) remaining" << std::endl;
#endif

    // Step 4(a): Sample a set of keypoints from the input image. Record that thay have been selected in the mask image, to avoid selecting them again.
    m_timerInlierSampling[iteration].start();
    const bool useMask = true;
    sample_inlier_candidates(useMask);
    m_timerInlierSampling[iteration].stop();

    // Step 4(b): If pose update is enabled, optimise all remaining candidates, taking into account the newly selected inliers.
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

    // Step 4(c): Compute the energy for each candidate and sort them in non-increasing order of quality.
    m_timerComputeEnergy[iteration].start();
    compute_energies_and_sort();
    m_timerComputeEnergy[iteration].stop();

    // Step 4(d): Remove the worse half of the candidates.
    m_poseCandidates->dataSize /= 2;

    ++iteration;
  }

  // If we initially generated a single candidate, the update step above wouldn't have been executed (zero iterations). Force its execution.
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

    // Run the optimisation.
    m_timerOptimisation[iteration].start();
    update_candidate_poses();
    m_timerOptimisation[iteration].stop();
  }

  m_timerTotal.stop();

  // Make sure the pose candidates available on the host are up to date.
  update_host_pose_candidates();

  // Step 5: If we managed to generate at least one candidate, return the best one.
  const PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  return m_poseCandidates->dataSize > 0 ? boost::optional<PoseCandidate>(candidates[0]) : boost::none;
}

void PreemptiveRansac::get_best_poses(std::vector<PoseCandidate>& poseCandidates) const
{
  // Set up the output container.
  poseCandidates.clear();
  poseCandidates.reserve(m_poseCandidatesAfterCull);

  // Copy all the poses that survived the initial cull. They are ordered in blocks: the first one is the one returned by
  // estimate_pose, the second is the one removed after the last RANSAC iteration, the third and fourth are removed in the
  // iteration before (whilst they are not in a specific order, they are worse than those in positions 0 and 1), etc.
  const PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  for(uint32_t poseIdx = 0; poseIdx < m_poseCandidatesAfterCull; ++poseIdx)
  {
    poseCandidates.push_back(candidates[poseIdx]);
  }
}

int PreemptiveRansac::get_min_nb_required_points() const
{
  // We need at least the number of inliers required for a RANSAC iteration.
  return m_ransacInliersPerIteration;
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void PreemptiveRansac::compute_candidate_poses_kabsch()
{
  // We assume that the data on the CPU is up-to-date (the CUDA subclass must ensure this).
  // This function will probably go away as soon as we implement a shared SVD solver.
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

#if 0
  std::cout << "Generated " << nbPoseCandidates << " candidates." << std::endl;
#endif

  // For each candidate:
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(size_t candidateIdx = 0; candidateIdx < nbPoseCandidates; ++candidateIdx)
  {
    PoseCandidate& candidate = poseCandidates[candidateIdx];

    // Copy the camera and world space points into two Eigen matrices (one point per column in each) on which we can run the Kabsch algorithm.
    Eigen::Matrix3f cameraPoints;
    Eigen::Matrix3f worldPoints;
    for(int i = 0; i < PoseCandidate::KABSCH_POINTS; ++i)
    {
      cameraPoints.col(i) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsCamera[i].v);
      worldPoints.col(i) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsWorld[i].v);
    }

    // Run the Kabsch algorithm and store the resulting camera -> world transformation in the candidate's cameraPose matrix.
    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = GeometryUtil::estimate_rigid_transform(cameraPoints, worldPoints);
  }
}

bool PreemptiveRansac::update_candidate_pose(int candidateIdx) const
{
  // Fill the struct that will be passed to the optimiser.
  PointsForLM ptsForLM;

  // The current number of inlier points.
  ptsForLM.nbPoints = static_cast<uint32_t>(m_inliersIndicesBlock->dataSize);

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
  // No-op by default
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void PreemptiveRansac::call_after_each_step(const alglib::real_1d_array& x, double func, void *ptr)
{
  return;
}

void PreemptiveRansac::Continuous3DOptimizationUsingFullCovariance(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingFullCovariance(*ptsLM, testPose);
}

void PreemptiveRansac::Continuous3DOptimizationUsingFullCovarianceJac(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingFullCovariance(*ptsLM, testPose, jac[0]);
}

void PreemptiveRansac::Continuous3DOptimizationUsingL2(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingL2(*ptsLM, testPose);
}

void PreemptiveRansac::Continuous3DOptimizationUsingL2Jac(const alglib::real_1d_array& ksi, alglib::real_1d_array& fi, alglib::real_2d_array& jac, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingL2(*ptsLM, testPose, jac[0]);
}

double PreemptiveRansac::EnergyForContinuous3DOptimizationUsingFullCovariance(const PointsForLM& pts, const ORUtils::SE3Pose& candidateCameraPose, double *jac)
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

double PreemptiveRansac::EnergyForContinuous3DOptimizationUsingL2(const PointsForLM& pts, const ORUtils::SE3Pose& candidateCameraPose, double *jac)
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

void PreemptiveRansac::print_timer(const AverageTimer& timer)
{
  std::cout << timer.name() << ": " << timer.count() << " times, avg: " << timer.average_duration() << ".\n";
}

}
