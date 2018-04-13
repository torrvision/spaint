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

PreemptiveRansac::PreemptiveRansac(const SettingsContainer_CPtr& settings, const std::string& settingsNamespace)
: m_poseCandidatesAfterCull(0),
  m_timerCandidateGeneration("Candidate Generation"),
  m_timerFirstComputeEnergy("First Energy Computation"),
  m_timerFirstTrim("First Trim"),
  m_timerTotal("P-RANSAC Total"),
  m_settings(settings)
{
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
  m_inlierRasterIndicesBlock = mbf.make_block<int>(m_nbMaxInliers);
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

  // Reset the number of inliers ready for the new pose estimation.
  {
    const bool resetMask = false;
    reset_inliers(resetMask);
  }

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
      sample_inliers(useMask);
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

  // Step 3: Reset the inlier mask and clear any inliers that might have been selected in a previous invocation of the method.
  {
    const bool resetMask = true;
    reset_inliers(resetMask);
  }

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
    sample_inliers(useMask);
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
    sample_inliers(true);
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
    for(int i = 0; i < PoseCandidate::KABSCH_CORRESPONDENCES_NEEDED; ++i)
    {
      cameraPoints.col(i) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsCamera[i].v);
      worldPoints.col(i) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsWorld[i].v);
    }

    // Run the Kabsch algorithm and store the resulting camera -> world transformation in the candidate's cameraPose matrix.
    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = GeometryUtil::estimate_rigid_transform(cameraPoints, worldPoints);
  }
}

void PreemptiveRansac::reset_inliers(bool resetMask)
{
  if(resetMask)
  {
    m_inliersMaskImage->ChangeDims(m_keypointsImage->noDims); // happens only once (i.e. a no-op on subsequent occasions)
    m_inliersMaskImage->Clear();
  }

  m_inlierRasterIndicesBlock->dataSize = 0;
}

bool PreemptiveRansac::update_candidate_pose(int candidateIdx) const
{
  // Fill in the struct that will be passed to the optimiser.
  PointsForLM ptsForLM;
  ptsForLM.nbPoints = static_cast<uint32_t>(m_inlierRasterIndicesBlock->dataSize);                          // The current number of inlier points.
  const uint32_t candidateOffset = ptsForLM.nbPoints * candidateIdx;                                        // The linearised offset in the pose optimisation buffers.
  ptsForLM.cameraPoints = m_poseOptimisationCameraPoints->GetData(MEMORYDEVICE_CPU) + candidateOffset;      // Pointers to the data for this candidate.
  ptsForLM.predictedModes = m_poseOptimisationPredictedModes->GetData(MEMORYDEVICE_CPU) + candidateOffset;

  // Look up the pose candidate. The assumption is that all of the pose candidates have already been copied
  // across to the CPU (non-CPU subclasses must ensure this). The plan is ultimately to reimplement the
  // optimisation using shared code, but for now everything is done on the CPU.
  PoseCandidate& poseCandidate = m_poseCandidates->GetData(MEMORYDEVICE_CPU)[candidateIdx];

  // Convert the candidate's current pose to a 6D twist vector that can be optimised by alglib.
  alglib::real_1d_array xi = make_twist_from_pose(poseCandidate.cameraPose);

  // Set up the optimiser itself.
  alglib::minlmstate state;
  const double differentiationStep = 0.0001;
#if 1
  alglib::minlmcreatev(6, 1, xi, differentiationStep, state);
#else
  alglib::minlmcreatevj(6, 1, xi, state);
#endif
  alglib::minlmsetcond(state, m_poseOptimisationGradientThreshold, m_poseOptimisationEnergyThreshold, m_poseOptimisationStepThreshold, m_poseOptimisationMaxIterations);

  // Run the optimiser.
  if(m_usePredictionCovarianceForPoseOptimization)
  {
    alglib::minlmoptimize(state, alglib_func_mahalanobis, alglib_jac_mahalanobis, alglib_rep, &ptsForLM);
  }
  else
  {
    alglib::minlmoptimize(state, alglib_func_l2, alglib_jac_l2, alglib_rep, &ptsForLM);
  }

  // Extract the results of the optimisation.
  alglib::minlmreport report;
  alglib::minlmresults(state, xi, report);
  const bool succeeded = report.terminationtype >= 0;

  // Iff the optimisation succeeded, update the candidate's pose.
  if(succeeded)
  {
    poseCandidate.cameraPose = make_pose_from_twist(xi).GetM();
  }

  return succeeded;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void PreemptiveRansac::update_host_pose_candidates() const
{
  // No-op by default
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void PreemptiveRansac::alglib_func_l2(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, void *pts)
{
  phi[0] = compute_energy_l2(make_pose_from_twist(xi), *reinterpret_cast<PointsForLM*>(pts));
}

void PreemptiveRansac::alglib_func_mahalanobis(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, void *pts)
{
  phi[0] = compute_energy_mahalanobis(make_pose_from_twist(xi), *reinterpret_cast<PointsForLM*>(pts));
}

void PreemptiveRansac::alglib_jac_l2(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, alglib::real_2d_array& jac, void *pts)
{
  phi[0] = compute_energy_l2(make_pose_from_twist(xi), *reinterpret_cast<PointsForLM*>(pts), jac[0]);
}

void PreemptiveRansac::alglib_jac_mahalanobis(const alglib::real_1d_array& xi, alglib::real_1d_array& phi, alglib::real_2d_array& jac, void *pts)
{
  phi[0] = compute_energy_mahalanobis(make_pose_from_twist(xi), *reinterpret_cast<PointsForLM*>(pts), jac[0]);
}

void PreemptiveRansac::alglib_rep(const alglib::real_1d_array& xi, double phi, void *pts)
{
  return;
}

double PreemptiveRansac::compute_energy_l2(const ORUtils::SE3Pose& candidateCameraPose, const PointsForLM& pts, double *jac)
{
  double res = 0.0;

  // If we're computing the Jacobian of the 6D twist vector, initially reset it to zero.
  if(jac)
  {
    for(int i = 0; i < 6; ++i)
    {
      jac[i] = 0.0;
    }
  }

  // For each point under consideration:
  for(uint32_t i = 0; i < pts.nbPoints; ++i)
  {
    // If the point's position in camera space is invalid, skip it.
    if(pts.cameraPoints[i].w == 0.0f) continue;

    // Compute the difference between the point's position in world space (i) as predicted by the camera
    // pose and its position in camera space, and (ii) as predicted by the position of the chosen mode.
    const Vector3f cameraPoint = pts.cameraPoints[i].toVector3();
    const Vector3f transformedPt = candidateCameraPose.GetM() * cameraPoint;
    const Vector3f diff = transformedPt - pts.predictedModes[i].position;

    // Based on this difference, add an L2 distance-based error term to the resulting energy.
#if 1
    const double err = dot(diff, diff); // squared L2 distance
#else
    const double err = length(diff);    // unsquared L2 distance
#endif

    res += err;

    // If we're computing the Jacobian of the 6D twist vector, update it as per equation (10.23) in
    // "A tutorial on SE(3) transformation parameterizations and on-manifold optimization" (Blanco).
    if(jac)
    {
#if 1
      const Vector3f normGradient = 2 * diff;   // for squared L2 distance
#else
      const Vector3f normGradient = diff / err; // for unsquared L2 distance
#endif

      const Vector3f poseGradient[6] = {
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        -Vector3f(0.0f, transformedPt.z, -transformedPt.y),
        -Vector3f(-transformedPt.z, 0.0f, transformedPt.x),
        -Vector3f(transformedPt.y, -transformedPt.x, 0.0f),
      };

      for(int i = 0; i < 6; ++i)
      {
        jac[i] += dot(normGradient, poseGradient[i]);
      }
    }
  }

  return res;
}

double PreemptiveRansac::compute_energy_mahalanobis(const ORUtils::SE3Pose& candidateCameraPose, const PointsForLM& pts, double *jac)
{
  double res = 0.0;

  // If we're computing the Jacobian of the 6D twist vector, initially reset it to zero.
  if(jac)
  {
    for(int i = 0; i < 6; ++i)
    {
      jac[i] = 0.0;
    }
  }

  // For each point under consideration:
  for(uint32_t i = 0; i < pts.nbPoints; ++i)
  {
    // If the point's position in camera space is invalid, skip it.
    if(pts.cameraPoints[i].w == 0.0f) continue;

    // Compute the difference between the point's position in world space (i) as predicted by the camera
    // pose and its position in camera space, and (ii) as predicted by the position of the chosen mode.
    const Vector3f transformedPt = candidateCameraPose.GetM() * pts.cameraPoints[i].toVector3();
    const Vector3f diff = transformedPt - pts.predictedModes[i].position;

    // Based on this difference, add a Mahalanobis distance-based error term to the resulting energy.
    // See also: https://en.wikipedia.org/wiki/Mahalanobis_distance.
    const Vector3f invCovarianceTimesDiff = pts.predictedModes[i].positionInvCovariance * diff;
#if 1
    const float err = dot(diff, invCovarianceTimesDiff);        // squared Mahalanobis distance
#else
    const float err = sqrtf(dot(diff, invCovarianceTimesDiff)); // unsquared Mahalanobis distance
#endif

    res += err;

    // If we're computing the Jacobian of the 6D twist vector, update it accordingly.
    if(jac)
    {
#if 1
      const Vector3f normGradient = 2 * invCovarianceTimesDiff;   // for squared Mahalanobis distance
#else
      const Vector3f normGradient = invCovarianceTimesDiff / err; // for unsquared Mahalanobis distance
#endif

      const Vector3f poseGradient[6] = {
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
        -Vector3f(0.0f, transformedPt.z, -transformedPt.y),
        -Vector3f(-transformedPt.z, 0.0f, transformedPt.x),
        -Vector3f(transformedPt.y, -transformedPt.x, 0.0f),
      };

      for(int i = 0; i < 6; ++i)
      {
        jac[i] += dot(normGradient, poseGradient[i]);
      }
    }
  }

  return res;
}

ORUtils::SE3Pose PreemptiveRansac::make_pose_from_twist(const alglib::real_1d_array& xi)
{
  return ORUtils::SE3Pose(
    static_cast<float>(xi[0]),
    static_cast<float>(xi[1]),
    static_cast<float>(xi[2]),
    static_cast<float>(xi[3]),
    static_cast<float>(xi[4]),
    static_cast<float>(xi[5])
  );
}

alglib::real_1d_array PreemptiveRansac::make_twist_from_pose(const ORUtils::SE3Pose& pose)
{
  alglib::real_1d_array xi;
  xi.setlength(6);
  for(int i = 0; i < 6; ++i)
  {
    xi[i] = static_cast<double>(pose.GetParams()[i]);
  }
  return xi;
}

void PreemptiveRansac::print_timer(const AverageTimer& timer)
{
  std::cout << timer.name() << ": " << timer.count() << " times, avg: " << timer.average_duration() << ".\n";
}

}
