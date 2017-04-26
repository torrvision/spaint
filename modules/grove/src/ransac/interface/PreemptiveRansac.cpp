/**
 * grove: PreemptiveRansac.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/interface/PreemptiveRansac.h"

#include <boost/lexical_cast.hpp>
#include <boost/timer/timer.hpp>

#include <Eigen/Dense>
#include <alglib/optimization.h>

#ifdef WITH_OPENMP
#include <omp.h>
#endif

#include <ORUtils/SE3Pose.h>

#include <itmx/MemoryBlockFactory.h>
using namespace itmx;

// To enable more detailed timing prints (VERY VERBOSE)
//#define ENABLE_TIMERS

namespace grove {

//#################### ANONYMOUS FREE FUNCTIONS ####################

namespace {

/**
 * \brief Estimate a rigid transformation between two sets of 3D points using the Kabsch algorithm.
 * \param P        A set of 3 3D points in camera coordinates.
 * \param Q        A set of 3 3D points in world coordinates.
 * \param weights  A set of weights associated to the points.
 * \param resRot   The resulting rotation matrix.
 * \param resTrans The resulting translation vector.
 */
void Kabsch(Eigen::Matrix3f &P,
            Eigen::Matrix3f &Q,
            Eigen::Vector3f &weights,
            Eigen::Matrix3f &resRot,
            Eigen::Vector3f &resTrans)
{
  const int D = P.rows(); // dimension of the space
  const Eigen::Vector3f normalizedWeights = weights / weights.sum();

  // Centroids
  const Eigen::Vector3f p0 = P * normalizedWeights;
  const Eigen::Vector3f q0 = Q * normalizedWeights;
  const Eigen::Vector3f v1 = Eigen::Vector3f::Ones();

  const Eigen::Matrix3f P_centred = P - p0 * v1.transpose(); // translating P to center the origin
  const Eigen::Matrix3f Q_centred = Q - q0 * v1.transpose(); // translating Q to center the origin

  // Covariance between both matrices
  const Eigen::Matrix3f C = P_centred * normalizedWeights.asDiagonal() * Q_centred.transpose();

  // SVD
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);

  const Eigen::Matrix3f V = svd.matrixU();
  const Eigen::Matrix3f W = svd.matrixV();
  Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

  if ((V * W.transpose()).determinant() < 0)
  {
    I(D - 1, D - 1) = -1;
  }

  // Recover the rotation and translation
  resRot = W * I * V.transpose();
  resTrans = q0 - resRot * p0;
}

/**
 * \brief Estimate a rigid transformation between two sets of 3D points using the Kabsch algorithm.
 * \param P        A set of 3 3D points in camera coordinates.
 * \param Q        A set of 3 3D points in world coordinates.
 * \param resRot   The resulting rotation matrix.
 * \param resTrans The resulting translation vector.
 */
void Kabsch(Eigen::Matrix3f &P, Eigen::Matrix3f &Q, Eigen::Matrix3f &resRot, Eigen::Vector3f &resTrans)
{
  // Setup unitary weights and call the function above.
  Eigen::Vector3f weights = Eigen::Vector3f::Ones();
  Kabsch(P, Q, weights, resRot, resTrans);
}

/**
 * \brief Estimate a rigid transformation between two sets of 3D points using the Kabsch algorithm.
 * \param P        A set of 3 3D points in camera coordinates.
 * \param Q        A set of 3 3D points in world coordinates.
 *
 * \return The estimated transformation matrix.
 */
Eigen::Matrix4f Kabsch(Eigen::Matrix3f &P, Eigen::Matrix3f &Q)
{
  Eigen::Matrix3f resRot;
  Eigen::Vector3f resTrans;

  // Call the other function.
  Kabsch(P, Q, resRot, resTrans);

  // Cecompose R + t in Rt.
  Eigen::Matrix4f res;
  res.block<3, 3>(0, 0) = resRot;
  res.block<3, 1>(0, 3) = resTrans;
  return res;
}

/**
 * \brief Pretty print a timer value.
 *
 * \param timer The timer to print.
 */
void printTimer(const PreemptiveRansac::AverageTimer &timer)
{
  std::cout << timer.name() << ": " << timer.count() << " times, avg: "
            << (timer.count() > 0 ? boost::chrono::duration_cast<boost::chrono::milliseconds>(timer.average_duration())
                                  : boost::chrono::milliseconds())
            << ".\n";
}
}

//#################### CONSTRUCTORS ####################

PreemptiveRansac::PreemptiveRansac()
  : m_timerCandidateGeneration("Candidate Generation")
  , m_timerFirstComputeEnergy("First Energy Computation")
  , m_timerFirstTrim("First Trim")
  , m_timerTotal("P-RANSAC Total")
{
  // Set parameters as in scoreforests. TODO: use the app parameters to allow configurability.
  m_batchSizeRansac = 500;
  m_checkMinDistanceBetweenSampledModes = true;
  //  m_checkRigidTransformationConstraint = false; // Speeds up a lot, at the expense of quality.
  m_checkRigidTransformationConstraint = true;
  m_minSquaredDistanceBetweenSampledModes = 0.3f * 0.3f;
  m_nbMaxInliers = 3000; // 500 per ransac iteration, starting from 64, not 1024. Could probably be computed from the
                         // other parameters.
  m_nbMaxPoseCandidates = 1024;
  m_nbPointsForKabschBoostrap = 3;
  m_poseOptimizationInlierThreshold = 0.2f;
  m_poseUpdate = true;
  m_translationErrorMaxForCorrectPose = 0.05f;
  //  m_trimKinitAfterFirstEnergyComputation = 1024; // Do not cull hypotheses.
  m_trimKinitAfterFirstEnergyComputation = 64;
  m_useAllModesPerLeafInPoseHypothesisGeneration = true;
  m_usePredictionCovarianceForPoseOptimization = true;
  //  m_usePredictionCovarianceForPoseOptimization = false; //Use L2 mode instead.

  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  // Allocate memory.
  m_inliersIndicesBlock = mbf.make_block<int>(m_nbMaxInliers);
  m_inliersMaskImage = mbf.make_image<int>();
  m_poseCandidates = mbf.make_block<PoseCandidate>(m_nbMaxPoseCandidates);

#ifdef ENABLE_TIMERS
  m_printTimers = true;
#else
  m_printTimers = false;
#endif

  m_printTimers =
      true; // This could probably become an AppParameter and we could leave the define for the verbose prints.

  // Setup the remaining timers.
  for (int i = 1; i <= 6; ++i)
  {
    m_timerInlierSampling.push_back(AverageTimer("Inlier Sampling " + boost::lexical_cast<std::string>(i)));
    m_timerOptimisation.push_back(AverageTimer("Optimisation " + boost::lexical_cast<std::string>(i)));
    m_timerComputeEnergy.push_back(AverageTimer("Energy Computation " + boost::lexical_cast<std::string>(i)));
  }
}

//#################### DESTRUCTOR ####################

PreemptiveRansac::~PreemptiveRansac()
{
  if (m_printTimers)
  {
    printTimer(m_timerTotal);
    printTimer(m_timerCandidateGeneration);
    printTimer(m_timerFirstTrim);
    printTimer(m_timerFirstComputeEnergy);

    for (size_t i = 0; i < m_timerInlierSampling.size(); ++i)
    {
      printTimer(m_timerInlierSampling[i]);
      printTimer(m_timerComputeEnergy[i]);
      printTimer(m_timerOptimisation[i]);
    }
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

boost::optional<PoseCandidate> PreemptiveRansac::estimate_pose(const Keypoint3DColourImage_CPtr &keypoints,
                                                               const ScorePredictionsImage_CPtr &forestPredictions)
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
    boost::timer::auto_cpu_timer t(6,
                                   "generating initial candidates: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    m_timerCandidateGeneration.start();
    generate_pose_candidates();
    m_timerCandidateGeneration.stop();
  }

  // Reset the number of inliers for the new pose estimation.
  m_inliersIndicesBlock->dataSize = 0;

  // 2. If we have to aggressively cull the initial hypotheses to a small number.
  if (m_trimKinitAfterFirstEnergyComputation < m_poseCandidates->dataSize)
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
    m_poseCandidates->dataSize = m_trimKinitAfterFirstEnergyComputation;

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
  while (m_poseCandidates->dataSize > 1)
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
    if (m_poseUpdate)
    {
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

  m_timerTotal.stop();

  // 5. If we have been able to generate at least one candidate hypothesis, return the best one.
  PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  return m_poseCandidates->dataSize > 0 ? candidates[0] : boost::optional<PoseCandidate>();
}

void PreemptiveRansac::get_best_poses(std::vector<PoseCandidate> &poseCandidates) const
{
  // No need to check dataSize since it will likely be 1 after running estimate_pose. If estimate_pose has never run
  // this will probably return garbage.
  // TODO: improve.
  const PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

  // Setup output container.
  poseCandidates.clear();
  poseCandidates.reserve(m_trimKinitAfterFirstEnergyComputation);

  // Copy the all the poses that survived after the initial cull. They are "ordered in blocks":
  // the first one is the one returned by estimate_pose, the second is the one removed after the last ransac iteration,
  // the third and fourth are removed in the iteration before (whilst they are not in a specific order, they are worse
  // than those in position 0 and 1), and so on...
  for (size_t poseIdx = 0; poseIdx < m_trimKinitAfterFirstEnergyComputation; ++poseIdx)
  {
    poseCandidates.push_back(candidates[poseIdx]);
  }
}

int PreemptiveRansac::get_min_nb_required_points() const
{
  return std::max(m_nbPointsForKabschBoostrap, m_batchSizeRansac);
}

//#################### PROTECTED VIRTUAL ABSTRACT MEMBER FUNCTIONS ####################

// Default implementation of the abstract function.
void PreemptiveRansac::update_candidate_poses()
{
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;

  // Assumption is that they have been already copied onto the CPU memory, the CUDA subclass should make sure of that.
  // In the long term we will move the whole optimisation step on the GPU (as proper shared code).
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

// Update every pose in parallel
#ifdef WITH_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (size_t i = 0; i < nbPoseCandidates; ++i)
  {
    update_candidate_pose(poseCandidates[i]);
  }
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
  for (size_t candidateIdx = 0; candidateIdx < nbPoseCandidates; ++candidateIdx)
  {
    PoseCandidate &candidate = poseCandidates[candidateIdx];

    // We haev to copy the points from the ORUtil Vector types to Eigen Matrices.
    Eigen::Matrix3f localPoints;
    Eigen::Matrix3f worldPoints;

    for (int s = 0; s < PoseCandidate::KABSCH_POINTS; ++s)
    {
      localPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsCamera[s].v);
      worldPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(candidate.pointsWorld[s].v);
    }

    // Run Kabsch and store the result in the candidate cameraPose matrix.
    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = Kabsch(localPoints, worldPoints);
  }
}

namespace {

/**
 * \brief This struct is used to compute the residual energy.
 */
struct PointForLM
{
  Vector3f point;
  Mode3DColour mode;

  PointForLM(const Vector3f &pt, const Mode3DColour &md) : point(pt), mode(md) {}
};

typedef std::vector<PointForLM> PointsForLM;

/**
 * \brief Compute the energy using the Mahalanobis distance.
 */
static double EnergyForContinuous3DOptimizationUsingFullCovariance(const PointsForLM &pts,
                                                                   const ORUtils::SE3Pose &candidateCameraPose)
{
  double res = 0.0;

  for (size_t i = 0; i < pts.size(); ++i)
  {
    const PointForLM &pt = pts[i];
    const Vector3f transformedPt = candidateCameraPose.GetM() * pt.point;
    const Vector3f diff = transformedPt - pt.mode.position;
    const double err = dot(diff, pt.mode.positionInvCovariance * diff); // Mahalanobis sqr distance
    res += err;
  }

  return res;
}

/**
 * \brief Function that will be called by alglib's optimiser.
 */
static void
    Continuous3DOptimizationUsingFullCovariance(const alglib::real_1d_array &ksi, alglib::real_1d_array &fi, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingFullCovariance(*ptsLM, testPose);
}

/***************************************************/
/* Routines to optimize the sum of 3D L2 distances */
/***************************************************/

/**
 * \brief Compute the energy using the L2 distance between the points.
 */
static double EnergyForContinuous3DOptimizationUsingL2(const PointsForLM &pts,
                                                       const ORUtils::SE3Pose &candidateCameraPose)
{
  double res = 0.0;

  for (size_t i = 0; i < pts.size(); ++i)
  {
    const PointForLM &pt = pts[i];
    const Vector3f transformedPt = candidateCameraPose.GetM() * pt.point;
    const Vector3f diff = transformedPt - pt.mode.position;
    const double err = length(diff); // sqr distance
    res += err;
  }

  return res;
}

/**
 * \brief Function that will be called by alglib's optimiser.
 */
static void Continuous3DOptimizationUsingL2(const alglib::real_1d_array &ksi, alglib::real_1d_array &fi, void *ptr)
{
  // Convert the void pointer in the proper data type and use the current parameters to set the pose matrix.
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4], ksi[5]);

  // Compute the current energy.
  fi[0] = EnergyForContinuous3DOptimizationUsingL2(*ptsLM, testPose);
}

/** Alglib's diagnostic function. Currently does nothing, but could print stuff. */
static void call_after_each_step(const alglib::real_1d_array &x, double func, void *ptr) { return; }

} // anonymous namespace

bool PreemptiveRansac::update_candidate_pose(PoseCandidate &poseCandidate) const
{
  // The assumption is that all data is available and up to date on the CPU. The CUDA subclass must make sure of it.
  // In the long term the optimisation will become shared code.
  const Keypoint3DColour *keypointsData = m_keypointsImage->GetData(MEMORYDEVICE_CPU);
  const ScorePrediction *predictionsData = m_predictionsImage->GetData(MEMORYDEVICE_CPU);

  const size_t nbInliers = m_inliersIndicesBlock->dataSize;
  const int *inliersData = m_inliersIndicesBlock->GetData(MEMORYDEVICE_CPU);

  // Construct an SE3 pose to optimise from the raw matrix.
  ORUtils::SE3Pose candidateCameraPose(poseCandidate.cameraPose);

  // Fill the inliers array with the point-mode pairs that will be used to compute the energy.
  PointsForLM ptsForLM;
  for (size_t inlierIdx = 0; inlierIdx < nbInliers; ++inlierIdx)
  {
    const int inlierLinearIdx = inliersData[inlierIdx];
    const Vector3f inlierCameraPosition = keypointsData[inlierLinearIdx].position;
    const Vector3f inlierWorldPosition = candidateCameraPose.GetM() * inlierCameraPosition;
    const ScorePrediction &prediction = predictionsData[inlierLinearIdx];

    // Find the best mode, do not rely on the one stored in the inlier because for the randomly sampled inliers it will
    // not be set.
    const int bestModeIdx = score_prediction_get_best_mode(prediction, inlierWorldPosition);
    if (bestModeIdx < 0 || bestModeIdx >= prediction.nbClusters)
      throw std::runtime_error("best mode idx invalid."); // This point should not have been selected as inlier.

    // We also assume that the inlier is valid (we checked that before, when we selected it).
    PointForLM ptLM(inlierCameraPosition, prediction.clusters[bestModeIdx]);

    // We add this pair to the vector of pairs to be evaluated iff the predicted mode and the world position estimated
    // by the current camera pose agree.
    if (length(ptLM.mode.position - inlierWorldPosition) < m_poseOptimizationInlierThreshold)
    {
      ptsForLM.push_back(ptLM);
    }
  }

  // Perform continuous optimization if we have enough points. See the paper for details.
  if (ptsForLM.size() > 3)
  {
    // Convert the 6 parameters to a format that alglib likes.
    const float *ksiF = candidateCameraPose.GetParams();
    double ksiD[6];

    // Cast to double
    for (int i = 0; i < 6; ++i) ksiD[i] = static_cast<double>(ksiF[i]);

    alglib::real_1d_array ksi_;
    ksi_.setcontent(6, ksiD);

    // Parameters harcoded as in Valentin's paper.
    double differentiationStep = 0.0001;
    double epsf = 0;
    double epsg = 0.000001;
    double epsx = 0;
    alglib::ae_int_t maxits = 100;

    // Set up the optimiser.
    alglib::minlmstate state;
    alglib::minlmreport rep;
    alglib::minlmcreatev(6, 1, ksi_, differentiationStep, state);
    alglib::minlmsetcond(state, epsg, epsf, epsx, maxits);

    // Compute the energy and run the optimiser.
    double energyBefore, energyAfter;
    if (m_usePredictionCovarianceForPoseOptimization)
    {
      energyBefore = EnergyForContinuous3DOptimizationUsingFullCovariance(ptsForLM, candidateCameraPose);
      alglib::minlmoptimize(state, Continuous3DOptimizationUsingFullCovariance, call_after_each_step, &ptsForLM);
    }
    else
    {
      energyBefore = EnergyForContinuous3DOptimizationUsingL2(ptsForLM, candidateCameraPose);
      alglib::minlmoptimize(state, Continuous3DOptimizationUsingL2, call_after_each_step, &ptsForLM);
    }

    // Extract the results and update the SE3Pose accordingly.
    alglib::minlmresults(state, ksi_, rep);
    candidateCameraPose.SetFrom(ksi_[0], ksi_[1], ksi_[2], ksi_[3], ksi_[4], ksi_[5]);

    // Compute the final energy.
    if (m_usePredictionCovarianceForPoseOptimization)
    {
      energyAfter = EnergyForContinuous3DOptimizationUsingFullCovariance(ptsForLM, candidateCameraPose);
    }
    else
    {
      energyAfter = EnergyForContinuous3DOptimizationUsingL2(ptsForLM, candidateCameraPose);
    }

    // Store the updated pose iff the final energy is better than the initial one.
    if (energyAfter < energyBefore)
    {
      poseCandidate.cameraPose = candidateCameraPose.GetM();
      return true;
    }
  }

  // No optimisation ran or we got a worse energy at the end.
  return false;
}

} // namespace grove
