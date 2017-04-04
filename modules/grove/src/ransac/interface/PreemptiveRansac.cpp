/**
 * grove: PreemptiveRansac.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/interface/PreemptiveRansac.h"

#include <boost/lexical_cast.hpp>
#include <boost/timer/timer.hpp>

//#include <ceres/ceres.h>
//#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <alglib/optimization.h>
#include <omp.h>

#include <ORUtils/SE3Pose.h>

#include <itmx/MemoryBlockFactory.h>
using namespace itmx;

//#define ENABLE_TIMERS

namespace grove {

PreemptiveRansac::PreemptiveRansac() :
    m_timerCandidateGeneration("Candidate Generation"), m_timerFirstComputeEnergy(
        "First Energy Computation"), m_timerFirstTrim("First Trim"), m_timerTotal(
        "P-RANSAC Total")
{
  // Set params as in scoreforests
  m_nbPointsForKabschBoostrap = 3;
  m_useAllModesPerLeafInPoseHypothesisGeneration = true;
  m_checkMinDistanceBetweenSampledModes = true;
  m_minSquaredDistanceBetweenSampledModes = 0.3f * 0.3f;
//  m_checkRigidTransformationConstraint = false; // Speeds up a lot, was true in scoreforests
  m_checkRigidTransformationConstraint = true;
  m_translationErrorMaxForCorrectPose = 0.05f;
  m_batchSizeRansac = 500;
  m_trimKinitAfterFirstEnergyComputation = 64;
//  m_trimKinitAfterFirstEnergyComputation = 1024;
  m_poseUpdate = true; // original
//  m_poseUpdate = false; // faster, might be OK
  m_usePredictionCovarianceForPoseOptimization = true; // original implementation
//  m_usePredictionCovarianceForPoseOptimization = false;
  m_poseOptimizationInlierThreshold = 0.2f;

  const MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_nbMaxPoseCandidates = 1024;
  m_poseCandidates = mbf.make_block<PoseCandidate>(m_nbMaxPoseCandidates);

  m_nbMaxInliers = 3000; // 500 per ransac iteration, starting from 64, not 1024.
  m_inliersIndicesBlock = mbf.make_block<int>(m_nbMaxInliers);
  m_inliersMaskImage = mbf.make_image<int>();

#ifdef ENABLE_TIMERS
  m_printTimers = true;
#else
  m_printTimers = false;
#endif

  m_printTimers = true;

  for (int i = 1; i <= 6; ++i)
  {
    m_timerInlierSampling.push_back(
        AverageTimer("Inlier Sampling " + boost::lexical_cast<std::string>(i)));
    m_timerOptimisation.push_back(
        AverageTimer("Optimisation " + boost::lexical_cast<std::string>(i)));
    m_timerComputeEnergy.push_back(
        AverageTimer(
            "Energy Computation " + boost::lexical_cast<std::string>(i)));
  }
}

void printTimer(const PreemptiveRansac::AverageTimer &timer)
{
  std::cout << timer.name() << ": " << timer.count() << " times, avg: "
      << (timer.count() > 0 ?
          boost::chrono::duration_cast<boost::chrono::milliseconds>(
              timer.average_duration()) :
          boost::chrono::milliseconds()) << ".\n";
}

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

int PreemptiveRansac::get_min_nb_required_points() const
{
  return std::max(m_nbPointsForKabschBoostrap, m_batchSizeRansac);
}

boost::optional<PoseCandidate> PreemptiveRansac::estimate_pose(
    const Keypoint3DColourImage_CPtr &keypoints,
    const ScorePredictionsImage_CPtr &forestPredictions)
{
  m_timerTotal.start();

  // Copy keypoint and references in the local variables, to avoid explicitely passing them to every function.
  m_keypointsImage = keypoints;
  m_predictionsImage = forestPredictions;

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

  if (m_trimKinitAfterFirstEnergyComputation < m_poseCandidates->dataSize)
  {
    m_timerFirstTrim.start();
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "first trim: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "sample inliers: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      sample_inlier_candidates(false); // no mask for the first pass
    }

    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "compute and sort energies: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      m_timerFirstComputeEnergy.start();
      compute_and_sort_energies();
      m_timerFirstComputeEnergy.stop();
    }

    m_poseCandidates->dataSize = m_trimKinitAfterFirstEnergyComputation;
    m_timerFirstTrim.stop();
  }

  //  std::cout << candidates.size() << " candidates remaining." << std::endl;
  //  std::cout << "Premptive RANSAC" << std::endl;

#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "ransac: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

  // Reset inlier mask (and inliers)
  m_inliersMaskImage->ChangeDims(m_keypointsImage->noDims); // Happens only once
  m_inliersMaskImage->Clear();
  m_inliersIndicesBlock->dataSize = 0;

  int iteration = 0;

  while (m_poseCandidates->dataSize > 1)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "ransac iteration: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    //    std::cout << candidates.size() << " camera remaining" << std::endl;

    m_timerInlierSampling[iteration].start();
    sample_inlier_candidates(true);
    m_timerInlierSampling[iteration].stop();

    if (m_poseUpdate)
    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "continuous optimization: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      m_timerOptimisation[iteration].start();
      update_candidate_poses();
      m_timerOptimisation[iteration].stop();
    }

    m_timerComputeEnergy[iteration].start();
    compute_and_sort_energies();
    m_timerComputeEnergy[iteration].stop();

    // Remove half of the candidates with the worse energies
    m_poseCandidates->dataSize /= 2;

    ++iteration;
  }

  m_timerTotal.stop();

  PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  return m_poseCandidates->dataSize > 0 ? candidates[0] : boost::optional<PoseCandidate>();
}

void PreemptiveRansac::get_best_poses(
    std::vector<PoseCandidate> &poseCandidates) const
{
  // Don't check dataSize since it will be likely 1
  const PoseCandidate *candidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

  for (size_t poseIdx = 0; poseIdx < m_trimKinitAfterFirstEnergyComputation;
      ++poseIdx)
  {
    poseCandidates.push_back(candidates[poseIdx]);
  }
}

namespace
{
void Kabsch(Eigen::Matrix3f &P, Eigen::Matrix3f &Q, Eigen::Vector3f &weights,
    Eigen::Matrix3f &resRot, Eigen::Vector3f &resTrans)
{
  const int D = P.rows();  // dimension of the space
  const Eigen::Vector3f normalizedWeights = weights / weights.sum();

  // Centroids
  const Eigen::Vector3f p0 = P * normalizedWeights;
  const Eigen::Vector3f q0 = Q * normalizedWeights;
  const Eigen::Vector3f v1 = Eigen::Vector3f::Ones();

  const Eigen::Matrix3f P_centred = P - p0 * v1.transpose(); // translating P to center the origin
  const Eigen::Matrix3f Q_centred = Q - q0 * v1.transpose(); // translating Q to center the origin

      // Covariance between both matrices
  const Eigen::Matrix3f C = P_centred * normalizedWeights.asDiagonal()
      * Q_centred.transpose();

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

void Kabsch(Eigen::Matrix3f &P, Eigen::Matrix3f &Q, Eigen::Matrix3f &resRot,
    Eigen::Vector3f &resTrans)
{
  Eigen::Vector3f weights = Eigen::Vector3f::Ones();
  Kabsch(P, Q, weights, resRot, resTrans);
}

Eigen::Matrix4f Kabsch(Eigen::Matrix3f &P, Eigen::Matrix3f &Q)
{
  Eigen::Matrix3f resRot;
  Eigen::Vector3f resTrans;

  Kabsch(P, Q, resRot, resTrans);

  // recompose R + t in Rt
  Eigen::Matrix4f res;
  res.block<3, 3>(0, 0) = resRot;
  res.block<3, 1>(0, 3) = resTrans;
  return res;
}
}

void PreemptiveRansac::update_candidate_poses()
{
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < nbPoseCandidates; ++i)
  {
    update_candidate_pose(poseCandidates[i]);
  }
}

void PreemptiveRansac::compute_candidate_poses_kabsch()
{
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

//  std::cout << "Generated " << nbPoseCandidates << " candidates." << std::endl;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (size_t candidateIdx = 0; candidateIdx < nbPoseCandidates; ++candidateIdx)
  {
    PoseCandidate &candidate = poseCandidates[candidateIdx];

    Eigen::Matrix3f localPoints;
    Eigen::Matrix3f worldPoints;

    for (int s = 0; s < PoseCandidate::KABSCH_POINTS; ++s)
    {
      localPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(
          candidate.pointsCamera[s].v);
      worldPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(
          candidate.pointsWorld[s].v);
    }

    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = Kabsch(localPoints,
        worldPoints);
  }
}

namespace
{
struct PointForLM
{
  Vector3f point;
  Mode3DColour mode;

  PointForLM()
  {
  }

  PointForLM(const Vector3f &pt, const Mode3DColour &md) :
      point(pt), mode(md)
  {
  }

  // If we use ceres uncomment
#if 0
  template<typename T>
  inline bool operator()(const T * const camera, T *residual) const
  {
    T pointT[3];
    pointT[0] = T(point.x);
    pointT[1] = T(point.y);
    pointT[2] = T(point.z);

    T transformedPt[3];

    // rotation
    ceres::AngleAxisRotatePoint(&camera[3], pointT, transformedPt);
    // translation
    transformedPt[0] += camera[0];
    transformedPt[1] += camera[1];
    transformedPt[2] += camera[2];

    T modePosition[3];
    modePosition[0] = T(mode.position.x);
    modePosition[1] = T(mode.position.y);
    modePosition[2] = T(mode.position.z);

    T pointDiff[3];
    pointDiff[0] = transformedPt[0] - modePosition[0];
    pointDiff[1] = transformedPt[1] - modePosition[1];
    pointDiff[2] = transformedPt[2] - modePosition[2];

    T modeInverseCovariance[9];
    // Col major to row major to perform dot product later
    modeInverseCovariance[0] = T(mode.positionInvCovariance.m[0]);
    modeInverseCovariance[1] = T(mode.positionInvCovariance.m[3]);
    modeInverseCovariance[2] = T(mode.positionInvCovariance.m[6]);

    modeInverseCovariance[3] = T(mode.positionInvCovariance.m[1]);
    modeInverseCovariance[4] = T(mode.positionInvCovariance.m[4]);
    modeInverseCovariance[5] = T(mode.positionInvCovariance.m[7]);

    modeInverseCovariance[6] = T(mode.positionInvCovariance.m[2]);
    modeInverseCovariance[7] = T(mode.positionInvCovariance.m[5]);
    modeInverseCovariance[8] = T(mode.positionInvCovariance.m[8]);

    // compute the mahalanobis square distance
    T firstDot[3];
    firstDot[0] = ceres::DotProduct(&modeInverseCovariance[0], pointDiff);
    firstDot[1] = ceres::DotProduct(&modeInverseCovariance[3], pointDiff);
    firstDot[2] = ceres::DotProduct(&modeInverseCovariance[6], pointDiff);

    // Finish computing the distance
    residual[0] = ceres::DotProduct(pointDiff, firstDot);
//    residual[0] = ceres::DotProduct(pointDiff, pointDiff);
    return true;
  }
#endif
};

typedef std::vector<PointForLM> PointsForLM;

//struct PointsForLM
//{
//  std::vector<Vector3f, GPUForestMode>> pts;
////  PointsForLM(int nbPts) :
////      pts(nbPts), blurred_img(NULL)
////  {
////  }
////  ~PointsForLM()
////  {
////  }
////  std::vector<
////      std::pair<std::vector<Eigen::VectorXd>,
////          std::vector<PredictedGaussianMean *>>> pts;
////  GaussianAggregatedRGBImage *blurred_img;
//};

static double EnergyForContinuous3DOptimizationUsingFullCovariance(
    const PointsForLM &pts, const ORUtils::SE3Pose &candidateCameraPose)
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

static void Continuous3DOptimizationUsingFullCovariance(
    const alglib::real_1d_array &ksi, alglib::real_1d_array &fi, void *ptr)
{
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4],
      ksi[5]);

  fi[0] = EnergyForContinuous3DOptimizationUsingFullCovariance(*ptsLM,
      testPose);
}

/***************************************************/
/* Routines to optimize the sum of 3D L2 distances */
/***************************************************/

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

static void Continuous3DOptimizationUsingL2(const alglib::real_1d_array &ksi,
    alglib::real_1d_array &fi, void *ptr)
{
  const PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
  const ORUtils::SE3Pose testPose(ksi[0], ksi[1], ksi[2], ksi[3], ksi[4],
      ksi[5]);

  fi[0] = EnergyForContinuous3DOptimizationUsingL2(*ptsLM, testPose);
}

static void call_after_each_step(const alglib::real_1d_array &x, double func,
    void *ptr)
{
  return;
}
}

bool PreemptiveRansac::update_candidate_pose(PoseCandidate &poseCandidate) const
{
  const Keypoint3DColour *keypointsData = m_keypointsImage->GetData(
      MEMORYDEVICE_CPU);
  const Prediction3DColour *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);
  const size_t nbInliers = m_inliersIndicesBlock->dataSize;
  const int *inliersData = m_inliersIndicesBlock->GetData(MEMORYDEVICE_CPU);

  ORUtils::SE3Pose candidateCameraPose(poseCandidate.cameraPose);

#if 0
  // Build the problem.
  ceres::Problem problem;

  double cameraState[6];

  for (int i = 0; i < 6; ++i)
  {
    cameraState[i] = candidateCameraPose.GetParams()[i];
  }

  for (int inlierIdx = 0; inlierIdx < nbInliers; ++inlierIdx)
  {
    const int inlierLinearIdx = inliersData[inlierIdx];
    const Vector3f inlierCameraPosition =
        keypointsData[inlierLinearIdx].position;
    const Vector3f inlierWorldPosition = candidateCameraPose.GetM()
        * inlierCameraPosition;
    const ScorePrediction &prediction = predictionsData[inlierLinearIdx];

    // Find the best mode
    // (do not rely on the one stored in the inlier because for the randomly sampled inliers it's not set)
    int bestModeIdx = prediction.get_best_mode(inlierWorldPosition);
    if (bestModeIdx < 0 || bestModeIdx >= prediction.nbModes)
    throw std::runtime_error("best mode idx invalid.");// should have not been selected as inlier

    if (length(prediction.modes[bestModeIdx].position - inlierWorldPosition)
        < m_poseOptimizationInlierThreshold)
    {

      ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<
      PointForLM, 1, 6>(
          new PointForLM(inlierCameraPosition, prediction.modes[bestModeIdx]));

//      ceres::CostFunction *costFunction = new ceres::NumericDiffCostFunction<
//          PointForLM, ceres::CENTRAL, 1, 6>(
//          new PointForLM(inlierCameraPosition, prediction.modes[bestModeIdx]));
      problem.AddResidualBlock(costFunction,
          new ceres::CauchyLoss(
              m_poseOptimizationInlierThreshold
              * m_poseOptimizationInlierThreshold), cameraState);
    }
  }

  if (problem.NumResidualBlocks() > 3)
  {
    ceres::Solver::Options options;
//    options.gradient_tolerance = 1e-6; // as in the alglib implementation
//    options.linear_solver_type = ceres::DENSE_QR;
//  options.minimizer_progress_to_stdout = true;
//  options.update_state_every_iteration = true;

    // Run the solver.
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

//    std::cout << summary.FullReport() << '\n';

    if (summary.termination_type == ceres::TerminationType::CONVERGENCE)
    {
      candidateCameraPose.SetFrom(cameraState[0], cameraState[1],
          cameraState[2], cameraState[3], cameraState[4], cameraState[5]);
      poseCandidate.cameraPose = candidateCameraPose.GetM();

      // Output a report.
//    std::cout << summary.BriefReport() << "\n";

      return true;
    }
  }

  return false;

#else

  PointsForLM ptsForLM;
  for (size_t inlierIdx = 0; inlierIdx < nbInliers; ++inlierIdx)
  {
    const int inlierLinearIdx = inliersData[inlierIdx];
    const Vector3f inlierCameraPosition =
        keypointsData[inlierLinearIdx].position;
    const Vector3f inlierWorldPosition = candidateCameraPose.GetM()
        * inlierCameraPosition;
    const Prediction3DColour &prediction = predictionsData[inlierLinearIdx];

    PointForLM ptLM;
    // The assumption is that the inlier is valid (checked before)
    ptLM.point = inlierCameraPosition;

    // Find the best mode
    // (do not rely on the one stored in the inlier because for the randomly sampled inliers it's not set)
    const int bestModeIdx = prediction.get_best_mode(inlierWorldPosition);
    if (bestModeIdx < 0 || bestModeIdx >= prediction.nbModes)
      throw std::runtime_error("best mode idx invalid."); // should have not been selected as inlier
    ptLM.mode = prediction.modes[bestModeIdx];

    if (length(ptLM.mode.position - inlierWorldPosition)
        < m_poseOptimizationInlierThreshold)
    {
      ptsForLM.push_back(ptLM);
    }
  }

// Continuous optimization
  if (ptsForLM.size() > 3)
  {
    const float *ksiF = candidateCameraPose.GetParams();
    double ksiD[6];

    // Cast to double
    for (int i = 0; i < 6; ++i)
      ksiD[i] = ksiF[i];

    alglib::real_1d_array ksi_;
    ksi_.setcontent(6, ksiD);

//    Eigen::MatrixXd candidateCameraPoseD = candidateCameraPose.cast<double>();
//
//    Eigen::VectorXd ksivd = Helpers::LieGroupToLieAlgebraSE3(
//        candidateCameraPoseD);
//
//    for (int i = 0; i < 6; ++i)
//    {
//      ksiD[i] = ksivd(i);
//    }
//
//    ksi_.setcontent(6, ksiD);

    alglib::minlmstate state;
    alglib::minlmreport rep;

    double differentiationStep = 0.0001;
    alglib::minlmcreatev(6, 1, ksi_, differentiationStep, state);

    double epsg = 0.000001;
    double epsf = 0;
    double epsx = 0;
    alglib::ae_int_t maxits = 100;
    alglib::minlmsetcond(state, epsg, epsf, epsx, maxits);

    double energyBefore, energyAfter;
    if (m_usePredictionCovarianceForPoseOptimization)
    {
      energyBefore = EnergyForContinuous3DOptimizationUsingFullCovariance(
          ptsForLM, candidateCameraPose);
      alglib::minlmoptimize(state, Continuous3DOptimizationUsingFullCovariance,
          call_after_each_step, &ptsForLM);
    }
    else
    {
      energyBefore = EnergyForContinuous3DOptimizationUsingL2(ptsForLM,
          candidateCameraPose);
      alglib::minlmoptimize(state, Continuous3DOptimizationUsingL2,
          call_after_each_step, &ptsForLM);
    }

    alglib::minlmresults(state, ksi_, rep);

    candidateCameraPose.SetFrom(ksi_[0], ksi_[1], ksi_[2], ksi_[3], ksi_[4],
        ksi_[5]);

//    memcpy(ksiD, ksi_.getcontent(), sizeof(double) * 6);
//    for (int i = 0; i < 6; ++i)
//    {
//      ksivd(i) = ksiD[i];
//    }
//    Eigen::MatrixXd updatedCandidateCameraPoseD =
//        Helpers::LieAlgebraToLieGroupSE3(ksivd);

    if (m_usePredictionCovarianceForPoseOptimization)
    {
      energyAfter = EnergyForContinuous3DOptimizationUsingFullCovariance(
          ptsForLM, candidateCameraPose);
    }
    else
    {
      energyAfter = EnergyForContinuous3DOptimizationUsingL2(ptsForLM,
          candidateCameraPose);
    }

    if (energyAfter < energyBefore)
    {
      poseCandidate.cameraPose = candidateCameraPose.GetM();
      return true;
    }
  }

////////////////////////////

//  std::vector<std::pair<int, int>> &samples = std::get < 1 > (poseCandidate);
//
//  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
//      MEMORYDEVICE_CPU);
//
//  PointsForLM ptsForLM(0);
//
//  for (int s = 0; s < samples.size(); ++s)
//  {
//    const int x = samples[s].first % m_featureImage->noDims.width;
//    const int y = samples[s].first / m_featureImage->noDims.width;
//    const int linearizedIdx = samples[s].first;
//
//    std::pair<std::vector<Eigen::VectorXd>, std::vector<PredictedGaussianMean *>> pt;
//
//    Eigen::VectorXf pixelLocalCoordinates = Eigen::Map<const Eigen::Vector4f>(
//        patchFeaturesData[linearizedIdx].position.v);
//
//    pt.first.push_back(pixelLocalCoordinates.cast<double>());
//    // Eigen::VectorXf  projectedPixel = candidateCameraPose * pixelLocalCoordinates;
//    Eigen::VectorXd projectedPixel = (candidateCameraPose
//        * pixelLocalCoordinates).cast<double>();
//
//    boost::shared_ptr<EnsemblePredictionGaussianMean> epgm =
//        m_featurePredictions[linearizedIdx];
//
//    int argmax = epgm->GetArgMax3D(projectedPixel, 0);
//    if (argmax == -1)
//      continue;
//    pt.second.push_back(epgm->_modes[argmax][0]);
//
//    if ((epgm->_modes[argmax][0]->_mean
//        - Helpers::ConvertWorldCoordinatesFromHomogeneousCoordinates(
//            projectedPixel)).norm() < 0.2)
//      ptsForLM.pts.push_back(pt);
//  }

  return false;
#endif
}

}
