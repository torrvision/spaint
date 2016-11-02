/**
 * spaint: GPURansac.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/interface/GPURansac.h"

#include <random>

#include <boost/timer/timer.hpp>

#include <Eigen/Dense>
#include <libalglib/optimization.h>
#include <omp.h>
#include "ORUtils/SE3Pose.h"

#include "util/MemoryBlockFactory.h"

//#define ENABLE_TIMERS

namespace spaint
{
GPURansac::GPURansac()
{
  // Set params as in scoreforests
  m_nbPointsForKabschBoostrap = 3;
  m_useAllModesPerLeafInPoseHypothesisGeneration = true;
  m_checkMinDistanceBetweenSampledModes = true;
  m_minDistanceBetweenSampledModes = 0.3f;
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

  m_poseCandidates = MemoryBlockFactory::instance().make_block<PoseCandidates>(
      1);
}

GPURansac::~GPURansac()
{
}

int GPURansac::get_min_nb_required_points() const
{
  return std::max(m_nbPointsForKabschBoostrap, m_batchSizeRansac);
}

boost::optional<PoseCandidate> GPURansac::estimate_pose(
    const RGBDPatchFeatureImage_CPtr &features,
    const GPUForestPredictionsImage_CPtr &forestPredictions)
{
  std::mt19937 random_engine;

  m_featureImage = features;
  m_predictionsImage = forestPredictions;

  m_featureImage->UpdateHostFromDevice(); // Need the features on the host for now
  m_predictionsImage->UpdateHostFromDevice(); // Also the predictions

//  std::vector<PoseCandidate> candidates;
  PoseCandidate *candidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->candidates;
  int &nbPoseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->nbCandidates;

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "generating initial candidates: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    generate_pose_candidates();
  }

  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "kabsch: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    compute_candidate_pose_kabsch();
  }

//  std::cout << "Generated " << m_nbPoseCandidates << " initial candidates."
//      << std::endl;

  if (m_trimKinitAfterFirstEnergyComputation < nbPoseCandidates)
  {
#ifdef ENABLE_TIMERS
    boost::timer::auto_cpu_timer t(6,
        "first trim: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
    int nbSamplesPerCamera = candidates[0].nbInliers;
    std::vector<Vector2i> sampledPixelIdx;
    std::vector<bool> dummy_vector;

    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "sample pixels: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      sample_pixels_for_ransac(dummy_vector, sampledPixelIdx, random_engine,
          m_batchSizeRansac);
    }

    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "update inliers: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      update_inliers_for_optimization(sampledPixelIdx);
    }

    {
#ifdef ENABLE_TIMERS
      boost::timer::auto_cpu_timer t(6,
          "compute and sort energies: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif
      compute_and_sort_energies();
    }

    nbPoseCandidates = m_trimKinitAfterFirstEnergyComputation;

    if (m_trimKinitAfterFirstEnergyComputation > 1)
    {
      for (int p = 0; p < nbPoseCandidates; ++p)
      {
        PoseCandidate &candidate = candidates[p];
        if (candidate.nbInliers > nbSamplesPerCamera)
          candidate.nbInliers = nbSamplesPerCamera;
      }
    }
  }

  //  std::cout << candidates.size() << " candidates remaining." << std::endl;
  //  std::cout << "Premptive RANSAC" << std::endl;

#ifdef ENABLE_TIMERS
  boost::timer::auto_cpu_timer t(6,
      "ransac: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
#endif

  std::vector<bool> maskSampledPixels(m_featureImage->dataSize, false);

  float iteration = 0.0f;

  while (nbPoseCandidates > 1)
  {
    //    boost::timer::auto_cpu_timer t(
    //        6, "ransac iteration: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
    ++iteration;
    //    std::cout << candidates.size() << " camera remaining" << std::endl;

    std::vector<Vector2i> sampledPixelIdx;
    sample_pixels_for_ransac(maskSampledPixels, sampledPixelIdx, random_engine,
        m_batchSizeRansac);

    //    std::cout << "Updating inliers to each pose candidate..." << std::endl;
    update_inliers_for_optimization(sampledPixelIdx);

    if (m_poseUpdate)
    {
      update_candidate_poses();
    }

    compute_and_sort_energies();

    // Remove half of the candidates with the worse energies
    nbPoseCandidates /= 2;
  }

  return nbPoseCandidates > 0 ? candidates[0] : boost::optional<PoseCandidate>();
}

void GPURansac::generate_pose_candidates()
{
  const int nbThreads = 12;

  PoseCandidate *poseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->candidates;
  int &nbPoseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->nbCandidates;

  nbPoseCandidates = 0;

  std::vector<std::mt19937> engs(nbThreads);
  for (int i = 0; i < nbThreads; ++i)
  {
    engs[i].seed(static_cast<unsigned int>(i + 1));
  }

  omp_set_num_threads(nbThreads);

//  std::cout << "Generating pose candidates Kabsch" << std::endl;
#pragma omp parallel for
  for (int i = 0; i < PoseCandidates::MAX_CANDIDATES; ++i)
  {
    int threadId = omp_get_thread_num();
    PoseCandidate candidate;

    if (hypothesize_pose(candidate, engs[threadId]))
    {
      if (candidate.nbInliers > 0) // Has some inliers
      {
        candidate.cameraId = i;

#pragma omp critical
        {
          poseCandidates[nbPoseCandidates++] = candidate;
        }
      }
    }
  }
}

namespace
{
void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::VectorXf &weights,
    Eigen::MatrixXf &resRot, Eigen::VectorXf &resTrans)
{
  if (P.cols() != Q.cols() || P.rows() != Q.rows())
    throw std::runtime_error("Kabsch: P and Q have different dimensions");
  int D = P.rows();  // dimension of the space
  int N = P.cols();  // number of points
  Eigen::VectorXf normalizedWeights = Eigen::VectorXf(weights.size());

  // normalize weights to sum to 1
  {
    float sumWeights = 0;
    for (int i = 0; i < weights.size(); ++i)
    {
      sumWeights += weights(i);
    }
    normalizedWeights = weights * (1.0f / sumWeights);
  }

  // Centroids
  Eigen::VectorXf p0 = P * normalizedWeights;
  Eigen::VectorXf q0 = Q * normalizedWeights;
  Eigen::VectorXf v1 = Eigen::VectorXf::Ones(N);

  Eigen::MatrixXf P_centred = P - p0 * v1.transpose(); // translating P to center the origin
  Eigen::MatrixXf Q_centred = Q - q0 * v1.transpose(); // translating Q to center the origin

      // Covariance between both matrices
  Eigen::MatrixXf C = P_centred * normalizedWeights.asDiagonal()
      * Q_centred.transpose();

  // SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(C,
      Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::MatrixXf V = svd.matrixU();
  Eigen::VectorXf S = svd.singularValues();
  Eigen::MatrixXf W = svd.matrixV();
  Eigen::MatrixXf I = Eigen::MatrixXf::Identity(D, D);

  if ((V * W.transpose()).determinant() < 0)
    I(D - 1, D - 1) = -1;

  // Recover the rotation and translation
  resRot = W * I * V.transpose();
  resTrans = q0 - resRot * p0;

  return;
}

void Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q, Eigen::MatrixXf &resRot,
    Eigen::VectorXf &resTrans)
{
  Eigen::VectorXf weights = Eigen::VectorXf::Ones(P.cols());
  Kabsch(P, Q, weights, resRot, resTrans);
}

Eigen::Matrix4f Kabsch(Eigen::MatrixXf &P, Eigen::MatrixXf &Q)
{
  Eigen::MatrixXf resRot;
  Eigen::VectorXf resTrans;

  Kabsch(P, Q, resRot, resTrans);

  // recompose R + t in Rt
  Eigen::Matrix4f res;
  res.block<3, 3>(0, 0) = resRot;
  res.block<3, 1>(0, 3) = resTrans;
  return res;
}
}

bool GPURansac::hypothesize_pose(PoseCandidate &res, std::mt19937 &eng)
{
  Eigen::MatrixXf worldPoints(3, m_nbPointsForKabschBoostrap);
  Eigen::MatrixXf localPoints(3, m_nbPointsForKabschBoostrap);

  std::uniform_int_distribution<int> col_index_generator(0,
      m_featureImage->noDims.width - 1);
  std::uniform_int_distribution<int> row_index_generator(0,
      m_featureImage->noDims.height - 1);

  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  bool foundIsometricMapping = false;
  const int maxIterationsOuter = 20;
  int iterationsOuter = 0;

  while (!foundIsometricMapping && iterationsOuter < maxIterationsOuter)
  {
    ++iterationsOuter;
    std::vector<std::tuple<int, int, int>> selectedPixelsAndModes;

    const int maxIterationsInner = 6000;
    int iterationsInner = 0;
    while (selectedPixelsAndModes.size() != m_nbPointsForKabschBoostrap
        && iterationsInner < maxIterationsInner)
    {
      ++iterationsInner;

      const int x = col_index_generator(eng);
      const int y = row_index_generator(eng);
      const int linearFeatureIdx = y * m_featureImage->noDims.width + x;
      const RGBDPatchFeature &selectedFeature =
          patchFeaturesData[linearFeatureIdx];

      if (selectedFeature.position.w < 0.f) // Invalid feature
        continue;

      const GPUForestPrediction &selectedPrediction =
          predictionsData[linearFeatureIdx];

      if (selectedPrediction.nbModes == 0)
        continue;

      int selectedModeIdx = 0;
      if (m_useAllModesPerLeafInPoseHypothesisGeneration)
      {
        std::uniform_int_distribution<int> mode_generator(0,
            selectedPrediction.nbModes - 1);
        selectedModeIdx = mode_generator(eng);
      }

      // This is the first pixel, check that the pixel colour corresponds with the selected mode
      if (selectedPixelsAndModes.empty())
      {
        const Vector3u colourDiff = selectedFeature.colour.toVector3().toUChar()
            - selectedPrediction.modes[selectedModeIdx].colour;
        const bool consistentColour = abs(colourDiff.x) <= 30
            && abs(colourDiff.y) <= 30 && abs(colourDiff.z) <= 30;

        if (!consistentColour)
          continue;
      }

      // if (false)
      if (m_checkMinDistanceBetweenSampledModes)
      {
        const Vector3f worldPt =
            selectedPrediction.modes[selectedModeIdx].position;

        // Check that this mode is far enough from the other modes
        bool farEnough = true;

        for (size_t idxOther = 0; idxOther < selectedPixelsAndModes.size();
            ++idxOther)
        {
          int xOther, yOther, modeIdxOther;
          std::tie(xOther, yOther, modeIdxOther) =
              selectedPixelsAndModes[idxOther];

          const int linearIdxOther = yOther * m_featureImage->noDims.width
              + xOther;
          const GPUForestPrediction &predOther = predictionsData[linearIdxOther];

          Vector3f worldPtOther = predOther.modes[modeIdxOther].position;

          float distOther = length(worldPtOther - worldPt);
          if (distOther < m_minDistanceBetweenSampledModes)
          {
            farEnough = false;
            break;
          }
        }

        if (!farEnough)
          continue;
      }

      // isometry?
//       if (false)
      // if (true)
      if (m_checkRigidTransformationConstraint)
      {
        bool violatesConditions = false;

        for (size_t m = 0;
            m < selectedPixelsAndModes.size() && !violatesConditions; ++m)
        {
          int xFirst, yFirst, modeIdxFirst;
          std::tie(xFirst, yFirst, modeIdxFirst) = selectedPixelsAndModes[m];

          const int linearIdxOther = yFirst * m_featureImage->noDims.width
              + xFirst;
          const GPUForestPrediction &predFirst = predictionsData[linearIdxOther];

          const Vector3f worldPtFirst = predFirst.modes[modeIdxFirst].position;
          const Vector3f worldPtCur =
              selectedPrediction.modes[selectedModeIdx].position;

          float distWorld = length(worldPtFirst - worldPtCur);

          const Vector3f localPred =
              patchFeaturesData[linearIdxOther].position.toVector3();
          const Vector3f localCur = selectedFeature.position.toVector3();

          float distLocal = length(localPred - localCur);

          if (distLocal < m_minDistanceBetweenSampledModes)
            violatesConditions = true;

          if (std::abs(distLocal - distWorld)
              > 0.5f * m_translationErrorMaxForCorrectPose)
          {
            violatesConditions = true;
          }
        }

        if (violatesConditions)
          continue;
      }

      selectedPixelsAndModes.push_back(
          std::tuple<int, int, int>(x, y, selectedModeIdx));
//      iterationsInner = 0;
    }

//    std::cout << "Inner iterations: " << iterationsInner << std::endl;

    // Reached limit of iterations
    if (selectedPixelsAndModes.size() != m_nbPointsForKabschBoostrap)
      return false;

    // Populate resulting pose
    res.nbInliers = selectedPixelsAndModes.size();
    if (res.nbInliers != 3)
      throw std::runtime_error("Kabsch needs 3 points");

    for (size_t s = 0; s < selectedPixelsAndModes.size(); ++s)
    {
      int x, y, modeIdx;
      std::tie(x, y, modeIdx) = selectedPixelsAndModes[s];
      const int linearIdx = y * m_featureImage->noDims.width + x;
      const GPUForestPrediction &pred = predictionsData[linearIdx];

      Eigen::VectorXf localPt = Eigen::Map<const Eigen::Vector3f>(
          patchFeaturesData[linearIdx].position.v);

      Eigen::VectorXf worldPt = Eigen::Map<const Eigen::Vector3f>(
          pred.modes[modeIdx].position.v);

      for (int idx = 0; idx < 3; ++idx)
      {
        localPoints(idx, s) = localPt(idx);
        worldPoints(idx, s) = worldPt(idx);
      }

      res.inliers[s].linearIdx = linearIdx;
      res.inliers[s].modeIdx = modeIdx;
      res.inliers[s].energy = 0.f;
    }

    {
//#ifdef ENABLE_TIMERS
//      boost::timer::auto_cpu_timer t(6,
//          "kabsch: %ws wall, %us user + %ss system = %ts CPU (%p%)\n");
//#endif
      Eigen::Map<Eigen::Matrix4f>(res.cameraPose.m) = Kabsch(localPoints,
          worldPoints);
    }

    foundIsometricMapping = true;

    res.energy = 0.f;
    res.cameraId = -1;
  }

  if (iterationsOuter < maxIterationsOuter)
    return true;

  return false;
}

void GPURansac::compute_candidate_pose_kabsch()
{
  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);
  const int nbPoseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->nbCandidates;
  PoseCandidate *poseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->candidates;

//  std::cout << "Generated " << nbPoseCandidates << " candidates." << std::endl;

#pragma omp parallel for
  for (int candidateIdx = 0; candidateIdx < nbPoseCandidates; ++candidateIdx)
  {
    PoseCandidate &candidate = poseCandidates[candidateIdx];

    Eigen::MatrixXf localPoints(3, candidate.nbInliers);
    Eigen::MatrixXf worldPoints(3, candidate.nbInliers);
    for (int s = 0; s < candidate.nbInliers; ++s)
    {
      const int linearIdx = candidate.inliers[s].linearIdx;
      const int modeIdx = candidate.inliers[s].modeIdx;
      const GPUForestPrediction &pred = predictions[linearIdx];

      localPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(
          features[linearIdx].position.v);
      worldPoints.col(s) = Eigen::Map<const Eigen::Vector3f>(
          pred.modes[modeIdx].position.v);
    }

    Eigen::Map<Eigen::Matrix4f>(candidate.cameraPose.m) = Kabsch(localPoints,
        worldPoints);
  }
}

void GPURansac::sample_pixels_for_ransac(std::vector<bool> &maskSampledPixels,
    std::vector<Vector2i> &sampledPixelIdx, std::mt19937 &eng, int batchSize)
{
  std::uniform_int_distribution<int> col_index_generator(0,
      m_featureImage->noDims.width - 1);
  std::uniform_int_distribution<int> row_index_generator(0,
      m_featureImage->noDims.height - 1);

  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  for (int i = 0; i < batchSize; ++i)
  {
    bool validIndex = false;
    int innerIterations = 0;

    while (!validIndex && innerIterations++ < 50)
    {
      const Vector2i s(col_index_generator(eng), row_index_generator(eng));
      const int linearIdx = s.y * m_featureImage->noDims.width + s.x;

      if (patchFeaturesData[linearIdx].position.w >= 0.f)
      {
        const GPUForestPrediction &selectedPrediction =
            predictionsData[linearIdx];

        if (selectedPrediction.nbModes > 0)
        {
          validIndex = maskSampledPixels.empty()
              || !maskSampledPixels[linearIdx];

          if (validIndex)
          {
            sampledPixelIdx.push_back(s);

            if (!maskSampledPixels.empty())
              maskSampledPixels[linearIdx] = true;
          }
        }
      }
    }

    if (!validIndex)
    {
      std::cout << "Couldn't sample a valid pixel. Returning "
          << sampledPixelIdx.size() << "/" << batchSize << std::endl;
      break;
    }
  }
}

void GPURansac::update_inliers_for_optimization(
    const std::vector<Vector2i> &sampledPixelIdx)
{
  PoseCandidate *poseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->candidates;
  int &nbPoseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->nbCandidates;

#pragma omp parallel for
  for (int p = 0; p < nbPoseCandidates; ++p)
  {
    PoseCandidate &candidate = poseCandidates[p];

    // add all the samples as inliers
    for (size_t s = 0; s < sampledPixelIdx.size(); ++s)
    {
      const Vector2i &sample = sampledPixelIdx[s];
      const int linearIdx = sample.y * m_featureImage->noDims.width + sample.x;

      candidate.inliers[candidate.nbInliers++] = PoseCandidate::Inlier
      { linearIdx, -1, 0.f };
    }
  }
}

void GPURansac::compute_and_sort_energies()
{
//  int nbPoseProcessed = 0;
  PoseCandidate *poseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->candidates;
  int &nbPoseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->nbCandidates;

#pragma omp parallel for
  for (int p = 0; p < nbPoseCandidates; ++p)
  {
    //#pragma omp critical
    //    {
    //      //#pragma omp flush(nbPoseProcessed)
    //      //      Helpers::displayPercentage(nbPoseProcessed++, poseCandidates.size());
    //    }

    compute_pose_energy(poseCandidates[p]);
  }

// Sort by ascending energy
  std::sort(poseCandidates, poseCandidates + nbPoseCandidates,
      [] (const PoseCandidate &a, const PoseCandidate &b)
      { return a.energy < b.energy;});
}

void GPURansac::compute_pose_energy(PoseCandidate &candidate) const
{
  float totalEnergy = 0.0f;

  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  for (int s = 0; s < candidate.nbInliers; ++s)
  {
    const int linearIdx = candidate.inliers[s].linearIdx;
    const Vector3f localPixel =
        patchFeaturesData[linearIdx].position.toVector3();
    const Vector3f projectedPixel = candidate.cameraPose * localPixel;

    const GPUForestPrediction &pred = predictionsData[linearIdx];

    // eval individual energy
    float energy;
    int argmax = pred.get_best_mode_and_energy(projectedPixel, energy);

    // Has at least a valid mode
    if (argmax < 0)
    {
      // should have not been inserted in the inlier set
      std::cout << "prediction " << linearIdx
          << " has negative argmax, nbModes: " << pred.nbModes << std::endl;
      for (int i = 0; i < pred.nbModes; ++i)
      {
        auto &mode = pred.modes[i];
        std::cout << "Mode " << i << ": inliers: " << mode.nbInliers
            << "\npos: " << mode.position << "\ncol: " << mode.colour
            << "\ndet: " << mode.determinant << "\ninvcov: "
            << mode.positionInvCovariance << "\n" << std::endl;
      }
      throw std::runtime_error("prediction has no valid modes");
    }

    if (pred.modes[argmax].nbInliers == 0)
    {
      // the original implementation had a simple continue
      std::cout << "mode has no inliers" << std::endl;
      throw std::runtime_error("mode has no inliers");
    }

    energy /= static_cast<float>(pred.nbModes);
    energy /= static_cast<float>(pred.modes[argmax].nbInliers);

    if (energy < 1e-6f)
      energy = 1e-6f;
    energy = -log10f(energy);

    candidate.inliers[s].energy = energy;
    candidate.inliers[s].modeIdx = argmax;
    totalEnergy += energy;
  }

  candidate.energy = totalEnergy / static_cast<float>(candidate.nbInliers);
}

void GPURansac::update_candidate_poses()
{
  PoseCandidate *poseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->candidates;
  int &nbPoseCandidates =
      m_poseCandidates->GetData(MEMORYDEVICE_CPU)->nbCandidates;

//  int nbUpdated = 0;
#pragma omp parallel for
  for (int i = 0; i < nbPoseCandidates; ++i)
  {
    if (update_candidate_pose(poseCandidates[i]))
    {
      //#pragma omp atomic
      //      ++nbUpdated;
    }
  }
//  std::cout << nbUpdated << "/" << poseCandidates.size() << " updated cameras" << std::endl;
}

namespace
{
struct PointForLM
{
  Vector3f point;
  GPUForestMode mode;
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

//static double EnergyForContinuous3DOptimizationUsingL2(
//    std::vector<
//        std::pair<std::vector<Eigen::VectorXd>,
//            std::vector<PredictedGaussianMean *>>> &pts,
//    Eigen::MatrixXd &candidateCameraPoseD)
//{
//  double res = 0.0;
//  Eigen::VectorXd diff = Eigen::VectorXd::Zero(3);
//  Eigen::VectorXd transformedPthomogeneous(4);
//
//  for (size_t i = 0; i < pts.size(); ++i)
//  {
//    Helpers::Rigid3DTransformation(candidateCameraPoseD, pts[i].first[0],
//        transformedPthomogeneous);
//
//    for (int p = 0; p < 3; ++p)
//    {
//      diff(p) = transformedPthomogeneous(p) - pts[i].second[0]->_mean(p);
//    }
//
//    double err = diff.norm();
//    err *= err;
//    res += err;
//  }
//  return res;
//}
//
//static void Continuous3DOptimizationUsingL2(const alglib::real_1d_array &x,
//    alglib::real_1d_array &fi, void *ptr)
//{
//  PointsForLM *ptsLM = reinterpret_cast<PointsForLM *>(ptr);
//
//  std::vector<
//      std::pair<std::vector<Eigen::VectorXd>,
//          std::vector<PredictedGaussianMean *>>> &pts = ptsLM->pts;
//  // integrate the size of the clusters?
//  Eigen::VectorXd ksi(6);
//  memcpy(ksi.data(), x.getcontent(), 6 * sizeof(double));
//  /*for (int i = 0 ; i < 6 ; ++i)
//   {
//   ksi(i) = x[i];
//   }*/
//  Eigen::MatrixXd updatedCandidateCameraPoseD =
//      Helpers::LieAlgebraToLieGroupSE3(ksi);
//
//  fi[0] = EnergyForContinuous3DOptimizationUsingL2(pts,
//      updatedCandidateCameraPoseD);
//  return;
//}
static void call_after_each_step(const alglib::real_1d_array &x, double func,
    void *ptr)
{
  return;
}
}

bool GPURansac::update_candidate_pose(PoseCandidate &poseCandidate) const
{
  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  ORUtils::SE3Pose candidateCameraPose(poseCandidate.cameraPose);

  PointsForLM ptsForLM;
  for (int inlierIdx = 0; inlierIdx < poseCandidate.nbInliers; ++inlierIdx)
  {
    const PoseCandidate::Inlier &inlier = poseCandidate.inliers[inlierIdx];
    const Vector3f inlierCameraPosition =
        patchFeaturesData[inlier.linearIdx].position.toVector3();
    const Vector3f inlierWorldPosition = candidateCameraPose.GetM()
        * inlierCameraPosition;
    const GPUForestPrediction &prediction = predictionsData[inlier.linearIdx];

    PointForLM ptLM;
    // The assumption is that the inlier is valid (checked before)
    ptLM.point = inlierCameraPosition;

    // Find the best mode
    // (do not rely on the one stored in the inlier because for the randomly sampled inliers it's not set)
    int bestModeIdx = prediction.get_best_mode(inlierWorldPosition);
    if (bestModeIdx < 0 || bestModeIdx >= prediction.nbModes)
      throw std::runtime_error("best mode idx invalid."); // should have not been selected as inlier
    ptLM.mode = prediction.modes[bestModeIdx];

    if (length(ptLM.mode.position - inlierWorldPosition)
        < m_poseOptimizationInlierThreshold)
      ptsForLM.push_back(ptLM);
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
      throw std::runtime_error("Not updated yet");
//      energyBefore = EnergyForContinuous3DOptimizationUsingL2(ptsForLM.pts,
//          candidateCameraPoseD);
//      alglib::minlmoptimize(state, Continuous3DOptimizationUsingL2,
//          call_after_each_step, &ptsForLM);
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
      throw std::runtime_error("Not updated yet");
//      energyAfter = EnergyForContinuous3DOptimizationUsingL2(ptsForLM.pts,
//          updatedCandidateCameraPoseD);
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
}

}
