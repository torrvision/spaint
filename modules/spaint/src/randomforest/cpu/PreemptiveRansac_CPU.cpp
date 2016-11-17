/**
 * spaint: PreemptiveRansac_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "randomforest/cpu/PreemptiveRansac_CPU.h"
#include "randomforest/shared/PreemptiveRansac_Shared.h"

#include "util/MemoryBlockFactory.h"

#include <Eigen/Dense>

using namespace tvgutil;

namespace spaint
{
PreemptiveRansac_CPU::PreemptiveRansac_CPU() :
    PreemptiveRansac()
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomGenerators = mbf.make_block<CPURNG>(m_nbMaxPoseCandidates);
  m_rngSeed = 42;

  init_random();
}

void PreemptiveRansac_CPU::init_random()
{
  CPURNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CPU);

  // Initialize random states
  for (int i = 0; i < m_nbMaxPoseCandidates; ++i)
  {
    randomGenerators[i].reset(m_rngSeed + i);
  }
}

void PreemptiveRansac_CPU::generate_pose_candidates()
{
  const Vector2i imgSize = m_featureImage->noDims;
  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  CPURNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CPU);
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

  m_poseCandidates->dataSize = 0;

#ifdef WITH_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int candidateIdx = 0; candidateIdx < m_nbMaxPoseCandidates;
      ++candidateIdx)
  {
    PoseCandidate candidate;
    candidate.cameraId = candidateIdx;

    bool valid = preemptive_ransac_generate_candidate(features, predictions,
        imgSize, randomGenerators[candidateIdx], candidate,
        m_useAllModesPerLeafInPoseHypothesisGeneration,
        m_checkMinDistanceBetweenSampledModes,
        m_minSquaredDistanceBetweenSampledModes,
        m_checkRigidTransformationConstraint,
        m_translationErrorMaxForCorrectPose);

    if (valid)
    {
      int candidateIdx;

#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
      candidateIdx = m_poseCandidates->dataSize++;
      poseCandidates[candidateIdx] = candidate;
    }
  }

  compute_candidate_pose_kabsch();
}

void PreemptiveRansac_CPU::compute_candidate_pose_kabsch()
{
  const RGBDPatchFeature *features = m_featureImage->GetData(MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictions = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

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

void PreemptiveRansac_CPU::sample_inlier_candidates(bool useMask)
{
  const Vector2i imgSize = m_featureImage->noDims;
  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);

  int *inlierMaskData = m_inliersMaskImage->GetData(MEMORYDEVICE_CPU);
  int *inlierIndicesData = m_inliersIndicesImage->GetData(MEMORYDEVICE_CPU);
  CPURNG *randomGenerators = m_randomGenerators->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (size_t sampleIdx = 0; sampleIdx < m_batchSizeRansac; ++sampleIdx)
  {
    int sampledLinearIdx = -1;

    if (useMask)
    {
      sampledLinearIdx = preemptive_ransac_sample_inlier<true>(
          patchFeaturesData, predictionsData, imgSize,
          randomGenerators[sampleIdx], inlierMaskData);
    }
    else
    {
      sampledLinearIdx = preemptive_ransac_sample_inlier<false>(
          patchFeaturesData, predictionsData, imgSize,
          randomGenerators[sampleIdx]);
    }

    if (sampledLinearIdx >= 0)
    {
      size_t inlierIdx = 0;

#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
      inlierIdx = m_inliersIndicesImage->dataSize++;

      inlierIndicesData[inlierIdx] = sampledLinearIdx;
    }
  }
}

void PreemptiveRansac_CPU::compute_and_sort_energies()
{
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int p = 0; p < nbPoseCandidates; ++p)
  {
    compute_pose_energy(poseCandidates[p]);
  }

  // Sort by ascending energy
  std::sort(poseCandidates, poseCandidates + nbPoseCandidates);
}

void PreemptiveRansac_CPU::compute_pose_energy(PoseCandidate &candidate) const
{
  const RGBDPatchFeature *patchFeaturesData = m_featureImage->GetData(
      MEMORYDEVICE_CPU);
  const GPUForestPrediction *predictionsData = m_predictionsImage->GetData(
      MEMORYDEVICE_CPU);
  const size_t nbInliers = m_inliersIndicesImage->dataSize;
  const int *inliersData = m_inliersIndicesImage->GetData(MEMORYDEVICE_CPU);

  const float totalEnergy = preemptive_ransac_compute_candidate_energy(
      candidate.cameraPose, patchFeaturesData, predictionsData, inliersData,
      nbInliers);

  candidate.energy = totalEnergy / static_cast<float>(nbInliers);
}

void PreemptiveRansac_CPU::update_candidate_poses()
{
  PreemptiveRansac::update_candidate_poses();
}

}
