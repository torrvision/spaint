/**
 * grove: PreemptiveRansac_CPU.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ransac/cpu/PreemptiveRansac_CPU.h"
using namespace tvgutil;

#include <Eigen/Dense>

#include <itmx/base/MemoryBlockFactory.h>
using namespace itmx;

#include "ransac/shared/PreemptiveRansac_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

PreemptiveRansac_CPU::PreemptiveRansac_CPU(const SettingsContainer_CPtr& settings)
: PreemptiveRansac(settings)
{
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();
  m_rngs = mbf.make_block<CPURNG>(m_maxPoseCandidates);
  m_rngSeed = 42;

  init_random();
}

//#################### PROTECTED MEMBER FUNCTIONS ####################

void PreemptiveRansac_CPU::compute_energies_and_sort()
{
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);

  // Compute the energies for all pose candidates.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(size_t i = 0; i < nbPoseCandidates; ++i)
  {
    compute_pose_energy(poseCandidates[i]);
  }

  // Sort the candidates into non-increasing order of energy.
  std::sort(poseCandidates, poseCandidates + nbPoseCandidates);
}

void PreemptiveRansac_CPU::generate_pose_candidates()
{
  const Vector2i imgSize = m_keypointsImage->noDims;
  const Keypoint3DColour *keypoints = m_keypointsImage->GetData(MEMORYDEVICE_CPU);
  PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  const ScorePrediction *predictions = m_predictionsImage->GetData(MEMORYDEVICE_CPU);
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);

  // Reset the number of pose candidates.
  m_poseCandidates->dataSize = 0;

  // Generate at most m_maxPoseCandidates new pose candidates.
#ifdef WITH_OPENMP
  #pragma omp parallel for schedule(dynamic)
#endif
  for(uint32_t candidateIdx = 0; candidateIdx < m_maxPoseCandidates; ++candidateIdx)
  {
    // Try to generate a valid pose candidate.
    PoseCandidate candidate;
    bool valid = generate_pose_candidate(
      keypoints, predictions, imgSize, rngs[candidateIdx], candidate, m_maxCandidateGenerationIterations, m_useAllModesPerLeafInPoseHypothesisGeneration,
      m_checkMinDistanceBetweenSampledModes, m_minSquaredDistanceBetweenSampledModes, m_checkRigidTransformationConstraint, m_maxTranslationErrorForCorrectPose
    );

    // If we succeed, grab a unique index in the output array and store the candidate into the corresponding array element.
    if(valid)
    {
      size_t finalCandidateIdx;

    #ifdef WITH_OPENMP
      #pragma omp atomic capture
    #endif
      finalCandidateIdx = m_poseCandidates->dataSize++;

      poseCandidates[finalCandidateIdx] = candidate;
    }
  }

  // Run Kabsch on all the generated candidates to estimate the rigid transformations.
  compute_candidate_poses_kabsch();
}

void PreemptiveRansac_CPU::prepare_inliers_for_optimisation()
{
  Vector4f *candidateCameraPoints = m_poseOptimisationCameraPoints->GetData(MEMORYDEVICE_CPU);
  Keypoint3DColourCluster *candidateModes = m_poseOptimisationPredictedModes->GetData(MEMORYDEVICE_CPU);
  const int *inlierRasterIndices = m_inlierRasterIndicesBlock->GetData(MEMORYDEVICE_CPU);
  const Keypoint3DColour *keypointsImage = m_keypointsImage->GetData(MEMORYDEVICE_CPU);
  const uint32_t nbInliers = static_cast<uint32_t>(m_inlierRasterIndicesBlock->dataSize);
  const size_t nbPoseCandidates = m_poseCandidates->dataSize;
  const PoseCandidate *poseCandidates = m_poseCandidates->GetData(MEMORYDEVICE_CPU);
  const ScorePrediction *predictionsImage = m_predictionsImage->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int candidateIdx = 0; candidateIdx < nbPoseCandidates; ++candidateIdx)
  {
    for(uint32_t inlierIdx = 0; inlierIdx < nbInliers; ++inlierIdx)
    {
      preemptive_ransac_prepare_inliers_for_optimisation(
        keypointsImage, predictionsImage, inlierRasterIndices, nbInliers, poseCandidates, candidateCameraPoints,
        candidateModes, m_poseOptimisationInlierThreshold, candidateIdx, inlierIdx
      );
    }
  }

  // Compute and set the actual size of the buffers.
  const uint32_t poseOptimisationBufferSize = static_cast<uint32_t>(nbInliers * nbPoseCandidates);
  m_poseOptimisationCameraPoints->dataSize = poseOptimisationBufferSize;
  m_poseOptimisationPredictedModes->dataSize = poseOptimisationBufferSize;
}

void PreemptiveRansac_CPU::sample_inliers(bool useMask)
{
  const Vector2i imgSize = m_keypointsImage->noDims;
  int *inlierRasterIndices = m_inlierRasterIndicesBlock->GetData(MEMORYDEVICE_CPU);
  int *inliersMask = m_inliersMaskImage->GetData(MEMORYDEVICE_CPU);
  const Keypoint3DColour *keypoints = m_keypointsImage->GetData(MEMORYDEVICE_CPU);
  const ScorePrediction *predictions = m_predictionsImage->GetData(MEMORYDEVICE_CPU);
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(uint32_t sampleIdx = 0; sampleIdx < m_ransacInliersPerIteration; ++sampleIdx)
  {
    // Try to sample the raster index of a valid keypoint whose prediction has at least one modal cluster, using the mask if necessary.
    int rasterIdx = -1;
    if(useMask) rasterIdx = sample_inlier<true>(keypoints, predictions, imgSize, rngs[sampleIdx], inliersMask);
    else rasterIdx = sample_inlier<false>(keypoints, predictions, imgSize, rngs[sampleIdx]);

    // If we succeed, grab a unique index in the output array and store the inlier raster index into the corresponding array element.
    if(rasterIdx >= 0)
    {
      size_t arrayIdx = 0;

    #ifdef WITH_OPENMP
      #pragma omp atomic capture
    #endif
      arrayIdx = m_inlierRasterIndicesBlock->dataSize++;

      inlierRasterIndices[arrayIdx] = rasterIdx;
    }
  }
}

void PreemptiveRansac_CPU::update_candidate_poses()
{
  // Just call the base class implementation.
  PreemptiveRansac::update_candidate_poses();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void PreemptiveRansac_CPU::compute_pose_energy(PoseCandidate& candidate) const
{
  const int *inlierRasterIndices = m_inlierRasterIndicesBlock->GetData(MEMORYDEVICE_CPU);
  const Keypoint3DColour *keypointsImage = m_keypointsImage->GetData(MEMORYDEVICE_CPU);
  const uint32_t nbInliers = static_cast<uint32_t>(m_inlierRasterIndicesBlock->dataSize);
  const ScorePrediction *predictionsImage = m_predictionsImage->GetData(MEMORYDEVICE_CPU);

  const float energySum = compute_energy_sum_for_inliers(candidate.cameraPose, keypointsImage, predictionsImage, inlierRasterIndices, nbInliers);
  candidate.energy = energySum / static_cast<float>(nbInliers);
}

void PreemptiveRansac_CPU::init_random()
{
  // Initialise each random number generator based on the specified seed.
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);
  for(uint32_t i = 0; i < m_maxPoseCandidates; ++i)
  {
    rngs[i].reset(m_rngSeed + i);
  }
}

}
