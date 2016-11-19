/**
 * spaint: PreemptiveRansacForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PREEMPTIVERANSACSHARED
#define H_SPAINT_PREEMPTIVERANSACSHARED

#include "ORUtils/PlatformIndependence.h"
#include "features/interface/RGBDPatchFeature.h"
#include "randomforest/ScoreForestTypes.h"

namespace spaint
{

namespace
{
enum
{
  MAX_CANDIDATE_GENERATION_ITERATIONS = 6000, SAMPLE_INLIER_ITERATIONS = 50
};
}

template<typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline bool preemptive_ransac_generate_candidate(
    const RGBDPatchFeature *patchFeaturesData,
    const ScorePrediction *predictionsData, const Vector2i &imgSize,
    RNG &randomGenerator, PoseCandidate &poseCandidate,
    bool m_useAllModesPerLeafInPoseHypothesisGeneration,
    bool m_checkMinDistanceBetweenSampledModes,
    float m_minSqDistanceBetweenSampledModes,
    bool m_checkRigidTransformationConstraint,
    float m_translationErrorMaxForCorrectPose)
{
  int selectedPixelCount = 0;
  int selectedPixelLinearIdx[PoseCandidate::KABSCH_POINTS];
  int selectedPixelMode[PoseCandidate::KABSCH_POINTS];

  for (int iterationIdx = 0;
      selectedPixelCount != PoseCandidate::KABSCH_POINTS
          && iterationIdx < MAX_CANDIDATE_GENERATION_ITERATIONS; ++iterationIdx)
  {
    const int x = randomGenerator.generate_int_from_uniform(0, imgSize.width);
    const int y = randomGenerator.generate_int_from_uniform(0, imgSize.height);
    const int linearFeatureIdx = y * imgSize.width + x;
    const RGBDPatchFeature &selectedFeature =
        patchFeaturesData[linearFeatureIdx];

    // Invalid feature
    if (!selectedFeature.valid())
      continue;

    const ScorePrediction &selectedPrediction =
        predictionsData[linearFeatureIdx];

    // Prediction has no modes
    if (selectedPrediction.nbModes == 0)
      continue;

    // either use the first mode or select one randomly
    const int selectedModeIdx =
        m_useAllModesPerLeafInPoseHypothesisGeneration ?
            randomGenerator.generate_int_from_uniform(0,
                selectedPrediction.nbModes) :
            0;

    // Cache camera and world points, used for the following checks
    const Vector3f selectedModeWorldPt =
        selectedPrediction.modes[selectedModeIdx].position;
    const Vector3f selectedFeatureCameraPt =
        selectedFeature.position.toVector3();

    // This is the first pixel, check that the pixel colour corresponds with the selected mode
    if (selectedPixelCount == 0)
    {
      const Vector3i colourDiff = selectedFeature.colour.toInt()
          - selectedPrediction.modes[selectedModeIdx].colour.toInt();
      const bool consistentColour = abs(colourDiff.x) <= 30
          && abs(colourDiff.y) <= 30 && abs(colourDiff.z) <= 30;

      if (!consistentColour)
        continue;
    }

    if (m_checkMinDistanceBetweenSampledModes)
    {
      // Check that this mode is far enough from the other modes
      bool farEnough = true;

      for (int idxOther = 0; farEnough && idxOther < selectedPixelCount;
          ++idxOther)
      {
        const int otherLinearIdx = selectedPixelLinearIdx[idxOther];
        const int otherModeIdx = selectedPixelMode[idxOther];
        const ScorePrediction &otherPrediction =
            predictionsData[otherLinearIdx];

        const Vector3f otherModeWorldPt =
            otherPrediction.modes[otherModeIdx].position;
        const Vector3f diff = otherModeWorldPt - selectedModeWorldPt;
        const float distOtherSq = dot(diff, diff);

        if (distOtherSq < m_minSqDistanceBetweenSampledModes)
        {
          farEnough = false;
        }
      }

      if (!farEnough)
        continue;
    }

    if (m_checkRigidTransformationConstraint)
    {
      bool violatesConditions = false;

      for (int m = 0; m < selectedPixelCount && !violatesConditions; ++m)
      {
        const int otherModeIdx = selectedPixelMode[m];
        const int otherLinearIdx = selectedPixelLinearIdx[m];
        const ScorePrediction &otherPrediction =
            predictionsData[otherLinearIdx];

        const Vector3f otherFeatureCameraPt =
            patchFeaturesData[otherLinearIdx].position.toVector3();
        const Vector3f diffCamera = otherFeatureCameraPt
            - selectedFeatureCameraPt;
        const float distCameraSq = dot(diffCamera, diffCamera);

        if (distCameraSq < m_minSqDistanceBetweenSampledModes)
        {
          violatesConditions = true;
          break;
        }

        const Vector3f otherModeWorldPt =
            otherPrediction.modes[otherModeIdx].position;
        const Vector3f diffWorld = otherModeWorldPt - selectedModeWorldPt;

        const float distWorld = length(diffWorld);
        const float distCamera = sqrtf(distCameraSq);
        if (fabsf(distCamera - distWorld)
            > 0.5f * m_translationErrorMaxForCorrectPose)
        {
          violatesConditions = true;
        }
      }

      if (violatesConditions)
        continue;
    }

    selectedPixelLinearIdx[selectedPixelCount] = linearFeatureIdx;
    selectedPixelMode[selectedPixelCount] = selectedModeIdx;
    ++selectedPixelCount;
  }

  // Reached limit of iterations
  if (selectedPixelCount != PoseCandidate::KABSCH_POINTS)
    return false;

  // Populate resulting pose candidate (except the actual pose that is computed on the CPU due to Kabsch)
  poseCandidate.energy = 0.f;

  for (int s = 0; s < selectedPixelCount; ++s)
  {
    const int linearIdx = selectedPixelLinearIdx[s];
    const int modeIdx = selectedPixelMode[s];

    const RGBDPatchFeature &selectedFeature = patchFeaturesData[linearIdx];
    const ScorePrediction &selectedPrediction = predictionsData[linearIdx];
    const ScoreMode &selectedMode = selectedPrediction.modes[modeIdx];

    poseCandidate.cameraPoints[s] = selectedFeature.position.toVector3();
    poseCandidate.worldPoints[s] = selectedMode.position;
  }

  return true;
}

template<bool useMask, typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline int preemptive_ransac_sample_inlier(
    const RGBDPatchFeature *patchFeaturesData,
    const ScorePrediction *predictionsData, const Vector2i &imgSize,
    RNG &randomGenerator, int *inlierMaskData = NULL)
{
  int inlierLinearIdx = -1;

  for (int iterationIdx = 0;
      inlierLinearIdx < 0 && iterationIdx < SAMPLE_INLIER_ITERATIONS;
      ++iterationIdx)
  {
    const int linearIdx = randomGenerator.generate_int_from_uniform(0,
        imgSize.width * imgSize.height);

    // Check that we have a valid feature for this sample
    if (patchFeaturesData[linearIdx].position.w < 0.f)
      continue;

    // Check that we have a prediction with a non null number of modes
    if (predictionsData[linearIdx].nbModes == 0)
      continue;

    bool validIndex = !useMask;

    if (useMask)
    {
      int *maskPtr = &inlierMaskData[linearIdx];
      int maskValue = -1;

#ifdef __CUDACC__
      maskValue = atomicAdd(maskPtr, 1);
#else

#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
      maskValue = (*maskPtr)++;

#endif

      validIndex = maskValue == 0;
    }

    if (validIndex)
      inlierLinearIdx = linearIdx;
  }

  return inlierLinearIdx;
}

_CPU_AND_GPU_CODE_
inline float preemptive_ransac_compute_candidate_energy(
    const Matrix4f &candidatePose, const RGBDPatchFeature *features,
    const ScorePrediction *predictions, const int *inlierIndices,
    uint32_t nbInliers, uint32_t inlierStartIdx = 0, uint32_t inlierStep = 1)
{
  float localEnergy = 0.f;

  for (uint32_t inlierIdx = inlierStartIdx; inlierIdx < nbInliers; inlierIdx +=
      inlierStep)
  {
    const int linearIdx = inlierIndices[inlierIdx];
    const Vector3f localPixel = features[linearIdx].position.toVector3();
    const Vector3f projectedPixel = candidatePose * localPixel;

    const ScorePrediction &pred = predictions[linearIdx];

    // eval individual energy
    float energy;
    int argmax = pred.get_best_mode_and_energy(projectedPixel, energy);

    // Has at least a valid mode
    if (argmax < 0)
    {
#ifdef __CUDACC__
      // should have not been inserted in the inlier set
      printf("prediction has no valid modes\n");
      asm("trap;");
#else
      throw std::runtime_error("prediction has no valid modes");
#endif
    }

    if (pred.modes[argmax].nbInliers == 0)
    {
#ifdef __CUDACC__
      // the original implementation had a simple continue
      printf("mode has no inliers\n");
      asm("trap;");
#else
      throw std::runtime_error("mode has no inliers");
#endif
    }

    energy /= static_cast<float>(pred.nbModes);
    energy /= static_cast<float>(pred.modes[argmax].nbInliers);

    if (energy < 1e-6f)
      energy = 1e-6f;
    energy = -log10f(energy);

    localEnergy += energy;
  }

  return localEnergy;
}

}

#endif
