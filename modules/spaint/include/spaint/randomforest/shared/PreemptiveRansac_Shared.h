/**
 * spaint: PreemptiveRansacForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_PREEMPTIVERANSACSHARED
#define H_SPAINT_PREEMPTIVERANSACSHARED

#include "ORUtils/PlatformIndependence.h"

namespace spaint
{

namespace
{
enum
{
  N_POINTS_FOR_KABSCH = 3, MAX_CANDIDATE_GENERATION_ITERATIONS = 6000
};
}

template<typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline bool preemptive_ransac_generate_candidate(
    const RGBDPatchFeature *patchFeaturesData,
    const GPUForestPrediction *predictionsData, const Vector2i &imgSize,
    RNG &randomGenerator, PoseCandidate &poseCandidate,
    bool m_useAllModesPerLeafInPoseHypothesisGeneration,
    bool m_checkMinDistanceBetweenSampledModes,
    float m_minSqDistanceBetweenSampledModes,
    bool m_checkRigidTransformationConstraint,
    float m_translationErrorMaxForCorrectPose)
{
  int selectedPixelCount = 0;
  int selectedPixelLinearIdx[N_POINTS_FOR_KABSCH];
  int selectedPixelMode[N_POINTS_FOR_KABSCH];

  for (int iterationIdx = 0;
      selectedPixelCount != N_POINTS_FOR_KABSCH
          && iterationIdx < MAX_CANDIDATE_GENERATION_ITERATIONS; ++iterationIdx)
  {
    const int x = randomGenerator.generate_int_from_uniform(0, imgSize.width);
    const int y = randomGenerator.generate_int_from_uniform(0, imgSize.height);
    const int linearFeatureIdx = y * imgSize.width + x;
    const RGBDPatchFeature &selectedFeature =
        patchFeaturesData[linearFeatureIdx];

    // Invalid feature
    if (selectedFeature.position.w < 0.f)
      continue;

    const GPUForestPrediction &selectedPrediction =
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
      const Vector3u colourDiff = selectedFeature.colour.toVector3().toUChar()
          - selectedPrediction.modes[selectedModeIdx].colour;
      const bool consistentColour = fabsf(colourDiff.x) <= 30.f
          && fabsf(colourDiff.y) <= 30.f && fabsf(colourDiff.z) <= 30.f;

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
        const GPUForestPrediction &otherPrediction =
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
        const GPUForestPrediction &otherPrediction =
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
  if (selectedPixelCount != N_POINTS_FOR_KABSCH)
    return false;

  // Populate resulting pose candidate (except the actual pose that is computed on the CPU due to Kabsch)
  poseCandidate.nbInliers = selectedPixelCount;
  poseCandidate.energy = 0.f;
  poseCandidate.cameraId = -1;

  for (int s = 0; s < selectedPixelCount; ++s)
  {
    const int linearIdx = selectedPixelLinearIdx[s];
    const int modeIdx = selectedPixelMode[s];

    poseCandidate.inliers[s].linearIdx = linearIdx;
    poseCandidate.inliers[s].modeIdx = modeIdx;
    poseCandidate.inliers[s].energy = 0.f;
  }

  return true;
}

}

#endif
