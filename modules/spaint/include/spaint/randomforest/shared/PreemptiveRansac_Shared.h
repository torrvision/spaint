/**
 * spaint: PreemptiveRansacForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ORUtils/PlatformIndependence.h"

#ifndef H_SPAINT_PREEMPTIVERANSACSHARED
#define H_SPAINT_PREEMPTIVERANSACSHARED

namespace spaint
{

template<typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline bool preemptive_ransac_generate_candidate(
    const RGBDPatchFeature *patchFeaturesData,
    const GPUForestPrediction *predictionsData, const Vector2i &imgSize,
    RNG &randomGenerator, PoseCandidate &poseCandidate,
    bool m_useAllModesPerLeafInPoseHypothesisGeneration,
    bool m_checkMinDistanceBetweenSampledModes,
    float m_minDistanceBetweenSampledModes,
    bool m_checkRigidTransformationConstraint,
    float m_translationErrorMaxForCorrectPose)
{
  static const int m_nbPointsForKabschBoostrap = 3;

  bool foundIsometricMapping = false;

  int selectedPixelCount = 0;
  int selectedPixelX[m_nbPointsForKabschBoostrap];
  int selectedPixelY[m_nbPointsForKabschBoostrap];
  int selectedPixelMode[m_nbPointsForKabschBoostrap];

  static const int maxIterationsInner = 6000;
  int iterationsInner = 0;

  while (selectedPixelCount != m_nbPointsForKabschBoostrap
      && iterationsInner < maxIterationsInner)
  {
    ++iterationsInner;

    const int x = randomGenerator.generate_int_from_uniform(0, imgSize.width);
    const int y = randomGenerator.generate_int_from_uniform(0, imgSize.height);
//      const int x = curand(randomState) % (imgSize.width - 1);
//      const int y = curand(randomState) % (imgSize.height - 1);
    // The implementation below sometimes generates OOB values (with 0.999999,
    // with 0.999 it works but seems a weird hack)
//      const int x = __float2int_rz(
//          curand_uniform(randomState) * (imgSize.width - 1 + 0.999999f));
//      const int y = __float2int_rz(
//          curand_uniform(randomState) * (imgSize.height - 1 + 0.999999f));
    const int linearFeatureIdx = y * imgSize.width + x;
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
      selectedModeIdx = randomGenerator.generate_int_from_uniform(0,
          selectedPrediction.nbModes);
//        selectedModeIdx = curand(randomState)
//            % (selectedPrediction.nbModes - 1);
//        selectedModeIdx = __float2int_rz(
//            curand_uniform(randomState)
//                * (selectedPrediction.nbModes - 1 + 0.999f));
    }

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

    // if (false)
    if (m_checkMinDistanceBetweenSampledModes)
    {
      const Vector3f worldPt =
          selectedPrediction.modes[selectedModeIdx].position;

      // Check that this mode is far enough from the other modes
      bool farEnough = true;

      for (int idxOther = 0; idxOther < selectedPixelCount; ++idxOther)
      {
        const int xOther = selectedPixelX[idxOther];
        const int yOther = selectedPixelY[idxOther];
        const int modeIdxOther = selectedPixelMode[idxOther];

        const int linearIdxOther = yOther * imgSize.width + xOther;
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

      for (int m = 0; m < selectedPixelCount && !violatesConditions; ++m)
      {
        const int xFirst = selectedPixelX[m];
        const int yFirst = selectedPixelY[m];
        const int modeIdxFirst = selectedPixelMode[m];
        const int linearIdxOther = yFirst * imgSize.width + xFirst;
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

    selectedPixelX[selectedPixelCount] = x;
    selectedPixelY[selectedPixelCount] = y;
    selectedPixelMode[selectedPixelCount] = selectedModeIdx;
    ++selectedPixelCount;
  }

  //    std::cout << "Inner iterations: " << iterationsInner << std::endl;

  // Reached limit of iterations
  if (selectedPixelCount != m_nbPointsForKabschBoostrap)
    return false;

  // Populate resulting pose (except the actual pose that is computed on the CPU due to Kabsch)
  foundIsometricMapping = true;
  poseCandidate.nbInliers = selectedPixelCount;
  poseCandidate.energy = 0.f;
  poseCandidate.cameraId = -1;

  for (int s = 0; s < selectedPixelCount; ++s)
  {
    const int x = selectedPixelX[s];
    const int y = selectedPixelY[s];
    const int modeIdx = selectedPixelMode[s];
    const int linearIdx = y * imgSize.width + x;

    poseCandidate.inliers[s].linearIdx = linearIdx;
    poseCandidate.inliers[s].modeIdx = modeIdx;
    poseCandidate.inliers[s].energy = 0.f;
  }

  return foundIsometricMapping;
}

}

#endif
