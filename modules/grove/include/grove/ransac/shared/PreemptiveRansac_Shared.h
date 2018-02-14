/**
 * grove: PreemptiveRansacForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSACSHARED
#define H_GROVE_PREEMPTIVERANSACSHARED

#include <ORUtils/PlatformIndependence.h>

#include "PoseCandidate.h"
#include "../../keypoints/Keypoint3DColour.h"
#include "../../scoreforests/ScorePrediction.h"

namespace grove {

//#################### CONSTANTS ####################

enum
{
  MAX_COLOUR_DELTA = 30,
  SAMPLE_INLIER_ITERATIONS = 50
};

//#################### FUNCTIONS ####################

/**
 * \brief Compute the energy associated with a candidate pose. The energy represents how well the selected inliers agree
 *        with the rigid camera transformation.
 *
 * \note  Each inlier contributes to a part of the total energy. This method can be used to compute the energy for a
 *        strided subset of inliers (used when parallelising the computation with CUDA).
 *
 * \param candidatePose  The rigid transformation from camera to world coordinates.
 * \param keypoints      The 3D keypoints extracted from a RGB-D image pair.
 * \param predictions    The SCoRe forest predictions associated to the keypoints.
 * \param inlierIndices  The raster indices of the keypoints used to compute the energy.
 * \param nbInliers      The number of inliers.
 * \param inlierStartIdx The first inlier in inlierIndices to use in the energy computation.
 * \param inlierStep     Step between the inliers that have to be used to compute the energy.
 *
 * \return The sum of energies contributed by the inliers.
 */
_CPU_AND_GPU_CODE_
inline float compute_energy_sum_for_inlier_subset(const Matrix4f& candidatePose, const Keypoint3DColour *keypoints, const ScorePrediction *predictions,
                                                  const int *inlierIndices, uint32_t nbInliers, uint32_t inlierStartIdx, uint32_t inlierStep)
{
  float localEnergy = 0.f;

  // Strided sum loop.
  for(uint32_t inlierIdx = inlierStartIdx; inlierIdx < nbInliers; inlierIdx += inlierStep)
  {
    const int linearIdx = inlierIndices[inlierIdx];
    const Vector3f inlierCameraCoordinates = keypoints[linearIdx].position;
    const Vector3f inlierWorldCoordinates = candidatePose * inlierCameraCoordinates;

    // Get the prediction associated to the current inlier.
    const ScorePrediction& pred = predictions[linearIdx];

    // Evaluate individual energy. First find the mode closest to the predicted world coordinates.
    float energy;
    int argmax = score_prediction_get_best_mode_and_energy(pred, inlierWorldCoordinates, energy);

    // The inlier must have at least a valid mode (made sure by the inlier sampling method). If not throw.
    if(argmax < 0)
    {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
      printf("prediction has no valid modes\n");
      asm("trap;");
#else
      throw std::runtime_error("prediction has no valid modes");
#endif
    }

    // If the best mode had no inliers throw (shouldn't be a mode, so something obviously went wrong during the
    // clustering). The original implementation (from Valentin's paper) had a simple continue.
    if(pred.elts[argmax].nbInliers == 0)
    {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
      printf("mode has no inliers\n");
      asm("trap;");
#else
      throw std::runtime_error("mode has no inliers");
#endif
    }

    // Normalise the energy.
    energy /= static_cast<float>(pred.size);
    energy /= static_cast<float>(pred.elts[argmax].nbInliers);

    if(energy < 1e-6f) energy = 1e-6f;
    energy = -log10f(energy);

    localEnergy += energy;
  }

  return localEnergy;
}

/**
 * \brief TODO
 */
_CPU_AND_GPU_CODE_
inline float compute_energy_sum_for_inliers(const Matrix4f& candidatePose, const Keypoint3DColour *keypoints, const ScorePrediction *predictions,
                                            const int *inlierIndices, uint32_t nbInliers)
{
  const uint32_t inlierStartIdx = 0;
  const uint32_t inlierStep = 1;
  return compute_energy_sum_for_inlier_subset(candidatePose, keypoints, predictions, inlierIndices, nbInliers, inlierStartIdx, inlierStep);
}

/**
 * \brief Try to generate a camera pose candidate according to the method described in the paper.
 *
 * \param keypointsData   The 3D keypoints extracted from a RGB-D image pair.
 * \param predictionsData Score predictions associated to the keypoints.
 * \param imgSize         The size of the input keypoint and predictions images.
 * \param randomGenerator Either a CPURNG or a CUDARNG according to the current device type.
 * \param poseCandidate   The variable where the generated pose candidate will be stored.
 * \param maxCandidateGenerationIterations             The maximum number of iterations in the candidate generation step.
 * \param useAllModesPerLeafInPoseHypothesisGeneration Whether to use all modes in the predictions when generating the
 *                                                     pose hypothesis or just the first one.
 * \param checkMinDistanceBetweenSampledModes          Whether or not to check that sampled modes have a minimum
 *                                                     distance between each other.
 * \param minSqDistanceBetweenSampledModes             The minimum (squared) distance between sampled modes if the above
 *                                                     parameter is true.
 * \param checkRigidTransformationConstraint           Whether or not to check that the selected modes define a
 *                                                     quasi-rigid transformation.
 * \param maxTranslationErrorForCorrectPose            The maximum difference between the distances of pair of points in
 *                                                     camera frame and pair of modes in world coordinates if the
 *                                                     previous check is enabled.
 *
 * \return true if a pose candidate was successfully generated.
 */
template <typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline bool preemptive_ransac_generate_candidate(const Keypoint3DColour *keypointsData, const ScorePrediction *predictionsData, const Vector2i& imgSize,
                                                 RNG& randomGenerator, PoseCandidate& poseCandidate, uint32_t maxCandidateGenerationIterations,
                                                 bool useAllModesPerLeafInPoseHypothesisGeneration, bool checkMinDistanceBetweenSampledModes,
                                                 float minSqDistanceBetweenSampledModes, bool checkRigidTransformationConstraint,
                                                 float maxTranslationErrorForCorrectPose)
{
  int selectedPixelCount = 0;
  int selectedPixelLinearIdx[PoseCandidate::KABSCH_POINTS];
  int selectedPixelMode[PoseCandidate::KABSCH_POINTS];

  // Try to generate a candidate (sample KABSCH_POINTS point pairs) for a certain number of times and return false if we
  // fail.
  for(uint32_t iterationIdx = 0;
      selectedPixelCount != PoseCandidate::KABSCH_POINTS && iterationIdx < maxCandidateGenerationIterations;
      ++iterationIdx)
  {
    // Sample a pixel in the input image.
    const int x = randomGenerator.generate_int_from_uniform(0, imgSize.width - 1);
    const int y = randomGenerator.generate_int_from_uniform(0, imgSize.height - 1);
    const int linearFeatureIdx = y * imgSize.width + x;

    // Grab its associated keypoint and continue only if it's valid.
    const Keypoint3DColour& selectedKeypoint = keypointsData[linearFeatureIdx];
    // Invalid keypoint, try again.
    if(!selectedKeypoint.valid) continue;

    // Grab its associated score prediction.
    const ScorePrediction& selectedPrediction = predictionsData[linearFeatureIdx];
    // If the prediction has no modes, try again.
    if(selectedPrediction.size == 0) continue;

    // Either use the first mode or select one randomly, depending on the parameters.
    const int selectedModeIdx = useAllModesPerLeafInPoseHypothesisGeneration
                                    ? randomGenerator.generate_int_from_uniform(0, selectedPrediction.size - 1)
                                    : 0;

    // Cache camera and world points, used for the following checks.
    const Vector3f selectedModeWorldPt = selectedPrediction.elts[selectedModeIdx].position;
    const Vector3f selectedFeatureCameraPt = selectedKeypoint.position;

    // If this is the first pixel, check that the pixel colour corresponds with the selected mode's colour.
    if(selectedPixelCount == 0)
    {
      const Vector3i colourDiff =
          selectedKeypoint.colour.toInt() - selectedPrediction.elts[selectedModeIdx].colour.toInt();
      const bool consistentColour = abs(colourDiff.x) <= MAX_COLOUR_DELTA && abs(colourDiff.y) <= MAX_COLOUR_DELTA &&
                                    abs(colourDiff.z) <= MAX_COLOUR_DELTA;

      // If not try to sample another pixel.
      if(!consistentColour) continue;
    }

    if(checkMinDistanceBetweenSampledModes)
    {
      // Check that this mode is far enough from the other modes in world coordinates.
      bool farEnough = true;

      for(int idxOther = 0; farEnough && idxOther < selectedPixelCount; ++idxOther)
      {
        const int otherLinearIdx = selectedPixelLinearIdx[idxOther];
        const int otherModeIdx = selectedPixelMode[idxOther];
        const ScorePrediction& otherPrediction = predictionsData[otherLinearIdx];

        const Vector3f otherModeWorldPt = otherPrediction.elts[otherModeIdx].position;
        const Vector3f diff = otherModeWorldPt - selectedModeWorldPt;

        // If they are too close, drop the current pixel and try to sample another one.
        const float distOtherSq = dot(diff, diff);
        if(distOtherSq < minSqDistanceBetweenSampledModes)
        {
          farEnough = false;
        }
      }

      if(!farEnough) continue;
    }

    if(checkRigidTransformationConstraint)
    {
      // Check that the sampled point pairs represent a quasi rigid triangle.
      bool violatesConditions = false;

      for(int m = 0; m < selectedPixelCount && !violatesConditions; ++m)
      {
        const int otherModeIdx = selectedPixelMode[m];
        const int otherLinearIdx = selectedPixelLinearIdx[m];
        const ScorePrediction& otherPrediction = predictionsData[otherLinearIdx];

        // First check that the current keypoint is far enough from the other keypoints, similarly to the previous
        // check.
        const Vector3f otherFeatureCameraPt = keypointsData[otherLinearIdx].position;
        const Vector3f diffCamera = otherFeatureCameraPt - selectedFeatureCameraPt;
        const float distCameraSq = dot(diffCamera, diffCamera);

        if(distCameraSq < minSqDistanceBetweenSampledModes)
        {
          violatesConditions = true;
          break;
        }

        // Then check that the distance between the current keypoint and the other keypoint and the distance between the
        // current mode and the other mode are similar enough.
        const Vector3f otherModeWorldPt = otherPrediction.elts[otherModeIdx].position;
        const Vector3f diffWorld = otherModeWorldPt - selectedModeWorldPt;

        const float distWorld = length(diffWorld);
        const float distCamera = sqrtf(distCameraSq);
        if(fabsf(distCamera - distWorld) > 0.5f * maxTranslationErrorForCorrectPose)
        {
          violatesConditions = true;
        }
      }

      // If we failed try to sample another pixel.
      if(violatesConditions) continue;
    }

    // We succeeded, save the current pixel raster index and the selected mode index.
    selectedPixelLinearIdx[selectedPixelCount] = linearFeatureIdx;
    selectedPixelMode[selectedPixelCount] = selectedModeIdx;
    ++selectedPixelCount;
  }

  // Reached limit of iterations and didn't find enough points. Early out.
  if(selectedPixelCount != PoseCandidate::KABSCH_POINTS) return false;

  // Populate resulting pose candidate (except the actual pose that is computed on the CPU due to the Kabsch
  // implementation which is currently CPU only).

  // Reset the energy.
  poseCandidate.energy = 0.f;

  // Copy the camera and corresponding world points.
  for(int s = 0; s < selectedPixelCount; ++s)
  {
    const int linearIdx = selectedPixelLinearIdx[s];
    const int modeIdx = selectedPixelMode[s];

    const Keypoint3DColour& selectedKeypoint = keypointsData[linearIdx];
    const ScorePrediction& selectedPrediction = predictionsData[linearIdx];
    const Keypoint3DColourCluster& selectedMode = selectedPrediction.elts[modeIdx];

    poseCandidate.pointsCamera[s] = selectedKeypoint.position;
    poseCandidate.pointsWorld[s] = selectedMode.position;
  }

  return true;
}

_CPU_AND_GPU_CODE_
inline void preemptive_ransac_prepare_inliers_for_optimisation(const Keypoint3DColour *keypoints, const ScorePrediction *predictions, const int *inlierIndices,
                                                               uint32_t nbInliers, const PoseCandidate *poseCandidates, Vector4f *inlierCameraPoints,
                                                               Keypoint3DColourCluster *inlierModes, float inlierThreshold, uint32_t candidateIdx, uint32_t inlierIdx)
{
  const int inlierLinearIdx = inlierIndices[inlierIdx];
  const PoseCandidate& poseCandidate = poseCandidates[candidateIdx];
  const Vector3f inlierCameraPosition = keypoints[inlierLinearIdx].position;
  const Vector3f inlierWorldPosition = poseCandidate.cameraPose * inlierCameraPosition;
  const ScorePrediction& prediction = predictions[inlierLinearIdx];

  // Find the best mode, do not rely on the one stored in the inlier because for the randomly sampled inliers it will
  // not be set.
  // We also assume that the inlier is valid (we checked that before, when we selected it).
  const int bestModeIdx = score_prediction_get_best_mode(prediction, inlierWorldPosition);
  if(bestModeIdx < 0 || bestModeIdx >= prediction.size)
  {
    // This point should not have been selected as inlier.
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
      printf("best mode idx invalid\n");
      asm("trap;");
#else
      throw std::runtime_error("best mode idx invalid");
#endif
  }

  const Keypoint3DColourCluster& bestMode = prediction.elts[bestModeIdx];

  uint32_t outputIdx = candidateIdx * nbInliers + inlierIdx; // The index in the row associated to the current candidate.

  // We add this pair to the vector of pairs to be evaluated iff the predicted mode and the world position estimated
  // by the current camera pose agree.
  if(length(bestMode.position - inlierWorldPosition) >= inlierThreshold)
  {
    inlierCameraPoints[outputIdx] = Vector4f(0.f);
    return;
  }

  // Store the inlier camera pose and the corresponding mode.
  inlierCameraPoints[outputIdx] = Vector4f(inlierCameraPosition, 1.f);
  inlierModes[outputIdx] = bestMode;
}

/**
 * \brief Try to sample the index of a keypoint which is valid and has at least one modal cluster associated.
 *
 * \param useMask Whether to use the mask to mark the sampled points. If true a point will not be sampled twice.
 *
 * \param keypointsData   The 3D keypoints.
 * \param predictionsData The ScorePredictions associated to the keypoints.
 * \param imgSize         The size of the keypoint and predictions images.
 * \param randomGenerator A random number generator used during sampling. Can either be a CPURNG or a CUDARNG.
 * \param inlierMaskData  The mask used to avoid sampling keypoint indices twice. Can be NULL if useMask is false.
 *
 * \return The linear index of the sample keypoint. -1 in case no keypoint was sampled.
 */
template <bool useMask, typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline int preemptive_ransac_sample_inlier(const Keypoint3DColour *keypointsData, const ScorePrediction *predictionsData, const Vector2i& imgSize,
                                           RNG& randomGenerator, int *inlierMaskData = NULL)
{
  int inlierLinearIdx = -1;

  // Limit the number of sampling attempts.
  for(int iterationIdx = 0; inlierLinearIdx < 0 && iterationIdx < SAMPLE_INLIER_ITERATIONS; ++iterationIdx)
  {
    // Sample a keypoint index.
    const int linearIdx = randomGenerator.generate_int_from_uniform(0, imgSize.width * imgSize.height - 1);

    // Check that we have a valid keypoint.
    if(!keypointsData[linearIdx].valid) continue;

    // Check that we have a prediction with a non null number of modes.
    if(predictionsData[linearIdx].size == 0) continue;

    // Finally, check the mask if necessary.
    bool validIndex = !useMask;

    if(useMask)
    {
      int *maskPtr = &inlierMaskData[linearIdx];
      int maskValue = -1;

// Atomically increment the mask and capture the current value.
#ifdef __CUDACC__
      maskValue = atomicAdd(maskPtr, 1);
#else

#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
      maskValue = (*maskPtr)++;
#endif

      // If the value was zero we can keep this keypoint.
      validIndex = maskValue == 0;
    }

    if(validIndex) inlierLinearIdx = linearIdx;
  }

  return inlierLinearIdx;
}

}

#endif
