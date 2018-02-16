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
 * \brief Computes an energy sum representing how well a strided subset of a set of "inlier" keypoints agree with a candidate camera pose.
 *
 * \note  Each "inlier" keypoint contributes a part of the total energy sum.
 * \note  This function exists to make the energy sum computation easier to parallelise using CUDA.
 *
 * \param candidatePose  The candidate camera pose (a rigid transformation from camera -> world coordinates).
 * \param keypoints      The 3D keypoints extracted from an RGB-D image pair.
 * \param predictions    The SCoRe forest predictions associated with the keypoints.
 * \param inlierIndices  The raster indices of the overall set of "inlier" keypoints.
 * \param nbInliers      The overall number of "inlier" keypoints.
 * \param inlierStartIdx The array index of the first "inlier" keypoint in inlierIndices to use when computing the energy sum.
 * \param inlierStep     The step between the array indices of the "inlier" keypoints to use when computing the energy sum.
 * \return               The sum of the energies contributed by the "inlier" keypoints in the strided subset.
 */
_CPU_AND_GPU_CODE_
inline float compute_energy_sum_for_inlier_subset(const Matrix4f& candidatePose, const Keypoint3DColour *keypoints, const ScorePrediction *predictions,
                                                  const int *inlierIndices, uint32_t nbInliers, uint32_t inlierStartIdx, uint32_t inlierStep)
{
  float energySum = 0.0f;

  // For each "inlier" keypoint in the strided subset:
  for(uint32_t inlierIdx = inlierStartIdx; inlierIdx < nbInliers; inlierIdx += inlierStep)
  {
    // Look up the raster index of the inlier and its position in camera space.
    const int rasterIdx = inlierIndices[inlierIdx];
    const Vector3f inlierCameraCoordinates = keypoints[rasterIdx].position;

    // Compute the hypothesised position of the inlier in world space.
    const Vector3f inlierWorldCoordinates = candidatePose * inlierCameraCoordinates;

    // Get the prediction associated with the inlier.
    const ScorePrediction& pred = predictions[rasterIdx];

    // Compute the energy for the inlier, which is based on the Mahalanobis distance between the hypothesised
    // position of the inlier (in world space) and the position of the closest predicted mode.
    float energy;
    int argmax = score_prediction_get_best_mode_and_energy(pred, inlierWorldCoordinates, energy);

    // We expect the inlier to have had at least one valid mode (this is guaranteed by the inlier sampling process).
    // If this isn't the case for some reason, defensively throw.
    if(argmax < 0)
    {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
      printf("prediction has no valid modes\n");
      asm("trap;");
#else
      throw std::runtime_error("prediction has no valid modes");
#endif
    }

    // We expect the best mode to have at least some inliers (this is guaranteed by the clustering process).
    // If this isn't the case for some reason, defensively throw.
    if(pred.elts[argmax].nbInliers == 0)
    {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
      printf("mode has no inliers\n");
      asm("trap;");
#else
      throw std::runtime_error("mode has no inliers");
#endif
    }

    // Assuming we have found a best mode and it has at least some inliers, appropriately normalise the energy.
    energy /= static_cast<float>(pred.size);
    energy /= static_cast<float>(pred.elts[argmax].nbInliers);

    // Compute the negative log of the energy (after first ensuring that it isn't too small for this to work).
    if(energy < 1e-6f) energy = 1e-6f;
    energy = -log10f(energy);

    // Add the resulting value to the energy sum.
    energySum += energy;
  }

  return energySum;
}

/**
 * \brief Computes an energy sum representing how well a set of "inlier" keypoints agree with a candidate camera pose.
 *
 * \param candidatePose The candidate camera pose (a rigid transformation from camera -> world coordinates).
 * \param keypoints     The 3D keypoints extracted from an RGB-D image pair.
 * \param predictions   The SCoRe forest predictions associated with the keypoints.
 * \param inlierIndices The raster indices of the "inlier" keypoints that we will use to compute the energy sum.
 * \param nbInliers     The number of "inlier" keypoints.
 * \return              The sum of the energies contributed by the "inlier" keypoints.
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
 * \brief Tries to generate a camera pose candidate using the method described in the paper.
 *
 * \param keypointsData                                 The 3D keypoints extracted from an RGB-D image pair.
 * \param predictionsData                               The SCoRe predictions associated with the keypoints.
 * \param imgSize                                       The size of the input keypoints and predictions images.
 * \param rng                                           Either a CPURNG or a CUDARNG, depending on the current device type.
 * \param poseCandidate                                 The variable in which the generated pose candidate (if any) will be stored.
 * \param maxCandidateGenerationIterations              The maximum number of iterations in the candidate generation step.
 * \param useAllModesPerLeafInPoseHypothesisGeneration  Whether or not to use all modes in the predictions when generating the pose hypothesis, rather than just the first one.
 * \param checkMinDistanceBetweenSampledModes           Whether or not to check that sampled modes have a minimum distance between each other.
 * \param minSqDistanceBetweenSampledModes              The minimum squared distance between sampled modes if the above parameter is true.
 * \param checkRigidTransformationConstraint            Whether or not to check that the selected modes define a quasi-rigid transformation.
 * \param maxTranslationErrorForCorrectPose             The maximum difference between the distances of a pair of points in the camera frame and a pair of modes in world coordinates
 *                                                      if the previous check is enabled.
 *
 * \return  true, if a pose candidate was successfully generated, or false otherwise.
 */
template <typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline bool generate_pose_candidate(const Keypoint3DColour *keypointsData, const ScorePrediction *predictionsData, const Vector2i& imgSize,
                                    RNG& rng, PoseCandidate& poseCandidate, uint32_t maxCandidateGenerationIterations,
                                    bool useAllModesPerLeafInPoseHypothesisGeneration, bool checkMinDistanceBetweenSampledModes,
                                    float minSqDistanceBetweenSampledModes, bool checkRigidTransformationConstraint,
                                    float maxTranslationErrorForCorrectPose)
{
  int correspondencesFound = 0;
  int selectedRasterIndices[PoseCandidate::KABSCH_CORRESPONDENCES_NEEDED];
  int selectedModeIndices[PoseCandidate::KABSCH_CORRESPONDENCES_NEEDED];

  // Try to generate correspondences for Kabsch, iterating in total at most maxCandidateGenerationIterations times.
  for(uint32_t i = 0; correspondencesFound != PoseCandidate::KABSCH_CORRESPONDENCES_NEEDED && i < maxCandidateGenerationIterations; ++i)
  {
    // Sample a pixel in the input image.
    const int x = rng.generate_int_from_uniform(0, imgSize.width - 1);
    const int y = rng.generate_int_from_uniform(0, imgSize.height - 1);
    const int rasterIdx = y * imgSize.width + x;

    // Look up the keypoint associated with the pixel and check whether it's valid. If not, skip this iteration of the loop and try again.
    const Keypoint3DColour& keypoint = keypointsData[rasterIdx];
    if(!keypoint.valid) continue;

    // Look up the SCoRe prediction associated with the pixel and check whether it has any modes. If not, skip this iteration of the loop and try again.
    const ScorePrediction& prediction = predictionsData[rasterIdx];
    if(prediction.size == 0) continue;

    // Choose which mode to use, depending on the parameters specified. This will either be the first mode, or a randomly-chosen one.
    const int modeIdx = useAllModesPerLeafInPoseHypothesisGeneration ? rng.generate_int_from_uniform(0, prediction.size - 1) : 0;

    // Cache the camera and world points to avoid repeated global reads (these are used multiple times in the following checks).
    const Vector3f cameraPt = keypoint.position;
    const Vector3f worldPt = prediction.elts[modeIdx].position;

    // If this is the first correspondence, check that the keypoint's colour is consistent with the mode's colour.
    if(correspondencesFound == 0)
    {
      const Vector3i colourDiff = keypoint.colour.toInt() - prediction.elts[modeIdx].colour.toInt();
      const bool consistentColour = abs(colourDiff.x) <= MAX_COLOUR_DELTA && abs(colourDiff.y) <= MAX_COLOUR_DELTA && abs(colourDiff.z) <= MAX_COLOUR_DELTA;

      // If not, skip this iteration of the loop and try again.
      if(!consistentColour) continue;
    }

    // If desired, check that the current mode is far enough (in world coordinates) from the modes of any previously selected correspondences.
    if(checkMinDistanceBetweenSampledModes)
    {
      bool farEnough = true;

      for(int j = 0; j < correspondencesFound; ++j)
      {
        const int otherRasterIdx = selectedRasterIndices[j];
        const int otherModeIdx = selectedModeIndices[j];
        const ScorePrediction& otherPrediction = predictionsData[otherRasterIdx];
        const Vector3f otherWorldPt = otherPrediction.elts[otherModeIdx].position;

        const Vector3f diff = otherWorldPt - worldPt;
        const float distSq = dot(diff, diff);
        if(distSq < minSqDistanceBetweenSampledModes)
        {
          farEnough = false;
          break;
        }
      }

      if(!farEnough) continue;
    }

    // If desired, check, for each previously selected correspondence, that:
    //
    // (i)  The current keypoint is far enough (in camera coordinates) from the keypoint of the previously selected correspondence.
    // (ii) The distance between the current keypoint and the keypoint of the previously selected correspondence is similar enough
    //      to the distance between the current mode and the mode of the previously selected correspondence.
    //
    // The purpose of these checks is to ensure that the triangle formed by the points in each space is non-degenerate,
    // and that the transformation between the two triangles is quasi-rigid.
    if(checkRigidTransformationConstraint)
    {
      bool violatesConditions = false;

      for(int j = 0; j < correspondencesFound; ++j)
      {
        // (i) Check that the current keypoint is far enough (in camera coordinates) from the other keypoint.
        const int otherRasterIdx = selectedRasterIndices[j];
        const ScorePrediction& otherPrediction = predictionsData[otherRasterIdx];
        const Vector3f otherCameraPt = keypointsData[otherRasterIdx].position;

        const Vector3f diffCamera = otherCameraPt - cameraPt;
        const float distCameraSq = dot(diffCamera, diffCamera);
        if(distCameraSq < minSqDistanceBetweenSampledModes)
        {
          violatesConditions = true;
          break;
        }

        // (ii) Check that the distance between the current keypoint and the other keypoint is similar enough to the distance
        //      between the current mode and the other mode.
        const int otherModeIdx = selectedModeIndices[j];
        const Vector3f otherWorldPt = otherPrediction.elts[otherModeIdx].position;

        const Vector3f diffWorld = otherWorldPt - worldPt;
        const float distWorld = length(diffWorld);
        const float distCamera = sqrtf(distCameraSq);
        if(fabsf(distCamera - distWorld) > 0.5f * maxTranslationErrorForCorrectPose)
        {
          violatesConditions = true;
          break;
        }
      }

      if(violatesConditions) continue;
    }

    // If we reach this point, we've found a valid correspondence, so save the raster index and mode index for later use.
    selectedRasterIndices[correspondencesFound] = rasterIdx;
    selectedModeIndices[correspondencesFound] = modeIdx;
    ++correspondencesFound;
  }

  // If we reached the iteration limit and didn't find enough correspondences, early out.
  if(correspondencesFound != PoseCandidate::KABSCH_CORRESPONDENCES_NEEDED) return false;

  // Populate the pose candidate. The actual pose will be computed later using a CPU-based implementation of the Kabsch
  // algorithm (we don't currently have a GPU-based implementation of Kabsch).
  poseCandidate.energy = 0.0f;

  // Copy the corresponding camera and world points into the pose candidate.
  for(int i = 0; i < correspondencesFound; ++i)
  {
    const int rasterIdx = selectedRasterIndices[i];
    const int modeIdx = selectedModeIndices[i];

    const Keypoint3DColour& keypoint = keypointsData[rasterIdx];
    const ScorePrediction& prediction = predictionsData[rasterIdx];
    const Keypoint3DColourCluster& mode = prediction.elts[modeIdx];

    poseCandidate.pointsCamera[i] = keypoint.position;
    poseCandidate.pointsWorld[i] = mode.position;
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
inline int sample_inlier(const Keypoint3DColour *keypointsData, const ScorePrediction *predictionsData, const Vector2i& imgSize, RNG& randomGenerator, int *inlierMaskData = NULL)
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
