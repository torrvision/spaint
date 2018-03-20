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
 * \param candidatePose       The candidate camera pose (a rigid transformation from camera -> world coordinates).
 * \param keypoints           The 3D keypoints extracted from an RGB-D image pair.
 * \param predictions         The SCoRe forest predictions associated with the keypoints.
 * \param inlierRasterIndices The raster indices of the overall set of "inlier" keypoints.
 * \param nbInliers           The overall number of "inlier" keypoints.
 * \param inlierStartIdx      The array index of the first "inlier" keypoint in inlierIndices to use when computing the energy sum.
 * \param inlierStep          The step between the array indices of the "inlier" keypoints to use when computing the energy sum.
 * \return                    The sum of the energies contributed by the "inlier" keypoints in the strided subset.
 */
_CPU_AND_GPU_CODE_
inline float compute_energy_sum_for_inlier_subset(const Matrix4f& candidatePose, const Keypoint3DColour *keypoints, const ScorePrediction *predictions,
                                                  const int *inlierRasterIndices, uint32_t nbInliers, uint32_t inlierStartIdx, uint32_t inlierStep)
{
  float energySum = 0.0f;

  // For each "inlier" keypoint in the strided subset:
  for(uint32_t inlierIdx = inlierStartIdx; inlierIdx < nbInliers; inlierIdx += inlierStep)
  {
    // Look up the raster index of the inlier and its position in camera space.
    const int inlierRasterIdx = inlierRasterIndices[inlierIdx];
    const Vector3f inlierCameraCoordinates = keypoints[inlierRasterIdx].position;

    // Compute the hypothesised position of the inlier in world space.
    const Vector3f inlierWorldCoordinates = candidatePose * inlierCameraCoordinates;

    // Get the prediction associated with the inlier.
    const ScorePrediction& pred = predictions[inlierRasterIdx];

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
 * \param candidatePose       The candidate camera pose (a rigid transformation from camera -> world coordinates).
 * \param keypoints           The 3D keypoints extracted from an RGB-D image pair.
 * \param predictions         The SCoRe forest predictions associated with the keypoints.
 * \param inlierRasterIndices The raster indices of the "inlier" keypoints that we will use to compute the energy sum.
 * \param nbInliers           The number of "inlier" keypoints.
 * \return                    The sum of the energies contributed by the "inlier" keypoints.
 */
_CPU_AND_GPU_CODE_
inline float compute_energy_sum_for_inliers(const Matrix4f& candidatePose, const Keypoint3DColour *keypoints, const ScorePrediction *predictions,
                                            const int *inlierRasterIndices, uint32_t nbInliers)
{
  const uint32_t inlierStartIdx = 0;
  const uint32_t inlierStep = 1;
  return compute_energy_sum_for_inlier_subset(candidatePose, keypoints, predictions, inlierRasterIndices, nbInliers, inlierStartIdx, inlierStep);
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

/**
 * \brief Computes the best mode in world space for the specified candidate pose and "inlier" keypoint, and stores both
 *        this mode and the inlier's position in camera space into arrays for use during pose optimisation.
 *
 * \param candidateIdx        The array index of the pose candidate being considered (in the pose candidates array).
 * \param inlierIdx           The array index of the "inlier" keypoint being considered (in the inlier raster indices array).
 * \param keypoints           The 3D keypoints extracted from an RGB-D image pair.
 * \param predictions         The SCoRe forest predictions associated with the keypoints.
 * \param inlierRasterIndices The raster indices of the "inlier" keypoints that we will use to compute the energy sum.
 * \param nbInliers           The overall number of "inlier" keypoints.
 * \param poseCandidates      The pose candidates.
 * \param inlierThreshold     The furthest the best mode's position can be from the inlier's position in world space for it to be usable.
 * \param inlierCameraPoints  The array into which to write the inlier's position in camera space.
 * \param inlierModes         The array into which to write the best mode for the specified candidate pose and inlier.
 */
_CPU_AND_GPU_CODE_
inline void prepare_inlier_for_optimisation(uint32_t candidateIdx, uint32_t inlierIdx, const Keypoint3DColour *keypoints, const ScorePrediction *predictions, const int *inlierRasterIndices,
                                            uint32_t nbInliers, const PoseCandidate *poseCandidates, const float inlierThreshold, Vector4f *inlierCameraPoints, Keypoint3DColourCluster *inlierModes)
{
  const int inlierRasterIdx = inlierRasterIndices[inlierIdx];
  const PoseCandidate& poseCandidate = poseCandidates[candidateIdx];
  const Vector3f inlierCameraPosition = keypoints[inlierRasterIdx].position;
  const ScorePrediction& prediction = predictions[inlierRasterIdx];

  // Try to find the index of the mode associated with the inlier whose position is closest to the position of the
  // inlier in world space as predicted by the specified candidate pose. (We assume the inlier itself is valid and
  // has at least one mode, since we checked that when we selected it.)
  const Vector3f inlierWorldPosition = poseCandidate.cameraPose * inlierCameraPosition;
  const int bestModeIdx = score_prediction_get_best_mode(prediction, inlierWorldPosition);

  // If we cannot find such a mode, it means that the inlier should never have been selected, so throw.
  // This is purely defensive, and should never happen in practice.
  if(bestModeIdx < 0 || bestModeIdx >= prediction.size)
  {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
    printf("Error: Could not find a best mode for the specified candidate pose and inlier keypoint\n");
    asm("trap;");
#else
    throw std::runtime_error("Error: Could not find a best mode for the specified candidate pose and inlier keypoint");
#endif
  }

  const Keypoint3DColourCluster& bestMode = prediction.elts[bestModeIdx];

  // Determine the location in the output arrays into which to write the inlier's camera position and the best mode.
  const uint32_t outputIdx = candidateIdx * nbInliers + inlierIdx;

  // If the best mode's position is too far from the inlier's position in world space, record an invalid position
  // for the inlier in camera space, and early out.
  if(length(bestMode.position - inlierWorldPosition) >= inlierThreshold)
  {
    inlierCameraPoints[outputIdx] = Vector4f(0.0f);
    return;
  }

  // Otherwise, record both the inlier's position in camera space and the best mode for use during pose optimisation.
  inlierCameraPoints[outputIdx] = Vector4f(inlierCameraPosition, 1.0f);
  inlierModes[outputIdx] = bestMode;
}

/**
 * \brief Tries to sample the raster index of a valid keypoint whose prediction has at least one modal cluster.
 *
 * \tparam useMask    Whether or not to record the sampled keypoints in a persistent mask (to prevent them being sampled twice).
 * \tparam RNG        The type of random number generator use for sampling (e.g. CPURNG or CUDARNG).
 *
 * \param keypoints   The 3D keypoints extracted from an RGB-D image pair.
 * \param predictions The SCoRe forest predictions associated with the keypoints.
 * \param imgSize     The size of the input keypoints and predictions images.
 * \param rng         The random number generator to use for sampling.
 * \param inliersMask The mask used to avoid sampling keypoint indices twice. Can be NULL if useMask is false.
 *
 * \return            The raster index of the sampled keypoint (if any), or -1 otherwise.
 */
template <bool useMask, typename RNG>
_CPU_AND_GPU_CODE_TEMPLATE_
inline int sample_inlier(const Keypoint3DColour *keypoints, const ScorePrediction *predictions, const Vector2i& imgSize, RNG& rng, int *inliersMask = NULL)
{
  int inlierRasterIdx = -1;

  // Attempt to sample a suitable keypoint up to SAMPLE_INLIER_ITERATIONS times.
  for(int i = 0; i < SAMPLE_INLIER_ITERATIONS; ++i)
  {
    // Randomly generate a keypoint index.
    const int rasterIdx = rng.generate_int_from_uniform(0, imgSize.width * imgSize.height - 1);

    // Check whether the corresponding keypoint is valid and has at least one modal cluster. If not, early out.
    if(!keypoints[rasterIdx].valid || predictions[rasterIdx].size == 0) continue;

    // If we're using the mask, check whether or not the keypoint has already been sampled.
    bool valid = !useMask;

    if(useMask)
    {
      int *maskPtr = &inliersMask[rasterIdx];
      int maskValue = -1;

      // Atomically increment the relevant pixel in the mask, capturing its original value in the process.
#ifdef __CUDACC__
      maskValue = atomicAdd(maskPtr, 1);
#else
    #ifdef WITH_OPENMP
      #pragma omp atomic capture
    #endif
      maskValue = (*maskPtr)++;
#endif

      // Accept the keypoint iff the original value of the corresponding pixel in the mask was zero.
      valid = maskValue == 0;
    }

    // If we're not using the mask, or the keypoint hadn't already been sampled, accept the keypoint and return it.
    if(valid)
    {
      inlierRasterIdx = rasterIdx;
      break;
    }
  }

  return inlierRasterIdx;
}

}

#endif
