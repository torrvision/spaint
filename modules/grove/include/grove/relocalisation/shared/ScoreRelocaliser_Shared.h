/**
 * grove: ScoreRelocaliser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSHARED
#define H_GROVE_SCORERELOCALISERSHARED

#include "../../scoreforests/ScorePrediction.h"

namespace grove {

/**
 * \brief Merges the SCoRe predictions associated with the specified keypoint into a single SCoRe prediction.
 *
 * \note  Each keypoint will have a SCoRe prediction (set of clusters) from each tree in the forest, obtained by passing
 *        the keypoint's descriptor down each tree and collecting a SCoRe prediction from each resulting leaf.
 * \note  Merging is performed by taking the largest clusters from each leaf. The assumption is that the modal cluters in
 *        each leaf are already sorted in non-increasing order of size.
 *
 * \param x                 The x coordinate of the keypoint.
 * \param y                 The y coordinate of the keypoint.
 * \param leafIndices       A pointer to the image containing the indices of the leaves (in the different trees) associated with each keypoint/descriptor pair.
 * \param predictionsBlock  A pointer to the storage area holding all of the SCoRe predictions associated with the forest leaves.
 * \param imgSize           The dimensions of the leafIndices and outputPredictions images.
 * \param maxClusterCount   The maximum number of clusters to keep for each output prediction.
 * \param outputPredictions A pointer to the image into which to store the merged SCoRe predictions.
 */
template <int TREE_COUNT>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void merge_predictions_for_keypoint(int x, int y, const ORUtils::VectorX<int,TREE_COUNT> *leafIndices, const ScorePrediction *predictionsBlock,
                                           Vector2i imgSize, int maxClusterCount, ScorePrediction *outputPredictions)
{
  typedef ORUtils::VectorX<int,TREE_COUNT> LeafIndices;

  // Compute the raster index of the keypoint whose predictions we want to merge.
  const int keypointRasterIdx = y * imgSize.width + x;

  // Copy the leaf indices associated with the keypoint into a local array.
  const LeafIndices leafIndicesForKeypoint = leafIndices[keypointRasterIdx];

  // Copy the input predictions associated with the keypoint's leaves into a contiguous local array (this makes it more efficient to access them later).
  ScorePrediction inputPredictions[TREE_COUNT];
  for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    inputPredictions[treeIdx] = predictionsBlock[leafIndicesForKeypoint[treeIdx]];
  }

  // Make an array of indices in which each element denotes the current mode to consider in each of the input predictions.
  // Initially, this will be the first/biggest mode in each prediction. Our strategy will be to repeatedly copy the biggest
  // remaining mode (across all the input predictions) across to the output prediction until the output prediction is full.
  // These indices exist to keep track of which modes have already been copied across.
  int currentModeIndices[TREE_COUNT];
  for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    currentModeIndices[treeIdx] = 0;
  }

  ScorePrediction outputPrediction;
  outputPrediction.size = 0;

  // While the output prediction is not yet full:
  while(outputPrediction.size < maxClusterCount)
  {
    int bestTreeIdx = 0;
    int bestNbInliers = 0;

    // Find the input prediction whose biggest as-yet-unconsidered mode has the most inliers.
    for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
    {
      // Look up the index of the biggest as-yet-unconsidered mode for this input prediction.
      const int currentModeIdx = currentModeIndices[treeIdx];

      // If we've already considered all of the modes for this input prediction, skip it and continue.
      if(currentModeIdx >= inputPredictions[treeIdx].size) continue;

      const Keypoint3DColourCluster& currentMode = inputPredictions[treeIdx].elts[currentModeIdx];

      // If the biggest as-yet-unprocessed mode for this input prediction has more inliers than the current best mode:
      if(currentMode.nbInliers > bestNbInliers)
      {
        // Update the best tree index and number of inliers accordingly.
        bestTreeIdx = treeIdx;
        bestNbInliers = currentMode.nbInliers;
      }
    }

    // If there were no valid modes remaining in any input prediction, early out.
    if(bestNbInliers == 0) break;

    // Otherwise, copy the chosen mode into the output prediction, and increment the current mode index for the associated input prediction.
    outputPrediction.elts[outputPrediction.size++] = inputPredictions[bestTreeIdx].elts[currentModeIndices[bestTreeIdx]];
    ++currentModeIndices[bestTreeIdx];
  }

  // Finally, copy the output prediction into the output predictions image.
  outputPredictions[keypointRasterIdx] = outputPrediction;
}

}

#endif
