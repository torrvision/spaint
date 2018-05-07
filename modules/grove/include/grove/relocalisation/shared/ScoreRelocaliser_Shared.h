/**
 * grove: ScoreRelocaliser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSHARED
#define H_GROVE_SCORERELOCALISERSHARED

#include <ORUtils/Vector.h>

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
 * \param predictionsBlock  A pointer to the storage area holding all of the SCoRe predictions associated with the forest leaves.
 * \param leafIndices       A pointer to the image containing the indices of the leaves (in the different trees) associated with each keypoint/descriptor pair.
 * \param outputPredictions A pointer to the image into which to store the merged SCoRe predictions.
 * \param imgSize           The dimensions of the leafIndices and outputPredictions images.
 * \param nbMaxPredictions  The maximum number of predictions to merge for each output prediction.
 */
template <int TREE_COUNT>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void merge_predictions_for_keypoint(int x, int y, const ScorePrediction *predictionsBlock, const ORUtils::VectorX<int,TREE_COUNT> *leafIndices,
                                           ScorePrediction *outputPredictions, Vector2i imgSize, int nbMaxPredictions)
{
  typedef ORUtils::VectorX<int,TREE_COUNT> LeafIndices;

  // Compute the raster index of the keypoint whose predictions we want to merge.
  const int keypointRasterIdx = y * imgSize.width + x;

  // Copy the leaf indices associated with the keypoint into a local array.
  const LeafIndices leafIndicesForKeypoint = leafIndices[keypointRasterIdx];

  // Setup and zero the indices of the current mode for each prediction
  int treeModeIdx[TREE_COUNT];
  for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    treeModeIdx[treeIdx] = 0;
  }

  // Copy the predictions associated with the keypoint's leaves into a contiguous local array (this makes it more efficient to access them later).
  ScorePrediction predictionsForKeypoint[TREE_COUNT];
  for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    predictionsForKeypoint[treeIdx] = predictionsBlock[leafIndicesForKeypoint[treeIdx]];
  }

  // Not using a reference to the output image to avoid global memory accesses.
  ScorePrediction outputPrediction;
  outputPrediction.size = 0;

  // Merge the first nbMaxPredictions from the selected cluster arrays.
  // The assumption is that the modal clusters in leafPredictions are already sorted by descending number of inliers.
  while(outputPrediction.size < nbMaxPredictions)
  {
    int bestTreeIdx = 0;
    int bestTreeNbInliers = 0;

    // Find the tree having the first (not yet processed) mode with most inliers
    for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
    {
      // Index of the mode that we have to check for the current tree.
      const int currentModeIdx = treeModeIdx[treeIdx];

      // The number of modes for the prediction associated to the current tree.
      const int predictionModeCount = predictionsForKeypoint[treeIdx].size;

      // If the prediction has less modes than currentModeIdx we cannot do anything for this tree.
      if(predictionModeCount <= currentModeIdx)
      {
        continue;
      }

      // The mode that we are evaluating (the first non processed mode).
      const Keypoint3DColourCluster& currentMode = predictionsForKeypoint[treeIdx].elts[currentModeIdx];

      // The current mode has more inliers than the currently best mode.
      if(currentMode.nbInliers > bestTreeNbInliers)
      {
        // Update best tree index and number of inliers.
        bestTreeIdx = treeIdx;
        bestTreeNbInliers = currentMode.nbInliers;
      }
    }

    // No valid modes in any tree. Early out.
    if(bestTreeNbInliers == 0)
    {
      break;
    }

    // Copy the chosen mode into the output array.
    outputPrediction.elts[outputPrediction.size++] = predictionsForKeypoint[bestTreeIdx].elts[treeModeIdx[bestTreeIdx]];

    // Increment the starting index for the associated tree.
    treeModeIdx[bestTreeIdx]++;
  }

  // Finally, store the prediction in the output image.
  outputPredictions[keypointRasterIdx] = outputPrediction;
}

}

#endif
