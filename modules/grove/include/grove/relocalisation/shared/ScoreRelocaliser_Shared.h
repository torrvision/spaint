/**
 * grove: ScoreRelocaliser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSHARED
#define H_GROVE_SCORERELOCALISERSHARED

#include "../interface/ScoreRelocaliser.h"

#include <ORUtils/PlatformIndependence.h>
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
 * \param x                The x coordinate of the keypoint.
 * \param y                The y coordinate of the keypoint.
 * \param leafPredictions  A pointer to the storage area holding all the ScorePredictions associated to forest leaves.
 * \param leafIndices      A pointer to the storage area where the leaf indices for the current example are stored.
 * \param outPredictions   A pointer to the storage area that will hold the final merged prediction.
 * \param imgSize          The dimensions of the leafIndices and outPredictions arrays.
 * \param nbMaxPredictions The maximum number of predictions to merge for each output prediction.
 */
template <int TREE_COUNT>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void merge_predictions_for_keypoint(int x, int y, const ScorePrediction *leafPredictions, const ORUtils::VectorX<int, TREE_COUNT> *leafIndices,
                                           ScorePrediction *outPredictions, Vector2i imgSize, int nbMaxPredictions)
{
  typedef ORUtils::VectorX<int, TREE_COUNT> LeafIndices;

  // Compute the linear index to the current leaves/output prediction.
  const int linearIdx = y * imgSize.width + x;

  // Grab a copy of the relevant leaves in a local variable.
  const LeafIndices selectedLeaves = leafIndices[linearIdx];

  // Setup and zero the indices of the current mode for each prediction
  int treeModeIdx[TREE_COUNT];
  for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    treeModeIdx[treeIdx] = 0;
  }

  // Copy the selected prediction for each leaf. This is to have them contigous in memory for the following operations.
  ScorePrediction selectedPredictions[TREE_COUNT];
  for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    selectedPredictions[treeIdx] = leafPredictions[selectedLeaves[treeIdx]];
  }

  // Not using a reference to the output image to avoid global memory accesses.
  ScorePrediction finalPrediction;
  finalPrediction.size = 0;

  // Merge the first nbMaxPredictions from the selected cluster arrays.
  // The assumption is that the modal clusters in leafPredictions are already sorted by descending number of inliers.
  while(finalPrediction.size < nbMaxPredictions)
  {
    int bestTreeIdx = 0;
    int bestTreeNbInliers = 0;

    // Find the tree having the first (not yet processed) mode with most inliers
    for(int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
    {
      // Index of the mode that we have to check for the current tree.
      const int currentModeIdx = treeModeIdx[treeIdx];

      // The number of modes for the prediction associated to the current tree.
      const int predictionModeCount = selectedPredictions[treeIdx].size;

      // If the prediction has less modes than currentModeIdx we cannot do anything for this tree.
      if(predictionModeCount <= currentModeIdx)
      {
        continue;
      }

      // The mode that we are evaluating (the first non processed mode).
      const Keypoint3DColourCluster& currentMode = selectedPredictions[treeIdx].elts[currentModeIdx];

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
    finalPrediction.elts[finalPrediction.size++] = selectedPredictions[bestTreeIdx].elts[treeModeIdx[bestTreeIdx]];

    // Increment the starting index for the associated tree.
    treeModeIdx[bestTreeIdx]++;
  }

  // Finally, store the prediction in the output image.
  outPredictions[linearIdx] = finalPrediction;
}

}

#endif
