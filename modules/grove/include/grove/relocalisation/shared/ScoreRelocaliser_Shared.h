/**
 * grove: ScoreRelocaliser_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_SCORERELOCALISERSHARED
#define H_GROVE_SCORERELOCALISERSHARED

#include "../interface/ScoreRelocaliser.h"

#include <ORUtils/PlatformIndependence.h>
#include <ORUtils/Vector.h>

#include "../../clustering/base/Prediction3DColour.h"


namespace grove {

template<int TREE_COUNT>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void get_prediction_for_leaf_shared(
    const ScorePrediction* leafPredictions,
    const ORUtils::VectorX<int, TREE_COUNT> *leafIndices,
    ScorePrediction* outPredictions, Vector2i imgSize, int x, int y)
{
  typedef ORUtils::VectorX<int, TREE_COUNT> LeafIndices;

  const int linearIdx = y * imgSize.width + x;
  const LeafIndices selectedLeaves = leafIndices[linearIdx];

  // Setup the indices of the selected mode for each prediction
  int treeModeIdx[TREE_COUNT];
  for (int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    treeModeIdx[treeIdx] = 0;
  }

  // Copy the prediction selected for each tree. This is to have them contigous in memory for the following operations.
  ScorePrediction selectedPredictions[TREE_COUNT];
  for (int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
  {
    selectedPredictions[treeIdx] = leafPredictions[selectedLeaves[treeIdx]];
  }

  // Not using a reference to the output image to avoid global memory accesses.
  ScorePrediction finalPrediction;
  finalPrediction.nbClusters = 0;

  // Merge first MAX_MODES from the sorted mode arrays.
  // The assumption is that the modes in leafPredictions are already sorted by descending number of inliers.
  while (finalPrediction.nbClusters < ScorePrediction::MAX_CLUSTERS)
  {
    int bestTreeIdx = 0;
    int bestTreeNbInliers = 0;

    // Find the tree having the first (not yet processed) mode with most inliers
    for (int treeIdx = 0; treeIdx < TREE_COUNT; ++treeIdx)
    {
      // Index of the mode that we have to check for the current tree.
      const int currentModeIdx = treeModeIdx[treeIdx];

      // The number of modes for the prediction associated to the current tree.
      const int predictionModeCount = selectedPredictions[treeIdx].nbClusters;

      // If the prediction has less modes than the currentModeIdx we cannot do anything for this tree.
      if(predictionModeCount <= currentModeIdx)
      {
        continue;
      }

      // The mode that we are evaluating.
      const Mode3DColour &currentMode = selectedPredictions[treeIdx].clusters[currentModeIdx];

      // The first not processed mode has more inliers than the current best mode
      if (currentMode.nbInliers > bestTreeNbInliers)
      {
        // Update best tree index and number of inliers.
        bestTreeIdx = treeIdx;
        bestTreeNbInliers = currentMode.nbInliers;
      }
    }

    // No valid modes in any tree.
    if (bestTreeNbInliers == 0)
    {
      break;
    }

    // Copy the chosen mode into the output array.
    finalPrediction.clusters[finalPrediction.nbClusters++] = selectedPredictions[bestTreeIdx].clusters[treeModeIdx[bestTreeIdx]];

    // Increment the starting index for the associated tree.
    treeModeIdx[bestTreeIdx]++;
  }

  // Store the prediction in the output image.
  outPredictions[linearIdx] = finalPrediction;
}

}

#endif
