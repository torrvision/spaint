/**
 * spaint: ScoreForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SCOREFORESTSHARED
#define H_SPAINT_SCOREFORESTSHARED

namespace spaint
{
_CPU_AND_GPU_CODE_
inline void evaluate_forest_shared(const ScoreForest::NodeEntry* forestTexture,
    const RGBDPatchFeature* featureData, const Vector2i &imgSize,
    ScoreForest::LeafIndices* leafData, int x, int y)
{
  const int linear_feature_idx = y * imgSize.width + x;
  const RGBDPatchFeature &currentFeature = featureData[linear_feature_idx];

  for (int treeIdx = 0; treeIdx < GPUFOREST_NTREES; ++treeIdx)
  {
    int currentNodeIdx = 0;
    ScoreForest::NodeEntry node = forestTexture[currentNodeIdx
        * GPUFOREST_NTREES + treeIdx];
    bool isLeaf = node.leafIdx >= 0;

    while (!isLeaf)
    {
      // Evaluate feature
      currentNodeIdx = node.leftChildIdx
          + (currentFeature.data[node.featureIdx] > node.featureThreshold);

      // Update node
      node = forestTexture[currentNodeIdx * GPUFOREST_NTREES + treeIdx];
      isLeaf = node.leafIdx >= 0; // a bit redundant but clearer
    }

    leafData[linear_feature_idx][treeIdx] = node.leafIdx;
  }
}

_CPU_AND_GPU_CODE_
inline void get_prediction_for_leaf_shared(
    const ScorePrediction* leafPredictions,
    const ScoreForest::LeafIndices* leafIndices,
    ScorePrediction* outPredictions, Vector2i imgSize, int x, int y)
{
  const int linearIdx = y * imgSize.width + x;
  const ScoreForest::LeafIndices selectedLeaves = leafIndices[linearIdx];

  // Setup the indices of the selected mode for each prediction
  int treeModeIdx[GPUFOREST_NTREES];
  for (int treeIdx = 0; treeIdx < GPUFOREST_NTREES; ++treeIdx)
  {
    treeModeIdx[treeIdx] = 0;
  }

  // TODO: maybe copying them is faster...
  const ScorePrediction *selectedPredictions[GPUFOREST_NTREES];
  for (int treeIdx = 0; treeIdx < GPUFOREST_NTREES; ++treeIdx)
  {
    selectedPredictions[treeIdx] = &leafPredictions[selectedLeaves[treeIdx]];
  }

  ScorePrediction &outPrediction = outPredictions[linearIdx];
  outPrediction.nbModes = 0;

  // Merge first MAX_MODES from the sorted mode arrays
  while (outPrediction.nbModes < ScorePrediction::MAX_MODES)
  {
    int bestTreeIdx = 0;
    int bestTreeNbInliers = 0;

    // Find the tree with most inliers
    for (int treeIdx = 0; treeIdx < GPUFOREST_NTREES; ++treeIdx)
    {
      if (selectedPredictions[treeIdx]->nbModes > treeModeIdx[treeIdx]
          && selectedPredictions[treeIdx]->modes[treeModeIdx[treeIdx]].nbInliers
              > bestTreeNbInliers)
      {
        bestTreeIdx = treeIdx;
        bestTreeNbInliers =
            selectedPredictions[treeIdx]->modes[treeModeIdx[treeIdx]].nbInliers;
      }
    }

    if (bestTreeNbInliers == 0)
    {
      // No more modes
      break;
    }

    // Copy its mode into the output array, increment its index
    outPrediction.modes[outPrediction.nbModes] =
        selectedPredictions[bestTreeIdx]->modes[treeModeIdx[bestTreeIdx]];
    outPrediction.nbModes++;
    treeModeIdx[bestTreeIdx]++;
  }
}
}

#endif
