/**
 * spaint: GPUForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_GPUFORESTSHARED
#define H_SPAINT_GPUFORESTSHARED

namespace spaint
{
_CPU_AND_GPU_CODE_
inline void evaluate_forest_shared(const GPUForestNode* forestTexture,
    const RGBDPatchFeature* featureData, const Vector2i &imgSize,
    GPUForest::LeafIndices* leafData, int x, int y)
{
  const int linear_feature_idx = y * imgSize.width + x;
  const RGBDPatchFeature &currentFeature = featureData[linear_feature_idx];

  for (int treeIdx = 0; treeIdx < GPUForest::NTREES; ++treeIdx)
  {
    int currentNodeIdx = 0;
    GPUForestNode node = forestTexture[currentNodeIdx * GPUForest::NTREES
        + treeIdx];
    bool isLeaf = node.leafIdx >= 0;

    while (!isLeaf)
    {
      // Evaluate feature
      currentNodeIdx = node.leftChildIdx
          + (currentFeature.data[node.featureIdx] > node.featureThreshold);

      // Update node
      node = forestTexture[currentNodeIdx * GPUForest::NTREES + treeIdx];
      isLeaf = node.leafIdx >= 0; // a bit redundant but clearer
    }

    leafData[linear_feature_idx][treeIdx] = node.leafIdx;
  }
}
}

#endif
