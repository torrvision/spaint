/**
 * grove: DecisionForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFORESTSHARED
#define H_GROVE_DECISIONFORESTSHARED

#include <ORUtils/PlatformIndependence.h>

namespace grove {

template <typename NodeType, typename DescriptorType, typename LeafType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void decision_forest_find_leaves_shared(
    const NodeType* forestTexture,
    const DescriptorType* descriptorsData, Vector2i descriptorsImgSize,
    LeafType* leafData, int x, int y)
{
  // LeafType is a VectorX<int, count>, thus has a value_size enum defining its length.
  const int nbTrees = LeafType::value_size;
  const int rasterDescriptorIdx = y * descriptorsImgSize.width + x;
  const DescriptorType &currentDescriptor = descriptorsData[rasterDescriptorIdx];

  for (uint32_t treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    uint32_t currentNodeIdx = 0;
    NodeType node = forestTexture[currentNodeIdx * nbTrees + treeIdx];
    bool isLeaf = node.leafIdx >= 0;

    while (!isLeaf)
    {
      // Evaluate split function
      currentNodeIdx = node.leftChildIdx
          + (currentDescriptor.data[node.featureIdx] > node.featureThreshold);

      // Update node
      node = forestTexture[currentNodeIdx * nbTrees + treeIdx];
      isLeaf = node.leafIdx >= 0; // A bit redundant (could test in the while condition), but clearer.
    }

    leafData[rasterDescriptorIdx][treeIdx] = node.leafIdx;
  }
}

}

#endif
