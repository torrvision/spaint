/**
 * grove: DecisionForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFORESTSHARED
#define H_GROVE_DECISIONFORESTSHARED

#include <ORUtils/PlatformIndependence.h>
#include <ORUtils/Vector.h>

namespace grove {

/**
 * \brief Find the leaf indices associated to a descriptor.
 *
 * \param forestTexture      The forest indexing structure.
 * \param descriptorsData    Pointer to an image of descriptors.
 * \param leafData           Pointer to the image wherein the leaf indices will be stored.
 * \param descriptorsImgSize Size of the descriptor image.
 * \param x                  The x coordinate of the descriptor to evaluate.
 * \param y                  The y coordinate of the descriptor to evaluate.
 */
template <typename NodeType, typename DescriptorType, int TreeCount>
_CPU_AND_GPU_CODE_TEMPLATE_ inline void decision_forest_find_leaves_shared(const NodeType *forestTexture,
                                                                           const DescriptorType *descriptorsData,
                                                                           ORUtils::VectorX<int, TreeCount> *leafData,
                                                                           Vector2i descriptorsImgSize,
                                                                           int x,
                                                                           int y)
{
  const int nbTrees = TreeCount;
  const int rasterDescriptorIdx = y * descriptorsImgSize.width + x;
  const DescriptorType &currentDescriptor = descriptorsData[rasterDescriptorIdx];

  for (int treeIdx = 0; treeIdx < nbTrees; ++treeIdx)
  {
    uint32_t currentNodeIdx = 0;
    NodeType node = forestTexture[currentNodeIdx * nbTrees + treeIdx];
    bool isLeaf = node.leafIdx >= 0;

    while (!isLeaf)
    {
      // Evaluate split function
      currentNodeIdx = node.leftChildIdx + (currentDescriptor.data[node.featureIdx] > node.featureThreshold);

      // Update node
      node = forestTexture[currentNodeIdx * nbTrees + treeIdx];
      isLeaf = node.leafIdx >= 0; // A bit redundant (could test in the while condition), but clearer.
    }

    leafData[rasterDescriptorIdx][treeIdx] = node.leafIdx;
  }
}

} // namespace grove

#endif
