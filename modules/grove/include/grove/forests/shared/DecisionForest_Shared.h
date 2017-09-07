/**
 * grove: DecisionForest_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFOREST_SHARED
#define H_GROVE_DECISIONFOREST_SHARED

#include <ORUtils/Vector.h>

namespace grove {

/**
 * \brief Finds the leaf indices associated with a descriptor and writes them into the leaf indices image.
 *
 * \param x           The x coordinate of the descriptor to evaluate.
 * \param y           The y coordinate of the descriptor to evaluate.
 * \param descriptors The descriptors image.
 * \param imgSize     The size of the descriptors and leaf indices images.
 * \param nodeImage   The forest indexing structure.
 * \param leafIndices An image in which to store the leaf indices computed for the descriptor.
 */
template <typename NodeType, typename DescriptorType, int TreeCount>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_leaf_indices(int x, int y, const DescriptorType *descriptors, Vector2i imgSize,
                                 const NodeType *nodeImage, ORUtils::VectorX<int,TreeCount> *leafIndices)
{
  // Look up the descriptor whose leaf indices we want to compute.
  const int rasterIdx = y * imgSize.width + x;
  const DescriptorType& currentDescriptor = descriptors[rasterIdx];

  // For each tree in the forest:
  for(int treeIdx = 0; treeIdx < TreeCount; ++treeIdx)
  {
    // Start from the root node and iteratively walk down the tree until a leaf is reached.
    uint32_t currentNodeIdx = 0;
    NodeType node = nodeImage[currentNodeIdx * TreeCount + treeIdx];

    // Note: This is for clarity: we could (if desired) test node.leafIdx directly in the while condition.
    bool isLeaf = node.leafIdx >= 0;

    while(!isLeaf)
    {
      // Descend to either the left or right subtree.
      currentNodeIdx = node.leftChildIdx + static_cast<int>(currentDescriptor.data[node.featureIdx] > node.featureThreshold);
      node = nodeImage[currentNodeIdx * TreeCount + treeIdx];
      isLeaf = node.leafIdx >= 0;
    }

    // Write the index of the leaf that has been reached into the leaf indices image.
    leafIndices[rasterIdx][treeIdx] = node.leafIdx;
  }
}

}

#endif
