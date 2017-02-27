/**
 * grove: DecisionForest_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "DecisionForest_CPU.h"

#include "../shared/DecisionForest_Shared.h"

namespace grove {

template <typename DescriptorType, int TreeCount>
DecisionForest_CPU<DescriptorType, TreeCount>::DecisionForest_CPU(const std::string& fileName)
  : DecisionForest<DescriptorType, TreeCount>(fileName)
{}

template <typename DescriptorType, int TreeCount>
void DecisionForest_CPU<DescriptorType, TreeCount>::find_leaves(const DescriptorImage_CPtr& descriptors,
                                                                 LeafIndicesImage_Ptr& leafIndices) const
{
  const NodeEntry* forestTexture = this->m_nodeImage->GetData(MEMORYDEVICE_CPU);

  const Vector2i imgSize = descriptors->noDims;
  const DescriptorType* descriptorsData = descriptors->GetData(MEMORYDEVICE_CPU);

  leafIndices->ChangeDims(imgSize);
  LeafIndices* leafData = leafIndices->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      decision_forest_find_leaves_shared(forestTexture, descriptorsData, imgSize, leafData, x, y);
    }
  }
}

}
