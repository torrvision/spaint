/**
 * grove: DecisionForest_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "DecisionForest_CPU.h"

#include "../shared/DecisionForest_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename DescriptorType, int TreeCount>
DecisionForest_CPU<DescriptorType,TreeCount>::DecisionForest_CPU(const std::string& filename)
: Base(filename)
{}

#ifdef WITH_SCOREFORESTS
template <typename DescriptorType, int TreeCount>
DecisionForest_CPU<DescriptorType,TreeCount>::DecisionForest_CPU(const EnsembleLearner& pretrainedForest)
: Base(pretrainedForest)
{}
#endif

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename DescriptorType, int TreeCount>
void DecisionForest_CPU<DescriptorType,TreeCount>::find_leaves(const DescriptorImage_CPtr& descriptors, LeafIndicesImage_Ptr& leafIndices) const
{
  // Ensure that the leaf indices image is the same size as the descriptors image.
  const Vector2i imgSize = descriptors->noDims;
  leafIndices->ChangeDims(imgSize);

  // Compute the leaf indices associated with each descriptor in the descriptors image.
  const DescriptorType *descriptorsPtr = descriptors->GetData(MEMORYDEVICE_CPU);
  const NodeEntry *nodeImage = this->m_nodeImage->GetData(MEMORYDEVICE_CPU);
  LeafIndices *leafIndicesPtr = leafIndices->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int y = 0; y < imgSize.y; ++y)
  {
    for(int x = 0; x < imgSize.x; ++x)
    {
      compute_leaf_indices(x, y, descriptorsPtr, imgSize, nodeImage, leafIndicesPtr);
    }
  }
}

}
