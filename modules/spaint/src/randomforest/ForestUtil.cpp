/**
 * spaint: ForestUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "randomforest/ForestUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::vector<rafl::Descriptor_CPtr> ForestUtil::make_descriptors(const ORUtils::MemoryBlock<float>& featuresMB, size_t descriptorCount, size_t featureCount)
{
  // Make sure that the features are available and up-to-date on the CPU.
  featuresMB.UpdateHostFromDevice();

  // Make the rafl feature descriptors.
  const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
  std::vector<rafl::Descriptor_CPtr> descriptors(descriptorCount);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int i = 0; i < static_cast<int>(descriptorCount); ++i)
  {
    // Copy the relevant features into a descriptor and add it.
    const float *featuresForDescriptor = features + i * featureCount;
    descriptors[i].reset(new rafl::Descriptor(featuresForDescriptor, featuresForDescriptor + featureCount));
  }

  return descriptors;
}

}
