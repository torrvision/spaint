/**
 * spaint: ForestUtil.cpp
 */

#include "randomforest/ForestUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::vector<rafl::Descriptor_CPtr> ForestUtil::make_descriptors(const ORUtils::MemoryBlock<float>& featuresMB, size_t sampledVoxelCount, size_t featureCount)
{
  // Make the descriptors.
  featuresMB.UpdateHostFromDevice();
  const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
  std::vector<rafl::Descriptor_CPtr> descriptors(sampledVoxelCount);
  size_t descriptorIndex = 0;
  for(size_t i = 0; i < sampledVoxelCount; ++i)
  {
    // Copy the relevant features into a descriptor and add it.
    const float *featuresForDescriptor = features + i * featureCount;
    descriptors[descriptorIndex++].reset(new rafl::Descriptor(featuresForDescriptor, featuresForDescriptor + featureCount));
  }

  return descriptors;
}

}
