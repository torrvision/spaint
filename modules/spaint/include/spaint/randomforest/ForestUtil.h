/**
 * spaint: ForestUtil.h
 */

#ifndef H_SPAINT_FORESTUTIL
#define H_SPAINT_FORESTUTIL

#include <ORUtils/MemoryBlock.h>

#include <rafl/examples/Example.h>

namespace spaint {

/**
 * \brief This struct provides utility functions that can make feature descriptors and examples for use with rafl random forests.
 */
struct ForestUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes rafl feature descriptors from feature descriptors that are stored implicitly and contiguously in an InfiniTAM memory block.
   *
   * \param featuresMB      The InfiniTAM memory block containing the feature descriptors.
   * \param descriptorCount The number of feature descriptors that are stored in the memory block.
   * \param featureCount    The number of features in a feature descriptor.
   * \return                The rafl feature descriptors.
   */
  static std::vector<rafl::Descriptor_CPtr> make_descriptors(const ORUtils::MemoryBlock<float>& featuresMB, size_t descriptorCount, size_t featureCount);

  /**
   * \brief TODO
   */
  template <typename Label>
  static std::vector<boost::shared_ptr<const rafl::Example<Label> > > make_examples(const ORUtils::MemoryBlock<float>& featuresMB,
                                                                                    const ORUtils::MemoryBlock<unsigned int>& sampledVoxelCountsMB, 
                                                                                    size_t featureCount, size_t maxVoxelsPerLabel, size_t maxLabelCount)
  {
    typedef boost::shared_ptr<const rafl::Example<Label> > Example_CPtr;

    // Determine the number of examples we are trying to make (one per voxel sampled from the scene).
    sampledVoxelCountsMB.UpdateHostFromDevice();
    const unsigned int *sampledVoxelCounts = sampledVoxelCountsMB.GetData(MEMORYDEVICE_CPU);
    size_t exampleCount = 0;
    for(size_t i = 0, size = sampledVoxelCountsMB.dataSize; i < size; ++i)
    {
      exampleCount += sampledVoxelCounts[i];
    }

    // Make the examples.
    featuresMB.UpdateHostFromDevice();
    const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
    std::vector<Example_CPtr> examples(exampleCount);
    size_t exampleIndex = 0;
    for(Label label = 0; label < static_cast<Label>(maxLabelCount); ++label)
    {
      for(size_t i = 0; i < sampledVoxelCounts[label]; ++i)
      {
        // Copy the features for the example into a descriptor.
        const float *featuresForExample = features + (label * maxVoxelsPerLabel + i) * featureCount;
        rafl::Descriptor_Ptr descriptor(new rafl::Descriptor(featuresForExample, featuresForExample + featureCount));

        // Add the example.
        examples[exampleIndex++] = Example_CPtr(new rafl::Example<Label>(descriptor, label));
      }
    }

    return examples;
  }
};

}

#endif
