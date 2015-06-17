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
   * \brief Makes rafl examples from feature descriptors that are stored implicitly and contiguously in an InfiniTAM memory block.
   *
   * The memory block contains feature descriptors that are grouped by the label that should be assigned to them. In particular,
   * the block is divided into equally-sized segments, each of which contains maxDescriptorsPerLabel feature descriptors. Within
   * segment i, the first descriptorCounts[i] (<= maxDescriptorsPerLabel) feature descriptors are valid and can be used to make
   * examples. Each feature descriptor in segment i is assigned label i when making examples.
   *
   * \param featuresMB              The InfiniTAM memory block containing the feature descriptors.
   * \param descriptorCountsMB      An InfiniTAM memory block containing the numbers of descriptors in each label segment that are valid.
   * \param featureCount            The number of features in a feature descriptor.
   * \param maxDescriptorsPerLabel  The number of descriptors that could potentially be stored in a label segment.
   * \param labelCount              The number of labels for which the memory block contains descriptors.
   * \return                        The rafl examples.
   */
  template <typename Label>
  static std::vector<boost::shared_ptr<const rafl::Example<Label> > > make_examples(const ORUtils::MemoryBlock<float>& featuresMB,
                                                                                    const ORUtils::MemoryBlock<unsigned int>& descriptorCountsMB, 
                                                                                    size_t featureCount, size_t maxDescriptorsPerLabel, size_t labelCount)
  {
    typedef boost::shared_ptr<const rafl::Example<Label> > Example_CPtr;

    // Determine the number of examples we are trying to make (one per valid descriptor).
    descriptorCountsMB.UpdateHostFromDevice();
    const unsigned int *descriptorCounts = descriptorCountsMB.GetData(MEMORYDEVICE_CPU);
    size_t exampleCount = 0;
    for(size_t i = 0, size = descriptorCountsMB.dataSize; i < size; ++i)
    {
      exampleCount += descriptorCounts[i];
    }

    // Make the examples.
    featuresMB.UpdateHostFromDevice();
    const float *features = featuresMB.GetData(MEMORYDEVICE_CPU);
    std::vector<Example_CPtr> examples(exampleCount);
    size_t exampleIndex = 0;
    for(Label label = 0; label < static_cast<Label>(labelCount); ++label)
    {
      for(size_t i = 0; i < descriptorCounts[label]; ++i)
      {
        // Copy the features for the example into a descriptor.
        const float *featuresForExample = features + (label * maxDescriptorsPerLabel + i) * featureCount;
        rafl::Descriptor_Ptr descriptor(new rafl::Descriptor(featuresForExample, featuresForExample + featureCount));

        // Make the example and add it.
        examples[exampleIndex++] = Example_CPtr(new rafl::Example<Label>(descriptor, label));
      }
    }

    return examples;
  }
};

}

#endif
