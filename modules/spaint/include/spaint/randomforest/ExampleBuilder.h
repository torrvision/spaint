/**
 * spaint: ExampleBuilder.h
 */

#ifndef H_SPAINT_EXAMPLEBUILDER
#define H_SPAINT_EXAMPLEBUILDER

#include <ORUtils/MemoryBlock.h>

#include <rafl/examples/Example.h>

namespace spaint {

/**
 * \brief TODO
 */
template <typename Label>
struct ExampleBuilder
{
  //#################### TYPEDEFS ####################
private:
  typedef boost::shared_ptr<Example<Label> > Example_Ptr;
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  static std::vector<Example_CPtr> make_examples(const ORUtils::MemoryBlock<float>& featuresMB, const ORUtils::MemoryBlock<unsigned int>& sampledVoxelCountsMB, 
                                                 size_t featureCount, size_t maxVoxelsPerLabel, size_t maxLabelCount)
  {
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
        Descriptor_Ptr descriptor(new Descriptor(featuresForExample, featuresForExample + featureCount));

        // Add the example.
        examples[exampleIndex++] = Example_CPtr(new Example<Label>(descriptor, label));
      }
    }

    return examples;
  }
};

}

#endif
