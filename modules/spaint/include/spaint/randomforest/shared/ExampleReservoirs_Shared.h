/**
 * spaint: ExampleReservoirs_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_EXAMPLERESERVOIRSSHARED
#define H_SPAINT_EXAMPLERESERVOIRSSHARED

#include <ORUtils/PlatformIndependence.h>

namespace spaint
{

// A function that will be used to convert a feature in a suitable example to be added into the reservoir
template<typename ExampleType, typename FeatureType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline ExampleType make_example_from_feature(const FeatureType &feature);

template<typename ExampleType, typename FeatureType, typename LeafType,
    typename RNGType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_reservoirs_add_example(const FeatureType &feature,
    const LeafType &leaves, RNGType &randomGenerator, ExampleType *reservoirs,
    int *reservoirSize, int *reservoirAddCalls, uint32_t reservoirCapacity)
{
  if (!feature.valid)
    return;

  ExampleType example = make_example_from_feature<ExampleType>(feature);

  for (int treeIdx = 0; treeIdx < leaves.size(); ++treeIdx)
  {
    const int leafIdx = leaves[treeIdx]; // The row in the reservoirs array
    const int reservoirStartIdx = leafIdx * reservoirCapacity;

    // Get the number of total add calls
    uint32_t addCallsCount = 0;

#ifdef __CUDACC__
    addCallsCount = atomicAdd(&reservoirAddCalls[leafIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
    addCallsCount = reservoirAddCalls[leafIdx]++;
#endif

    // if addCallsCount is less than capacity we can add the example straightaway
    if (addCallsCount < reservoirCapacity)
    {
      reservoirs[reservoirStartIdx + addCallsCount] = example;
      // Also increment the size of the current reservoir
      // might not be needed, reservoirAddCalls also has the same info but care needs
      // to be taken when reading values since they will be greater than the reservoir capacity

#ifdef __CUDACC__
      atomicAdd(&reservoirSize[leafIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic
#endif
      reservoirSize[leafIdx]++;
#endif
    }
    else
    {
      // Generate a random index and check if we have to evict an example
      const uint32_t randomIdx = randomGenerator.generate_int_from_uniform(0,
          addCallsCount);
      // The code below always evicts a sample.
//      const uint32_t randomIdx = randomGenerator.generate_int_from_uniform(0,
//          reservoirCapacity);

      if (randomIdx < reservoirCapacity)
      {
        reservoirs[reservoirStartIdx + randomIdx] = example;
      }
    }
  }
}

}

#endif
