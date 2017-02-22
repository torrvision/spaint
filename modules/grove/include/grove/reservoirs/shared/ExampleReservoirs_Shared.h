/**
 * grove: ExampleReservoirs_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSSHARED
#define H_GROVE_EXAMPLERESERVOIRSSHARED

#include <ORUtils/PlatformIndependence.h>

namespace grove {

template <typename ExampleType, typename IndexType, typename RNGType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_reservoirs_add_example(const ExampleType &example,
    const IndexType &indices, RNGType &randomGenerator, ExampleType *reservoirs,
    int *reservoirSize, int *reservoirAddCalls, uint32_t reservoirCapacity)
{
  if (!example.valid) return;

  for (int i = 0; i < indices.size(); ++i)
  {
    const int reservoirIdx = indices[i]; // The row in the reservoirs array
    const int reservoirStartIdx = reservoirIdx * reservoirCapacity;

    // Get the number of total add calls
    uint32_t addCallsCount = 0;

#ifdef __CUDACC__
    addCallsCount = atomicAdd(&reservoirAddCalls[reservoirIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
    addCallsCount = reservoirAddCalls[reservoirIdx]++;
#endif

    // if addCallsCount is less than capacity we can add the example straight-away
    if (addCallsCount < reservoirCapacity)
    {
      reservoirs[reservoirStartIdx + addCallsCount] = example;
      // Also increment the size of the current reservoir
      // might not be needed, reservoirAddCalls also has the same info but care needs
      // to be taken when reading values outside this class since they will be greater than the reservoir capacity

#ifdef __CUDACC__
      atomicAdd(&reservoirSize[reservoirIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic
#endif
      reservoirSize[reservoirIdx]++;
#endif
    }
    else
    {
#if 1
      // Generate a random index and check if we have to evict an example
      const uint32_t randomIdx = randomGenerator.generate_int_from_uniform(0,
          addCallsCount);
#else
      // The code below always evicts a sample.
      const uint32_t randomIdx = randomGenerator.generate_int_from_uniform(0,
          reservoirCapacity);
#endif

      if (randomIdx < reservoirCapacity)
      {
        reservoirs[reservoirStartIdx + randomIdx] = example;
      }
    }
  }
}

}

#endif
