/**
 * grove: ExampleReservoirs_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSSHARED
#define H_GROVE_EXAMPLERESERVOIRSSHARED

#include <ORUtils/PlatformIndependence.h>

#define ALWAYS_ADD_EXAMPLES 0

namespace grove {

/**
 * \brief Add an example to some reservoirs.
 *
 * If the example is valid, an attempt to add it to each reservoir specified by indices is made.
 * If a reservoir is not full, then the example is added.
 * Otherwise, if ALWAYS_ADD_EXAMPLES is set, a randomly-selected existing example is discarded and replaced by the current example.
 * If ALWAYS_ADD_EXAMPLES is 0 then whether to replace an existing example is randomly decided as well.
 *
 * \param example            The example to add to the reservoirs.
 * \param indices            Pointer to the indices to the reservoir wherein to add the example.
 * \param indicesCount       Number of elements in the indices array.
 * \param randomGenerator    A random number generator.
 * \param reservoirs         A pointer to the reservoir data.
 * \param reservoirSize      A pointer to the current sizes of each reservoir.
 * \param reservoirAddCalls  A pointer to the number of times an add operation has been attempted for each reservoir.
 * \param reservoirCapacity  The maximum size of each reservoir.
 */
template <typename ExampleType, typename RNGType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_reservoirs_add_example(const ExampleType &example,
    const int *indices, uint32_t indicesCount, RNGType &randomGenerator, ExampleType *reservoirs,
    int *reservoirSize, int *reservoirAddCalls, uint32_t reservoirCapacity)
{
  // Early out if the example is invalid.
  if (!example.valid) return;

  // Try to add the example to each reservoir in indices.
  for (uint32_t i = 0; i < indicesCount; ++i)
  {
    // The reservoir index, representing its row in the reservoirs array.
    const int reservoirIdx = indices[i];

    // The linearised index of the first example of the reservoir.
    const int reservoirStartIdx = reservoirIdx * reservoirCapacity;

    // Get the number of total add calls for the current reservoir.
    uint32_t addCallsCount = 0;

#ifdef __CUDACC__
    addCallsCount = atomicAdd(&reservoirAddCalls[reservoirIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
    addCallsCount = reservoirAddCalls[reservoirIdx]++;
#endif

    // If addCallsCount is less than capacity we can add the example straight-away.
    if (addCallsCount < reservoirCapacity)
    {
      reservoirs[reservoirStartIdx + addCallsCount] = example;

      // Also increment the size of the current reservoir.
      // Might not be strictly necessary, since reservoirAddCalls also has the same info and we could
      // return that in get_reservoirs_size(), but then care would need to be taken when examining values
      // outside this class since they will be greater than the actual reservoir capacity.
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
#if ALWAYS_ADD_EXAMPLES
      // The code below always evicts a sample from the reservoir.
      const uint32_t randomIdx = randomGenerator.generate_int_from_uniform(0,
          reservoirCapacity);
#else
      // Generate a random number between 0 and addCallsCount.
      const uint32_t randomIdx = randomGenerator.generate_int_from_uniform(0,
          addCallsCount);
#endif

      // Check if we have to evict an example.
      if (randomIdx < reservoirCapacity)
      {
        reservoirs[reservoirStartIdx + randomIdx] = example;
      }
    }
  }
}

}

#endif
