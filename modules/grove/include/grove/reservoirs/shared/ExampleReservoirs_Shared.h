/**
 * grove: ExampleReservoirs_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRS_SHARED
#define H_GROVE_EXAMPLERESERVOIRS_SHARED

#include <ORUtils/PlatformIndependence.h>

#define ALWAYS_ADD_EXAMPLES 0

namespace grove {

/**
 * \brief Attempts to add an example to some reservoirs.
 *
 * If the example is valid, we attempt to add it to each specified reservoir. If a
 * reservoir is not full, then the example is added. Otherwise, if ALWAYS_ADD_EXAMPLES
 * is 1, a randomly-selected existing example is discarded and replaced by the current
 * example. If ALWAYS_ADD_EXAMPLES is 0, then an additional random decision is made as
 * to *whether* to replace an existing example.
 *
 * \param example             The example to attempt to add to the reservoirs.
 * \param reservoirIndices    The indices of the reservoirs to which to attempt to add the example.
 * \param reservoirIndexCount The number of reservoirs to which to attempt to add the example.
 * \param reservoirs          The example reservoirs: an image in which each row allows the storage of up to reservoirCapacity examples.
 * \param reservoirSizes      The current size of each reservoir.
 * \param reservoirAddCalls   The number of times the insertion of an example has been attempted for each reservoir.
 * \param reservoirCapacity   The capacity (maximum size) of each reservoir.
 * \param randomGenerator     A random number generator.
 */
template <typename ExampleType, typename RNGType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void add_example_to_reservoirs(const ExampleType& example, const int *reservoirIndices, uint32_t reservoirIndexCount,
                                      ExampleType *reservoirs, int *reservoirSizes, int *reservoirAddCalls, uint32_t reservoirCapacity,
                                      RNGType& randomGenerator)
{
  // If the example is invalid, early out.
  if(!example.valid) return;

  // Try to add the example to each specified reservoir.
  for(uint32_t i = 0; i < reservoirIndexCount; ++i)
  {
    // The reservoir index (this corresponds to a row in the reservoirs image).
    const int reservoirIdx = reservoirIndices[i];

    // The raster index (in the reservoirs image) of the first example in the reservoir.
    const int reservoirStartIdx = reservoirIdx * reservoirCapacity;

    // Get the total number of add calls that have ever been made for the current reservoir, and increment it for next time.
    uint32_t oldAddCallsCount = 0;

#ifdef __CUDACC__
    oldAddCallsCount = atomicAdd(&reservoirAddCalls[reservoirIdx], 1);
#else
  #ifdef WITH_OPENMP
    #pragma omp atomic capture
  #endif
    oldAddCallsCount = reservoirAddCalls[reservoirIdx]++;
#endif

    // If the old total number of add calls is less than the reservoir's capacity, then we can immediately add the example.
    // Otherwise, we need to decide whether or not to replace an existing example with this one.
    if(oldAddCallsCount < reservoirCapacity)
    {
      // Store the example in the reservoir.
      reservoirs[reservoirStartIdx + oldAddCallsCount] = example;

      // Increment the reservoir's size. Note that it is not strictly necessary to
      // maintain the reservoir sizes separately, since we can obtain the same
      // information from reservoirAddCalls by clamping the values to the reservoir
      // capacity, but writing it this way is much clearer and the cost in efficiency
      // is limited in practice.
#ifdef __CUDACC__
      atomicAdd(&reservoirSizes[reservoirIdx], 1);
#else
    #ifdef WITH_OPENMP
      #pragma omp atomic
    #endif
      ++reservoirSizes[reservoirIdx];
#endif
    }
    else
    {
#if ALWAYS_ADD_EXAMPLES
      // Generate a random offset that will always result in an example being evicted from the reservoir.
      const uint32_t randomOffset = randomGenerator.generate_int_from_uniform(0, reservoirCapacity - 1);
#else
      // Generate a random offset that may or may not result in an example being evicted from the reservoir.
      const uint32_t randomOffset = randomGenerator.generate_int_from_uniform(0, oldAddCallsCount - 1);
#endif

      // If the random offset corresponds to an example in the reservoir, replace that with the new example.
      if(randomOffset < reservoirCapacity)
      {
        reservoirs[reservoirStartIdx + randomOffset] = example;
      }
    }
  }
}

}

#endif
