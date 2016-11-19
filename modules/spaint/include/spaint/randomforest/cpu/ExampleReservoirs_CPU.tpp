/**
 * spaint: ExampleReservoirs_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_EXAMPLERESERVOIRSCPUTPP
#define H_SPAINT_EXAMPLERESERVOIRSCPUTPP

#include "randomforest/cpu/ExampleReservoirs_CPU.h"
#include "randomforest/shared/ExampleReservoirs_Shared.h"
#include "util/MemoryBlockFactory.h"

using namespace tvgutil;

namespace spaint
{
template<typename ExampleType, typename FeatureType, typename LeafType>
ExampleReservoirs_CPU<ExampleType, FeatureType, LeafType>::ExampleReservoirs_CPU(
    size_t capacity, size_t nbLeaves, uint32_t rngSeed) :
    ExampleReservoirs<ExampleType, FeatureType, LeafType>(capacity, nbLeaves,
        rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomStates = mbf.make_block<CPURNG>();
  init_random();
}

template<typename ExampleType, typename FeatureType, typename LeafType>
void ExampleReservoirs_CPU<ExampleType, FeatureType, LeafType>::add_examples(
    const FeatureImage_CPtr &features, const LeafImage_CPtr &leafIndices)
{
  const Vector2i imgSize = features->noDims;
  const size_t nbExamples = imgSize.width * imgSize.height;

  // Check that we have enough random states and if not reallocate them
  if (nbExamples > m_randomStates->dataSize)
  {
    m_randomStates->ChangeDims(nbExamples);
    init_random();
  }

  const FeatureType *featureData = features->GetData(MEMORYDEVICE_CPU);
  const LeafType *leafIndicesData = leafIndices->GetData(MEMORYDEVICE_CPU);

  CPURNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CPU);
  ExampleType *reservoirData = m_data->GetData(MEMORYDEVICE_CPU);
  int *reservoirSize = m_reservoirsSize->GetData(MEMORYDEVICE_CPU);
  int *reservoirAddCalls = m_reservoirsAddCalls->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.height; ++y)
  {
    for (int x = 0; x < imgSize.width; ++x)
    {
      const int linearIdx = y * imgSize.x + x;

      example_reservoirs_add_example(featureData[linearIdx],
          leafIndicesData[linearIdx], randomStates[linearIdx], reservoirData,
          reservoirSize, reservoirAddCalls, m_reservoirCapacity);
    }
  }
}

template<typename ExampleType, typename FeatureType, typename LeafType>
void ExampleReservoirs_CPU<ExampleType, FeatureType, LeafType>::clear()
{
  ExampleReservoirs<ExampleType, FeatureType, LeafType>::clear();
  init_random();
}

template<typename ExampleType, typename FeatureType, typename LeafType>
void ExampleReservoirs_CPU<ExampleType, FeatureType, LeafType>::init_random()
{
  const size_t nbStates = m_randomStates->dataSize;
  CPURNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CPU);

  for (size_t stateIdx = 0; stateIdx < nbStates; ++stateIdx)
  {
    randomStates[stateIdx].reset(m_rngSeed + stateIdx);
  }
}

}

#endif
