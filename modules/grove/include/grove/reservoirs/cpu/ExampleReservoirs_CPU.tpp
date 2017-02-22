/**
 * grove: ExampleReservoirs_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs_CPU.h"

#include <spaint/util/MemoryBlockFactory.h>

#include "../shared/ExampleReservoirs_Shared.h"

using namespace spaint;
using namespace tvgutil;

namespace grove {

template <typename ExampleType, typename IndexType>
ExampleReservoirs_CPU<ExampleType, IndexType>::ExampleReservoirs_CPU(
    uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed) :
    ExampleReservoirs<ExampleType, IndexType>(reservoirCapacity, reservoirCount, rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();
  m_randomStates = mbf.make_block<CPURNG>();
  init_random();
}

template <typename ExampleType, typename IndexType>
void ExampleReservoirs_CPU<ExampleType, IndexType>::add_examples(const ExampleImage_CPtr &examples,
    const IndexImage_CPtr &reservoirIndices)
{
  if(examples->noDims != reservoirIndices->noDims)
    throw std::invalid_argument("The example and indices images should have the same size.");

  const Vector2i imgSize = examples->noDims;
  const size_t nbExamples = imgSize.width * imgSize.height;

  // Check that we have enough random states and if not reallocate them
  if (nbExamples > m_randomStates->dataSize)
  {
    m_randomStates->ChangeDims(nbExamples);
    init_random();
  }

  const ExampleType *exampleData = examples->GetData(MEMORYDEVICE_CPU);
  const IndexType *indicesData = reservoirIndices->GetData(MEMORYDEVICE_CPU);

  CPURNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CPU);
  ExampleType *reservoirData = this->m_data->GetData(MEMORYDEVICE_CPU);
  int *reservoirSize = this->m_reservoirsSize->GetData(MEMORYDEVICE_CPU);
  int *reservoirAddCalls = this->m_reservoirsAddCalls->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.height; ++y)
  {
    for (int x = 0; x < imgSize.width; ++x)
    {
      const int linearIdx = y * imgSize.x + x;

      example_reservoirs_add_example(exampleData[linearIdx],
          indicesData[linearIdx], randomStates[linearIdx], reservoirData,
          reservoirSize, reservoirAddCalls, this->m_capacity);
    }
  }
}

template <typename ExampleType, typename IndexType>
void ExampleReservoirs_CPU<ExampleType, IndexType>::clear()
{
  ExampleReservoirs<ExampleType, IndexType>::clear();
  init_random();
}

template <typename ExampleType, typename IndexType>
void ExampleReservoirs_CPU<ExampleType, IndexType>::init_random()
{
  const size_t nbStates = m_randomStates->dataSize;
  CPURNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CPU);

  for (size_t stateIdx = 0; stateIdx < nbStates; ++stateIdx)
  {
    randomStates[stateIdx].reset(this->m_rngSeed + stateIdx);
  }
}

}
