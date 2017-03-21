/**
 * grove: ExampleReservoirs_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs_CPU.h"

#include <tvgutil/itm/MemoryBlockFactory.h>
using namespace tvgutil;

#include "../shared/ExampleReservoirs_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType>
ExampleReservoirs_CPU<ExampleType>::ExampleReservoirs_CPU(
    uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed) :
    ExampleReservoirs<ExampleType>(reservoirCapacity, reservoirCount, rngSeed)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  // Initialise the random number generators.
  m_randomStates = mbf.make_block<CPURNG>();
  init_random();
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::clear()
{
  ExampleReservoirs<ExampleType>::clear();
  init_random();
}

//#################### PROTECTED VIRTUAL MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::add_examples(const ExampleImage_CPtr &examples,
    const char *reservoirIndicesCPU, const char *reservoirIndicesCUDA, uint32_t reservoirIndicesCount,
    uint32_t reservoirIndicesStep)
{
  const Vector2i imgSize = examples->noDims;
  const size_t nbExamples = imgSize.width * imgSize.height;

  // Check that we have enough random states and, if not, reallocate them.
  if (nbExamples > m_randomStates->dataSize)
  {
    m_randomStates->ChangeDims(nbExamples);
    init_random();
  }

  const ExampleType *exampleData = examples->GetData(MEMORYDEVICE_CPU);

  CPURNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CPU);
  int *reservoirAddCalls = this->m_reservoirsAddCalls->GetData(MEMORYDEVICE_CPU);
  ExampleType *reservoirData = this->m_data->GetData(MEMORYDEVICE_CPU);
  int *reservoirSize = this->m_reservoirsSize->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.height; ++y)
  {
    for (int x = 0; x < imgSize.width; ++x)
    {
      const int linearIdx = y * imgSize.x + x;
      const int linearIndicesIdx = (y * imgSize.x + x) * reservoirIndicesStep;
      const int* indices = reinterpret_cast<const int*>(reservoirIndicesCPU + linearIndicesIdx);

      example_reservoirs_add_example(exampleData[linearIdx], indices,
          reservoirIndicesStep, randomStates[linearIdx], reservoirData,
          reservoirSize, reservoirAddCalls, this->m_capacity);
    }
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::init_random()
{
  const size_t nbStates = m_randomStates->dataSize;
  CPURNG *randomStates = m_randomStates->GetData(MEMORYDEVICE_CPU);

  for (size_t stateIdx = 0; stateIdx < nbStates; ++stateIdx)
  {
    randomStates[stateIdx].reset(this->m_rngSeed + stateIdx);
  }
}

}
