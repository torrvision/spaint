/**
 * grove: ExampleReservoirs_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleReservoirs_CPU.h"

#include <itmx/base/MemoryBlockFactory.h>

#include "../shared/ExampleReservoirs_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType>
ExampleReservoirs_CPU<ExampleType>::ExampleReservoirs_CPU(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed)
: ExampleReservoirs<ExampleType>(reservoirCapacity, reservoirCount, rngSeed)
{
  itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();

  // Initialise the random number generators.
  m_rngs = mbf.make_block<CPURNG>();
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
void ExampleReservoirs_CPU<ExampleType>::add_examples(const ExampleImage_CPtr& examples, const char *reservoirIndicesCPU, const char *reservoirIndicesCUDA,
                                                      uint32_t reservoirIndicesCount, uint32_t reservoirIndicesStep)
{
  const Vector2i imgSize = examples->noDims;
  const size_t exampleCount = imgSize.width * imgSize.height;

  // Check that we have enough random number generators and reallocate them if not.
  if(m_rngs->dataSize < exampleCount)
  {
    m_rngs->Resize(exampleCount);
    init_random();
  }

  const ExampleType *exampleData = examples->GetData(MEMORYDEVICE_CPU);
  int *reservoirAddCalls = this->m_reservoirAddCalls->GetData(MEMORYDEVICE_CPU);
  ExampleType *reservoirData = this->m_reservoirs->GetData(MEMORYDEVICE_CPU);
  int *reservoirSizes = this->m_reservoirSizes->GetData(MEMORYDEVICE_CPU);
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.height; ++y)
  {
    for (int x = 0; x < imgSize.width; ++x)
    {
      const int linearIdx = y * imgSize.x + x;
      const int *indices = reinterpret_cast<const int*>(reservoirIndicesCPU + linearIdx * reservoirIndicesStep);

      example_reservoirs_add_example(
        exampleData[linearIdx], indices, reservoirIndicesStep, rngs[linearIdx], reservoirData,
        reservoirSizes, reservoirAddCalls, this->m_reservoirCapacity
      );
    }
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::init_random()
{
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);
  const int rngCount = static_cast<int>(m_rngs->dataSize);

  for (int i = 0; i < rngCount; ++i)
  {
    rngs[i].reset(this->m_rngSeed + i);
  }
}

}
