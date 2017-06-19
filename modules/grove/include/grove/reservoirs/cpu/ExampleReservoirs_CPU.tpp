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
  reset();
}

//#################### PUBLIC VIRTUAL MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::reset()
{
  ExampleReservoirs<ExampleType>::reset();
  init_random();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType>
template <int ReservoirIndexCount>
void ExampleReservoirs_CPU<ExampleType>::add_examples_sub(const ExampleImage_CPtr& examples, const boost::shared_ptr<const ORUtils::Image<ORUtils::VectorX<int,ReservoirIndexCount> > >& reservoirIndices)
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
  const ORUtils::VectorX<int,ReservoirIndexCount> *reservoirIndicesPtr = reservoirIndices->GetData(MEMORYDEVICE_CPU);
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
      const int *indices = reservoirIndicesPtr[linearIdx].v;

      example_reservoirs_add_example(
        exampleData[linearIdx], indices, ReservoirIndexCount, rngs[linearIdx], reservoirData,
        reservoirSizes, reservoirAddCalls, this->m_reservoirCapacity
      );
    }
  }
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::init_random()
{
  if(!m_rngs)
  {
    itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();
    m_rngs = mbf.make_block<CPURNG>();
  }

  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);
  const int rngCount = static_cast<int>(m_rngs->dataSize);

  for (int i = 0; i < rngCount; ++i)
  {
    rngs[i].reset(this->m_rngSeed + i);
  }
}

}
