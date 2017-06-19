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

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::reset()
{
  ExampleReservoirs<ExampleType>::reset();
  reinit_rngs();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::accept(const Visitor& visitor)
{
  visitor.visit(*this);
}

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
    reinit_rngs();
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

      add_example_to_reservoirs(
        exampleData[linearIdx], indices, ReservoirIndexCount, reservoirData,
        reservoirSizes, reservoirAddCalls, this->m_reservoirCapacity, rngs[linearIdx]
      );
    }
  }
}

template <typename ExampleType>
void ExampleReservoirs_CPU<ExampleType>::reinit_rngs()
{
  // If the random number generators don't yet exist, create them.
  if(!m_rngs)
  {
    itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();
    m_rngs = mbf.make_block<CPURNG>();
  }

  // Reinitialise each random number generator based on the specified seed.
  CPURNG *rngs = m_rngs->GetData(MEMORYDEVICE_CPU);
  const uint32_t rngCount = static_cast<uint32_t>(m_rngs->dataSize);
  for(uint32_t i = 0; i < rngCount; ++i)
  {
    rngs[i].reset(this->m_rngSeed + i);
  }
}

}
