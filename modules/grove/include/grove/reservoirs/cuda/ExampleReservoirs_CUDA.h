/**
 * grove: ExampleReservoirs_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLERESERVOIRSCUDA
#define H_GROVE_EXAMPLERESERVOIRSCUDA

#include "../interface/ExampleReservoirs.h"

#include <tvgutil/numbers/SimpleRandomNumberGenerator_CUDA.h>

namespace grove
{
template <typename ExampleType, typename IndexType>
class ExampleReservoirs_CUDA : public ExampleReservoirs<ExampleType, IndexType>
{
public:
  using typename ExampleReservoirs<ExampleType, IndexType>::ExampleImage_CPtr;
  using typename ExampleReservoirs<ExampleType, IndexType>::IndexImage_CPtr;

  ExampleReservoirs_CUDA(uint32_t reservoirCapacity, uint32_t reservoirCount, uint32_t rngSeed = 42);

  virtual void add_examples(const ExampleImage_CPtr &examples,
      const IndexImage_CPtr &reservoirIndices);
  virtual void clear();

private:
  tvgutil::CUDARNGMemoryBlock_Ptr m_randomStates;

  void init_random();
};
}

#endif
