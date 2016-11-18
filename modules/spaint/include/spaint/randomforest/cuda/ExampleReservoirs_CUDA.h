/**
 * spaint: ExampleReservoirs_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_EXAMPLERESERVOIRSCUDA
#define H_SPAINT_EXAMPLERESERVOIRSCUDA

#include "../interface/ExampleReservoirs.h"

#include "tvgutil/numbers/SimpleRandomNumberGenerator_CUDA.h"

namespace spaint
{
template<typename ExampleType, typename FeatureType, typename LeafType>
class ExampleReservoirs_CUDA: public ExampleReservoirs<ExampleType, FeatureType,
    LeafType>
{
public:
  using typename ExampleReservoirs<ExampleType, FeatureType, LeafType>::FeatureImage_CPtr;
  using typename ExampleReservoirs<ExampleType, FeatureType, LeafType>::LeafImage_CPtr;

  ExampleReservoirs_CUDA(size_t capacity, size_t nbLeaves,
      uint32_t rngSeed = 42);

  virtual void add_examples(const FeatureImage_CPtr &features,
      const LeafImage_CPtr &leafIndices);
  virtual void clear();

protected:
  using ExampleReservoirs<ExampleType, FeatureType, LeafType>::m_data;
  using ExampleReservoirs<ExampleType, FeatureType, LeafType>::m_reservoirsSize;
  using ExampleReservoirs<ExampleType, FeatureType, LeafType>::m_reservoirsAddCalls;

  using ExampleReservoirs<ExampleType, FeatureType, LeafType>::m_reservoirCapacity;
  using ExampleReservoirs<ExampleType, FeatureType, LeafType>::m_rngSeed;

private:
  tvgutil::CUDARNGMemoryBlock_Ptr m_randomStates;

  void init_random();
};
}

#endif
