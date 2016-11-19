/**
 * spaint: ExampleReservoirs_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_EXAMPLERESERVOIRSCPU
#define H_SPAINT_EXAMPLERESERVOIRSCPU

#include "../interface/ExampleReservoirs.h"

#include "tvgutil/numbers/SimpleRandomNumberGenerator_CPU.h"

namespace spaint
{
template<typename ExampleType, typename FeatureType, typename LeafType>
class ExampleReservoirs_CPU: public ExampleReservoirs<ExampleType, FeatureType,
    LeafType>
{
public:
  using typename ExampleReservoirs<ExampleType, FeatureType, LeafType>::FeatureImage_CPtr;
  using typename ExampleReservoirs<ExampleType, FeatureType, LeafType>::LeafImage_CPtr;

  ExampleReservoirs_CPU(size_t capacity, size_t nbLeaves,
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
  tvgutil::CPURNGMemoryBlock_Ptr m_randomStates;

  void init_random();
};
}

#endif
