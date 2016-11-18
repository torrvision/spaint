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
template<typename ExampleType, typename FeatureType>
class ExampleReservoirs_CUDA: public ExampleReservoirs<ExampleType, FeatureType>
{
public:
  ExampleReservoirs_CUDA(size_t capacity, size_t nbLeaves, uint32_t rngSeed = 42);

  virtual void add_examples(const RGBDPatchFeatureImage_CPtr &features,
      const LeafIndicesImage_CPtr &leafIndices);
  virtual void clear();

protected:
  using ExampleReservoirs<ExampleType, FeatureType>::m_data;
  using ExampleReservoirs<ExampleType, FeatureType>::m_reservoirsSize;
  using ExampleReservoirs<ExampleType, FeatureType>::m_reservoirsAddCalls;

  using ExampleReservoirs<ExampleType, FeatureType>::m_reservoirCapacity;
  using ExampleReservoirs<ExampleType, FeatureType>::m_rngSeed;

private:
  tvgutil::CUDARNGMemoryBlock_Ptr m_randomStates;

  void init_random();
};
}

#endif
