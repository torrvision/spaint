/**
 * grove: ExampleClusterer_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERCUDA
#define H_GROVE_EXAMPLECLUSTERERCUDA

#include "../interface/ExampleClusterer.h"

namespace grove
{

template <typename ExampleType, typename ClusterType>
class ExampleClusterer_CUDA : public ExampleClusterer<ExampleType, ClusterType>
{
public:
  using typename ExampleClusterer<ExampleType, ClusterType>::ClusterBlock;
  using typename ExampleClusterer<ExampleType, ClusterType>::ClusterBlock_Ptr;
  using typename ExampleClusterer<ExampleType, ClusterType>::ClusterBlock_CPtr;

  using typename ExampleClusterer<ExampleType, ClusterType>::ExampleImage;
  using typename ExampleClusterer<ExampleType, ClusterType>::ExampleImage_Ptr;
  using typename ExampleClusterer<ExampleType, ClusterType>::ExampleImage_CPtr;

public:
  ExampleClusterer_CUDA(float sigma, float tau, uint32_t minClusterSize);

  virtual void find_modes(const ExampleImage_CPtr &exampleReservoirs,
                          const ITMIntMemoryBlock_CPtr &keypointReservoirsSize,
                          ClusterBlock_Ptr &predictions,
                          uint32_t startIdx, uint32_t count);
};

}
#endif
