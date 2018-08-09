/**
 * grove: ExampleClustererFactory.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleClustererFactory.h"

#include "cpu/ExampleClusterer_CPU.h"

#ifdef WITH_CUDA
#include "cuda/ExampleClusterer_CUDA.h"
#endif

namespace grove {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MaxClusters>
typename ExampleClustererFactory<ExampleType,ClusterType,MaxClusters>::Clusterer_Ptr
ExampleClustererFactory<ExampleType,ClusterType,MaxClusters>::make_clusterer(
  float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize, DeviceType deviceType
)
{
  Clusterer_Ptr clusterer;

  if(deviceType == DEVICE_CUDA)
  {
#ifdef WITH_CUDA
    clusterer.reset(new ExampleClusterer_CUDA<ExampleType,ClusterType,MaxClusters>(sigma, tau, maxClusterCount, minClusterSize));
#else
    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
  }
  else
  {
    clusterer.reset(new ExampleClusterer_CPU<ExampleType,ClusterType,MaxClusters>(sigma, tau, maxClusterCount, minClusterSize));
  }

  return clusterer;
}

}
