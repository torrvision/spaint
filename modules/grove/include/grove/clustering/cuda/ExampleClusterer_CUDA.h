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
  ExampleClusterer_CUDA(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

protected:
  virtual const ExampleType* get_pointer_to_example_set(const ExampleImage_CPtr &exampleReservoirs, uint32_t reservoirIdx) const;
  virtual const int* get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr &exampleReservoirsSize, uint32_t reservoirIdx) const;
  virtual ClusterType* get_pointer_to_prediction(const ClusterBlock_Ptr &predictions, uint32_t predictionIdx) const;

  virtual void reset_temporaries(uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void reset_predictions(ClusterType *predictionsData, uint32_t reservoirCount) const;
  virtual void compute_density(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float sigma);
  virtual void link_neighbors(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float tauSq);
  virtual void identify_clusters(uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void compute_cluster_size_histograms(uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void compute_modes(const ExampleType *examples, const int *reservoirSizes, ClusterType *predictionsData, uint32_t maxClusterCount, uint32_t reservoirCapacity, uint32_t reservoirCount);
};

}
#endif
