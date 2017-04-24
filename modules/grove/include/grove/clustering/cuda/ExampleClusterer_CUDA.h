/**
 * grove: ExampleClusterer_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERCUDA
#define H_GROVE_EXAMPLECLUSTERERCUDA

#include "../interface/ExampleClusterer.h"

namespace grove
{

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS = 10>
class ExampleClusterer_CUDA : public ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>
{
public:
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::Clusters;

  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ClustersBlock;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ClustersBlock_Ptr;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ClustersBlock_CPtr;

  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleImage;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleImage_Ptr;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleImage_CPtr;

public:
  ExampleClusterer_CUDA(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

protected:
  virtual const ExampleType* get_pointer_to_example_set(const ExampleImage_CPtr &exampleReservoirs, uint32_t reservoirIdx) const;
  virtual const int* get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr &exampleReservoirsSize, uint32_t reservoirIdx) const;
  virtual Clusters* get_pointer_to_cluster(const ClustersBlock_Ptr &clusters, uint32_t clusterIdx) const;

  virtual void reset_temporaries(uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void reset_clusters(Clusters *clustersData, uint32_t clustersCount) const;
  virtual void compute_density(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float sigma);
  virtual void link_neighbors(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float tauSq);
  virtual void identify_clusters(uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void compute_cluster_size_histograms(uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t reservoirCapacity, uint32_t reservoirCount);
  virtual void compute_cluster_parameters(const ExampleType *examples, const int *reservoirSizes, Clusters *clustersData, uint32_t maxClusterCount, uint32_t reservoirCapacity, uint32_t reservoirCount);
};

}
#endif
