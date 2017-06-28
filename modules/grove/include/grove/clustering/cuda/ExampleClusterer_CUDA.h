/**
 * grove: ExampleClusterer_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER_CUDA
#define H_GROVE_EXAMPLECLUSTERER_CUDA

#include "../interface/ExampleClusterer.h"

namespace grove {

/**
 * \brief An instance of this class can find clusters from a set of examples using CUDA.
 *
 * See the base class template for additional documentation.
 *
 * \param ExampleType  The type of example to cluster.
 * \param ClusterType  The type of cluster being generated.
 * \param MAX_CLUSTERS The maximum number of clusters being generated for each set of examples.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
class ExampleClusterer_CUDA : public ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>
{
  //#################### USINGS ####################
public:
  using typename ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::Clusters;
  using typename ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::ClustersBlock;
  using typename ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::ClustersBlock_Ptr;
  using typename ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::ExampleImage;
  using typename ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::ExampleImage_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CUDA-based example clusterer.
   *
   * \param sigma            The sigma distance used when estimating the example density.
   * \param tau              The maximum distance there can be between two examples that are part of the same cluster.
   * \param maxClusterCount  The maximum number of clusters retained for each set of examples (all clusters are estimated
   *                         but only the maxClusterCount largest ones are returned). Must be <= MAX_CLUSTERS.
   * \param minClusterSize   The minimum number of elements there must be in a cluster for it to be valid.
   *
   * \throws std::invalid_argument If maxClusterCount > MAX_CLUSTERS.
   */
  ExampleClusterer_CUDA(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void compute_cluster_parameters(const ExampleType *exampleSets, const int *exampleSetSizes, Clusters *clustersData,
                                          uint32_t maxClusterCount, uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void compute_density(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetsCapacity,
                               uint32_t exampleSetsCount, float sigma);

  /** Override */
  virtual Clusters *get_pointer_to_cluster(const ClustersBlock_Ptr& clusters, uint32_t clusterIdx) const;

  /** Override */
  virtual const ExampleType *get_pointer_to_example_set(const ExampleImage_CPtr& exampleSets, uint32_t setIdx) const;

  /** Override */
  virtual const int *get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr& exampleSetSizes, uint32_t setIdx) const;

  /** Override */
  virtual void identify_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void link_neighbors(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetsCapacity,
                              uint32_t exampleSetsCount, float tauSq);

  /** Override */
  virtual void reset_clusters(Clusters *clustersData, uint32_t exampleSetCount) const;

  /** Override */
  virtual void reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t exampleSetCapacity, uint32_t exampleSetCount);
};

}

#endif
