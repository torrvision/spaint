/**
 * grove: ExampleClusterer_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER_CPU
#define H_GROVE_EXAMPLECLUSTERER_CPU

#include "../interface/ExampleClusterer.h"

namespace grove {

/**
 * \brief An instance of this class can find clusters from a set of examples using the CPU.
 *
 * See the base class template for additional documentation.
 *
 * \param ExampleType  The type of example to cluster.
 * \param ClusterType  The type of cluster being generated.
 * \param MAX_CLUSTERS The maximum number of clusters being generated for each set of examples.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
class ExampleClusterer_CPU : public ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>
{
  //#################### TYPEDEFS AND USINGS ####################
public:
  typedef ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS> Base;
  using typename Base::Clusters;
  using typename Base::ClustersBlock;
  using typename Base::ClustersBlock_Ptr;
  using typename Base::ExampleImage;
  using typename Base::ExampleImage_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a CPU-based example clusterer.
   *
   * \param sigma            The sigma of the Gaussian used when computing the example densities.
   * \param tau              The maximum distance there can be between two examples that are part of the same cluster.
   * \param maxClusterCount  The maximum number of clusters retained for each set of examples (all clusters are estimated
   *                         but only the maxClusterCount largest ones are returned). Must be <= MAX_CLUSTERS.
   * \param minClusterSize   The minimum size of cluster to keep.
   *
   * \throws std::invalid_argument If maxClusterCount > MAX_CLUSTERS.
   */
  ExampleClusterer_CPU(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual void compute_cluster_indices(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void compute_densities(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetCapacity,
                                 uint32_t exampleSetCount, float sigma);

  /** Override */
  virtual void compute_parents(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetCapacity,
                               uint32_t exampleSetCount, float tauSq);

  /** Override */
  virtual void create_selected_clusters(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t maxSelectedClusters,
                                        uint32_t exampleSetCapacity, uint32_t exampleSetCount, Clusters *clustersData);

  /** Override */
  virtual Clusters *get_pointer_to_cluster(const ClustersBlock_Ptr& clusters, uint32_t clusterIdx) const;

  /** Override */
  virtual const ExampleType *get_pointer_to_example_set(const ExampleImage_CPtr& exampleSets, uint32_t setIdx) const;

  /** Override */
  virtual const int *get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr& exampleSetSizes, uint32_t setIdx) const;

  /** Override */
  virtual void reset_clusters(Clusters *clustersData, uint32_t exampleSetCount) const;

  /** Override */
  virtual void reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void select_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount);
};

}

#endif
