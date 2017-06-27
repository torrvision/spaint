/**
 * grove: ExampleClusterer_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERCUDA
#define H_GROVE_EXAMPLECLUSTERERCUDA

#include "../interface/ExampleClusterer.h"

namespace grove {

/**
 * \brief An instance of this class can find clusters from a set of "examples" using the GPU.
 *        Clustering is performed via the "Really Quick shift" algorithm by Fulkerson and Soatto.
 *        See: http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf for details.
 *
 * \note  The clusterer is capable of clustering multiple sets of examples in parallel,
 *        for this reason the interface to the main clustering method expects not a single set of examples but an
 *        "image" wherein each row contains a certain number examples to be clustered.
 *        Different rows are then clustered independently.
 *
 * \note  The following functions are required to be defined:
 *        - _CPU_AND_GPU_CODE_ inline float distanceSquared(const ExampleType &a, const ExampleType &b)
 *          Returns the squared distancebetween two examples.
 *        - _CPU_AND_GPU_CODE_ inline void createClusterFromExamples(const ExampleType *examples,
 *                                                                   const int *exampleKeys, int examplesCount,
 *                                                                   int key, ClusterType &outputCluster)
 *          Aggregates all the examples in the examples array having a certain key into a single cluster mode.
 *
 * \param ExampleType  The type of examples to cluster.
 * \param ClusterType  The type of clusters being generated.
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
   * \brief Constructs an instance of ExampleClusterer_CUDA.
   *
   * \param sigma            The sigma distance used when estimating the example density.
   * \param tau              The maximum distance that two examples must have to be part of the same cluster.
   * \param maxClusterCount  The maximum number of clusters retained for each set of examples
   *                         (all clusters are estimated but only the maxClusterCount largest ones are returned).
   *                         Must be <= than MAX_CLUSTERS.
   * \param minClusterSize   The minimum number of elements that have to be in a cluster for it to be valid.
   *
   * \throws std::invalid_argument if maxClusterCount is > than MAX_CLUSTERS.
   */
  ExampleClusterer_CUDA(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /** Override */
  virtual void compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void compute_density(const ExampleType *exampleSets, const int *exampleSetsSizes, uint32_t exampleSetsCapacity,
                               uint32_t exampleSetsCount, float sigma);

  /** Override */
  virtual void compute_cluster_parameters(const ExampleType *exampleSets, const int *exampleSetSizes, Clusters *clustersData,
                                          uint32_t maxClusterCount, uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual Clusters *get_pointer_to_cluster(const ClustersBlock_Ptr& clusters, uint32_t clusterIdx) const;

  /** Override */
  virtual const ExampleType *get_pointer_to_example_set(const ExampleImage_CPtr& exampleSets, uint32_t setIdx) const;

  /** Override */
  virtual const int *get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr& exampleSetsSize, uint32_t setIdx) const;

  /** Override */
  virtual void identify_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void link_neighbors(const ExampleType *exampleSets, const int *exampleSetsSizes, uint32_t exampleSetsCapacity,
                              uint32_t exampleSetsCount, float tauSq);

  /** Override */
  virtual void reset_clusters(Clusters *clustersData, uint32_t clustersCount) const;

  /** Override */
  virtual void reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /** Override */
  virtual void select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t exampleSetCapacity, uint32_t exampleSetCount);
};

}

#endif
