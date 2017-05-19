/**
 * grove: ExampleClusterer_CPU.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERCPU
#define H_GROVE_EXAMPLECLUSTERERCPU

#include "../interface/ExampleClusterer.h"

namespace grove {

/**
 * \brief An instance of this class can find clusters from a set of "examples" using the CPU.
 *        Clustering is performed via the "Really Quick shift" algorithm by Fulkerson and Soatto.
 *        See: http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf for details.
 *
 * \note  The clusterer is capable of clustering multiple sets of examples (in parallel, when using OpenMP),
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
class ExampleClusterer_CPU : public ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>
{
  //#################### USINGS ####################
public:
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::Clusters;

  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ClustersBlock;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ClustersBlock_Ptr;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ClustersBlock_CPtr;

  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleImage;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleImage_Ptr;
  using typename ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleImage_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an instance of ExampleClusterer_CPU.
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
  ExampleClusterer_CPU(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

  //#################### PROTECTED VIRTUAL MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Build a histgram of cluster sizes.
   *        One histogram per example set, each element of the histogram represents the number of clusters having a
   * certain size.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /**
   * \brief Compute the density of examples around each example in the input sets.
   *
   * \param exampleSets         A pointer to the example sets.
   * \param exampleSetsSizes    A pointer to the size of each example set.
   * \param exampleSetsCapacity The maximum size of each exampleSet.
   * \param exampleSetsCount    The number of example sets.
   * \param sigma               The sigma used when computing the density.
   */
  virtual void compute_density(const ExampleType *exampleSets,
                               const int *exampleSetsSizes,
                               uint32_t exampleSetsCapacity,
                               uint32_t exampleSetsCount,
                               float sigma);

  /**
   * \brief Having selected the examples part of each cluster, compute the cluster parameters
   *        (e.g. centroid, covariances etc...).
   *
   * \param exampleSets        A pointer to the example sets.
   * \param exampleSetsSizes   A pointer to the size of each example set.
   * \param predictionsData    A pointer to the output containers wherein to store the clusters
   *                           extracted from each set of examples.
   * \param maxClusterCount    Maximum number of clusters to select for each example set.
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void compute_cluster_parameters(const ExampleType *exampleSets,
                                          const int *exampleSetSizes,
                                          Clusters *clustersData,
                                          uint32_t maxClusterCount,
                                          uint32_t exampleSetCapacity,
                                          uint32_t exampleSetCount);

  /**
   * \brief Virtual function returning a pointer to the output cluster container for set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param clusters The output cluster containers.
   * \param setIdx   The index to the first set of interest.
   *
   * \return         A raw pointer to the output cluster container for set setIdx.
   */
  virtual Clusters *get_pointer_to_cluster(const ClustersBlock_Ptr &clusters, uint32_t clusterIdx) const;

  /**
   * \brief Virtual function returning a pointer to the first example of the example set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param exampleSets The example sets.
   * \param setIdx      The index to the first set of interest.
   *
   * \return            A raw pointer to the first example of the example set setIdx.
   */
  virtual const ExampleType *get_pointer_to_example_set(const ExampleImage_CPtr &exampleSets, uint32_t setIdx) const;

  /**
   * \brief Virtual function returning a pointer to the size of the example set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param exampleSetsSize The example set sizes.
   * \param setIdx          The index to the first set of interest.
   *
   * \return                A raw pointer to the size of the example set setIdx.
   */
  virtual const int *get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr &exampleSetsSize,
                                                     uint32_t setIdx) const;

  /**
   * \brief Analyse the tree structure to identify clusters of neighboring examples.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void identify_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /**
   * \brief Link neighboring examples in order to form a tree structure.
   *
   * \param exampleSets         A pointer to the example sets.
   * \param exampleSetsSizes    A pointer to the size of each example set.
   * \param exampleSetsCapacity The maximum size of each exampleSet.
   * \param exampleSetsCount    The number of example sets.
   * \param tauSq               Maximum distance (squared) between examples that are to be linked.
   */
  virtual void link_neighbors(const ExampleType *exampleSets,
                              const int *exampleSetsSizes,
                              uint32_t exampleSetsCapacity,
                              uint32_t exampleSetsCount,
                              float tauSq);

  /**
   * \brief Reset output cluster containers.
   *
   * \param clustersData  Pointer to the storage for the extracted clusters.
   * \param clustersCount Number of example sets to be clustered.
   */
  virtual void reset_clusters(Clusters *clustersData, uint32_t clustersCount) const;

  /**
   * \brief Reset temporary values used during the clustering operation.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount);

  /**
   * \brief Selects up to maxClusterCount clusters of at least minClusterSize size for each example set.
   *        Prefers larger clusters.
   *
   * \param maxClusterCount    Maximum number of clusters to select for each example set.
   * \param minClusterSize     Minimum size of a cluster to be selected.
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void select_clusters(uint32_t maxClusterCount,
                               uint32_t minClusterSize,
                               uint32_t exampleSetCapacity,
                               uint32_t exampleSetCount);
};

} // namespace grove

#endif // H_GROVE_EXAMPLECLUSTERERCPU
