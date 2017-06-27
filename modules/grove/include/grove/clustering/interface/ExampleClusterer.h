/**
 * grove: ExampleClusterer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER
#define H_GROVE_EXAMPLECLUSTERER

#include <boost/shared_ptr.hpp>

#include <ORUtils/Image.h>
#include <ORUtils/MemoryBlock.h>

#include <itmx/base/ITMImagePtrTypes.h>
#include <itmx/base/ITMMemoryBlockPtrTypes.h>

#include "../../util/Array.h"

namespace grove {

/**
 * \brief An instance of a class deriving from this one can find clusters from a set of examples.
 *        Clustering is performed via the "Really quick shift" algorithm by Fulkerson and Soatto.
 *        See http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf for details.
 *
 * \note  The clusterer is capable of clustering multiple sets of examples (in parallel, when using CUDA or OpenMP).
 *        For this reason, the interface to the main clustering method expects not a single set of examples, but an
 *        image in which each row contains a certain number of examples to be clustered. Different rows are then
 *        clustered independently.
 *
 * \note  The following functions must be defined for each example type:
 *
 *        1) _CPU_AND_GPU_CODE_ inline float distanceSquared(const ExampleType& a, const ExampleType& b);
 *
 *           Returns the squared distance between two examples.
 *
 *        2) _CPU_AND_GPU_CODE_ inline void createClusterFromExamples(const ExampleType *examples,
 *                                                                    const int *exampleKeys, int examplesCount,
 *                                                                    int key, ClusterType& outputCluster);
 *
 *           Aggregates all the examples in the examples array that have a certain key into a single cluster mode.
 *
 * \param ExampleType  The type of example to cluster.
 * \param ClusterType  The type of cluster being generated.
 * \param MAX_CLUSTERS The maximum number of clusters being generated for each set of examples.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
class ExampleClusterer
{
  //#################### TYPEDEFS ####################
protected:
  typedef Array<ClusterType,MAX_CLUSTERS> Clusters;
  typedef ORUtils::MemoryBlock<Clusters> ClustersBlock;
  typedef boost::shared_ptr<ClustersBlock> ClustersBlock_Ptr;
  typedef ORUtils::Image<ExampleType> ExampleImage;
  typedef boost::shared_ptr<const ExampleImage> ExampleImage_CPtr;

  //#################### PROTECTED VARIABLES ####################
protected:
  /** The maximum number of clusters to return for each set of examples. */
  uint32_t m_maxClusterCount;

  /** The minimum number of examples in a valid cluster. */
  uint32_t m_minClusterSize;

  /** The sigma used to estimate example densities. */
  float m_sigma;

  /** The maximum distance between examples in the same set. */
  float m_tau;

  //########################### FIND MODES STATE VARIABLES ###########################
  //                                                                                //
  // These variables are used to store the state needed when calling                //
  // find_modes(exampleSets, exampleSetSizes, startIdx, count, clusterContainers).  //
  //                                                                                //
  //##################################################################################
protected:
  /** Cluster index to which each example is associated. Has count rows and exampleSets->width columns. */
  ITMIntImage_Ptr m_clusterIdx;

  /** Size of each cluster. Has count rows and exampleSets->width columns. */
  ITMIntImage_Ptr m_clusterSizes;

  /** Histogram representing the number of clusters having a certain size. Has count rows and exampleSets->width columns. */
  ITMIntImage_Ptr m_clusterSizesHistogram;

  /** An image representing the density of examples around each example in the input sets. Has count rows and exampleSets->width columns. */
  ITMFloatImage_Ptr m_densities;

  /** Stores the number of valid clusters in each example set. Has count elements. */
  ITMIntMemoryBlock_Ptr m_nbClustersPerExampleSet;

  /** Defines the cluster tree structure. Holds the index to the parent for each example in the input sets. Has count rows and exampleSets->width columns. */
  ITMIntImage_Ptr m_parents;

  /** Stores the index of selected clusters in each example set. Has count rows and m_maxClusterCount columns. */
  ITMIntImage_Ptr m_selectedClusters;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an example clusterer.
   *
   * \param sigma            The sigma distance used when estimating the example density.
   * \param tau              The maximum distance there can be between two examples that are part of the same cluster.
   * \param maxClusterCount  The maximum number of clusters retained for each set of examples (all clusters are estimated
   *                         but only the maxClusterCount largest ones are returned). Must be <= MAX_CLUSTERS.
   * \param minClusterSize   The minimum number of elements there must be in a cluster for it to be valid.
   *
   * \throws std::invalid_argument If maxClusterCount > MAX_CLUSTERS.
   */
  ExampleClusterer(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the example clusterer.
   */
  virtual ~ExampleClusterer();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Clusters several sets of examples in parallel.
   *
   * \param exampleSets       An image containing the sets of examples to be clustered (one set per row). The width of
   *                          the image specifies the maximum number of examples that can be contained in each set.
   * \param exampleSetSizes   The number of valid examples in each row of the exampleSets image.
   * \param startIdx          The index of the first example set for which to compute clusters.
   * \param count             The number of example sets for which to compute clusters.
   * \param clusterContainers Output containers that will hold the clusters computed for each example set.
   *
   * \throws std::invalid_argument If startIdx + count would result in out-of-bounds access in exampleSets.
   */
  void find_modes(const ExampleImage_CPtr& exampleSets, const ITMIntMemoryBlock_CPtr& exampleSetSizes,
                  uint32_t startIdx, uint32_t count, ClustersBlock_Ptr& clusterContainers);

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Builds a histogram of cluster sizes.
   *        One histogram per example set, each element of the histogram represents the number of clusters having a
   * certain size.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Compute the density of examples around each example in the input sets.
   *
   * \param exampleSets         A pointer to the example sets.
   * \param exampleSetSizes     A pointer to the size of each example set.
   * \param exampleSetsCapacity The maximum size of each exampleSet.
   * \param exampleSetsCount    The number of example sets.
   * \param sigma               The sigma used when computing the density.
   */
  virtual void compute_density(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetsCapacity,
                               uint32_t exampleSetsCount, float sigma) = 0;

  /**
   * \brief Having selected the examples part of each cluster, compute the cluster parameters
   *        (e.g. centroid, covariances etc...).
   *
   * \param exampleSets        A pointer to the example sets.
   * \param exampleSetSizes    A pointer to the size of each example set.
   * \param predictionsData    A pointer to the output containers wherein to store the clusters
   *                           extracted from each set of examples.
   * \param maxClusterCount    Maximum number of clusters to select for each example set.
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void compute_cluster_parameters(const ExampleType *exampleSets, const int *exampleSetSizes, Clusters *clustersData,
                                          uint32_t maxClusterCount, uint32_t exampleSetsCapacity, uint32_t exampleSetsCount) = 0;

  /**
   * \brief Virtual function returning a pointer to the output cluster container for set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param clusters The output cluster containers.
   * \param setIdx   The index to the first set of interest.
   *
   * \return         A raw pointer to the output cluster container for set setIdx.
   */
  virtual Clusters *get_pointer_to_cluster(const ClustersBlock_Ptr& clusters, uint32_t clusterIdx) const = 0;

  /**
   * \brief Virtual function returning a pointer to the first example of the example set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param exampleSets The example sets.
   * \param setIdx      The index to the first set of interest.
   *
   * \return            A raw pointer to the first example of the example set setIdx.
   */
  virtual const ExampleType *get_pointer_to_example_set(const ExampleImage_CPtr& exampleSets, uint32_t setIdx) const = 0;

  /**
   * \brief Virtual function returning a pointer to the size of the example set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param exampleSetSizes The example set sizes.
   * \param setIdx          The index to the first set of interest.
   *
   * \return                A raw pointer to the size of the example set setIdx.
   */
  virtual const int *get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr& exampleSetSizes, uint32_t setIdx) const = 0;

  /**
   * \brief Analyse the tree structure to identify clusters of neighboring examples.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void identify_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Link neighboring examples in order to form a tree structure.
   *
   * \param exampleSets         A pointer to the example sets.
   * \param exampleSetSizes     A pointer to the size of each example set.
   * \param exampleSetsCapacity The maximum size of each exampleSet.
   * \param exampleSetsCount    The number of example sets.
   * \param tauSq               Maximum distance (squared) between examples that are to be linked.
   */
  virtual void link_neighbors(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetsCapacity,
                              uint32_t exampleSetsCount, float tauSq) = 0;

  /**
   * \brief Reset output cluster containers.
   *
   * \param clustersData  Pointer to the storage for the extracted clusters.
   * \param clustersCount Number of example sets to be clustered.
   */
  virtual void reset_clusters(Clusters *clustersData, uint32_t clustersCount) const = 0;

  /**
   * \brief Reset temporary values used during the clustering operation.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Selects up to maxClusterCount clusters of at least minClusterSize size for each example set.
   *        Prefers larger clusters.
   *
   * \param maxClusterCount    Maximum number of clusters to select for each example set.
   * \param minClusterSize     Minimum size of a cluster to be selected.
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  virtual void select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Allocate memory to store the temporary data used to perform clustering.
   *
   * \param exampleSetCapacity Maximum size of each example set.
   * \param exampleSetCount    Number of example sets to be clustered.
   */
  void allocate_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount);
};

}

#endif
