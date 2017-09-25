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
 *        1) _CPU_AND_GPU_CODE_ inline float distance_squared(const ExampleType& a, const ExampleType& b);
 *
 *           Returns the squared distance between two examples.
 *
 *        2) _CPU_AND_GPU_CODE_ inline void create_cluster_from_examples(int key, const ExampleType *examples,
 *                                                                       const int *exampleKeys, int examplesCount,
 *                                                                       ClusterType& outputCluster);
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
  /** The maximum number of clusters to retain for each set of examples. */
  uint32_t m_maxClusterCount;

  /** The minimum size of cluster to keep. */
  uint32_t m_minClusterSize;

  /** The sigma of the Gaussian used when computing the example densities. */
  float m_sigma;

  /** The maximum distance there can be between two examples that are part of the same cluster. */
  float m_tau;

  //##################### CLUSTER EXAMPLES TEMPORARY VARIABLES #####################
  //                                                                              //
  // These temporary variables are used to store the state needed when invoking:  //
  //                                                                              //
  // cluster_examples(exampleSets, exampleSetSizes, exampleSetStart,              //
  //            exampleSetCount, clusterContainers);                              //
  //                                                                              //
  //################################################################################
protected:
  /** An image storing the cluster index associated with each example in the input sets. Has exampleSetCount rows and exampleSets->width columns. */
  ITMIntImage_Ptr m_clusterIndices;

  /**
   * An image storing a cluster size histogram for each example set under consideration (one histogram per row).
   * Pixel (i,j) in the image counts the number of clusters in example set i that have size j.
   */
  ITMIntImage_Ptr m_clusterSizeHistograms;

  /**
   * The size of each cluster (for each considered example set). Has exampleSetCount rows and exampleSets->width columns.
   * The number of clusters for each example set can range between 1 (i.e. a single cluster of size exampleSets->width)
   * and exampleSets->width (i.e. a cluster for each individual example). Within the row of the image corresponding to
   * example set i, the first m_nbClustersPerExampleSet[i] pixels store the sizes of the clusters for that example set.
   * The remaining pixels on the row are ignored.
   */
  ITMIntImage_Ptr m_clusterSizes;

  /** An image storing the density of examples around each example in the input sets. Has exampleSetCount rows and exampleSets->width columns. */
  ITMFloatImage_Ptr m_densities;

  /** Stores the number of valid clusters in each example set. Has exampleSetCount elements. */
  ITMIntMemoryBlock_Ptr m_nbClustersPerExampleSet;

  /** Defines the cluster tree structure. Holds the index of the parent for each example in the input sets. Has exampleSetCount rows and exampleSets->width columns. */
  ITMIntImage_Ptr m_parents;

  /** Stores the indices of the selected clusters in each example set. Has exampleSetCount rows and m_maxClusterCount columns. */
  ITMIntImage_Ptr m_selectedClusters;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an example clusterer.
   *
   * \param sigma            The sigma of the Gaussian used when computing the example densities.
   * \param tau              The maximum distance there can be between two examples that are part of the same cluster.
   * \param maxClusterCount  The maximum number of clusters retained for each set of examples (all clusters are estimated
   *                         but only the maxClusterCount largest ones are returned). Must be <= MAX_CLUSTERS.
   * \param minClusterSize   The minimum size of cluster to keep.
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
   * \param exampleSetSizes   The number of valid examples in each example set.
   * \param exampleSetStart   The index of the first example set for which to compute clusters.
   * \param exampleSetCount   The number of example sets for which to compute clusters.
   * \param clusterContainers Output containers that will hold the clusters computed for each example set.
   *
   * \throws std::invalid_argument If exampleSetStart + exampleSetCount would result in out-of-bounds access in exampleSets.
   */
  void cluster_examples(const ExampleImage_CPtr& exampleSets, const ITMIntMemoryBlock_CPtr& exampleSetSizes,
                        uint32_t exampleSetStart, uint32_t exampleSetCount, ClustersBlock_Ptr& clusterContainers);

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Computes final cluster indices for the examples by following the parent links previously computed.
   *
   * \param exampleSetCapacity The maximum size of each example set.
   * \param exampleSetCount    The number of example sets being clustered.
   */
  virtual void compute_cluster_indices(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Builds a histogram of cluster sizes.
   *        One histogram per example set, each element of the histogram represents the number of clusters having a
   * certain size.
   *
   * \param exampleSetCapacity The maximum size of each example set.
   * \param exampleSetCount    The number of example sets being clustered.
   */
  virtual void compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Compute the density of examples around each example in the input sets.
   *
   * \param exampleSets         An image containing the sets of examples to be clustered (one set per row). The width of
   *                            the image specifies the maximum number of examples that can be contained in each set.
   * \param exampleSetSizes     The number of valid examples in each example set.
   * \param exampleSetCapacity  The maximum size of each example set.
   * \param exampleSetCount     The number of example sets being clustered.
   */
  virtual void compute_densities(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Computes parents and initial cluster indices for the examples as part of the neighbour-linking step of the
   *        really quick shift (RQS) algorithm.
   *
   * \note For details of RQS, see the paper by Fulkerson and Soatto: http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf
   *
   * \param exampleSets         An image containing the sets of examples to be clustered (one set per row). The width of
   *                            the image specifies the maximum number of examples that can be contained in each set.
   * \param exampleSetSizes     The number of valid examples in each example set.
   * \param exampleSetCapacity  The maximum size of each example set.
   * \param exampleSetCount     The number of example sets being clustered.
   * \param tauSq               The square of the maximum distance allowed between examples if they are to be linked.
   */
  virtual void compute_parents(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetCapacity,
                               uint32_t exampleSetCount, float tauSq) = 0;

  /**
   * \brief Computes the parameters (e.g. centroid, covariances, etc.) for and stores each selected cluster for each example set.
   *
   * \param exampleSets         An image containing the sets of examples to be clustered (one set per row). The width of
   *                            the image specifies the maximum number of examples that can be contained in each set.
   * \param exampleSetSizes     The number of valid examples in each example set.
   * \param exampleSetCapacity  The maximum size of each example set.
   * \param exampleSetCount     The number of example sets being clustered.
   * \param clustersData        A pointer to the start of the cluster container for the first example set being clustered.
   */
  virtual void create_selected_clusters(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetCapacity,
                                        uint32_t exampleSetCount, Clusters *clustersData) = 0;

  /**
   * \brief Virtual function returning a pointer to the output cluster container for set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param clusters The output cluster containers.
   * \param setIdx   The index to the first set of interest.
   * \return         A raw pointer to the output cluster container for set setIdx.
   */
  virtual Clusters *get_pointer_to_cluster(const ClustersBlock_Ptr& clusters, uint32_t clusterIdx) const = 0;

  /**
   * \brief Virtual function returning a pointer to the first example of the example set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param exampleSets An image containing the sets of examples to be clustered (one set per row). The width of
   *                    the image specifies the maximum number of examples that can be contained in each set.
   * \param setIdx      The index to the first set of interest.
   * \return            A raw pointer to the first example of the example set setIdx.
   */
  virtual const ExampleType *get_pointer_to_example_set(const ExampleImage_CPtr& exampleSets, uint32_t setIdx) const = 0;

  /**
   * \brief Virtual function returning a pointer to the size of the example set setIdx.
   *        Used to get a raw pointer, independent from the memory type.
   *
   * \param exampleSetSizes The number of valid examples in each example set.
   * \param setIdx          The index to the first set of interest.
   * \return                A raw pointer to the size of the example set setIdx.
   */
  virtual const int *get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr& exampleSetSizes, uint32_t setIdx) const = 0;

  /**
   * \brief Resets the output cluster containers.
   *
   * \param clustersData    A pointer to the start of the cluster container for the first example set being clustered.
   * \param exampleSetCount The number of example sets being clustered.
   */
  virtual void reset_clusters(Clusters *clustersData, uint32_t exampleSetCount) const = 0;

  /**
   * \brief Resets the temporary variables needed during a find_modes call.
   *
   * \param exampleSetCapacity The maximum size of each example set.
   * \param exampleSetCount    The number of example sets being clustered.
   */
  virtual void reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  /**
   * \brief Selects the largest clusters for each example set (up to a maximum limit).
   *
   * \param exampleSetCapacity The maximum size of each example set.
   * \param exampleSetCount    The number of example sets being clustered.
   */
  virtual void select_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount) = 0;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Reallocates the temporary variables needed during a find_modes call as necessary.
   *
   * \param exampleSetCapacity The maximum size of each example set.
   * \param exampleSetCount    The number of example sets being clustered.
   */
  void reallocate_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount);
};

}

#endif
