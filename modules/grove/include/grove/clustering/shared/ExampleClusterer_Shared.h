/**
 * grove: ExampleClusterer_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER_SHARED
#define H_GROVE_EXAMPLECLUSTERER_SHARED

#include <ORUtils/MathUtils.h>
#include <ORUtils/PlatformIndependence.h>

#include "../../util/Array.h"

namespace grove {

/**
 * \brief Computes the final cluster index for the specified example by following the parent links computed in compute_parent.
 *
 * \note  The compute_parent function split all the examples into subtrees and allocated a cluster index to the root
 *        of each subtree. With this function, we navigate each example's subtree until we find the root and then copy
 *        the cluster index across to the example. We also compute the size of each cluster.
 *
 * \param exampleSetIdx      The index of the example set containing the example.
 * \param exampleIdx         The index of the example within its example set.
 * \param exampleSetCapacity The maximum size of each example set.
 * \param parents            An image containing the parent indices for the examples.
 * \param clusterIndices     An image containing the cluster indices for the examples.
 * \param clusterSizes       An array in which to keep track of the size of each cluster. Must contain zeros
 *                           at the point at which the function is called.
 */
_CPU_AND_GPU_CODE_
inline void compute_cluster_index(int exampleSetIdx, int exampleIdx, int exampleSetCapacity, const int *parents, int *clusterIndices, int *clusterSizes)
{
  // Compute the linear offset to the beginning of the data associated with the specified example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Compute the raster offset of the specified example in the example sets image.
  const int exampleOffset = exampleSetOffset + exampleIdx;

  // Follow the parent links from the specified example up to the root of its subtree.
  // Note that there is no need to check if the example is valid, since compute_parent
  // set the parents of invalid examples to themselves.
  int currentIdx = exampleIdx;
  int parentIdx = parents[exampleOffset];
  while(parentIdx != currentIdx)
  {
    currentIdx = parentIdx;
    parentIdx = parents[exampleSetOffset + parentIdx];
  }

  // Get the cluster index of the subtree root and assign it to this example.
  const int clusterIdx = clusterIndices[exampleSetOffset + parentIdx];
  clusterIndices[exampleOffset] = clusterIdx;

  // If the cluster is valid then atomically increase its size (it might be invalid if we started from an invalid example).
  if(clusterIdx >= 0)
  {
    // Note: The __CUDA_ARCH__ check is needed because this function is not a template.
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
    atomicAdd(&clusterSizes[exampleSetOffset + clusterIdx], 1);
#else
  #ifdef WITH_OPENMP
    #pragma omp atomic
  #endif
    clusterSizes[exampleSetOffset + clusterIdx]++;
#endif
  }
}

/**
 * \brief Compute the density of examples around an individual example in one of the example sets.
 *
 * \param exampleSetIdx      The index of the example set containing the example.
 * \param exampleIdx         The index of the example within its example set.
 * \param exampleSets        An image containing the sets of examples to be clustered (one set per row). The width of
 *                           the image specifies the maximum number of examples that can be contained in each set.
 * \param exampleSetSizes    The number of valid examples in each example set.
 * \param exampleSetCapacity The maximum size of each example set.
 * \param sigma              The sigma of the Gaussian used when computing the example density.
 * \param densities          The memory in which to store the density of each example (one example set per row,
 *                           one density value per column).
 */
template <typename ExampleType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_density(int exampleSetIdx, int exampleIdx, const ExampleType *exampleSets, const int *exampleSetSizes,
                            int exampleSetCapacity, float sigma, float *densities)
{
  // Compute the linear offset to the beginning of the data associated with the specified example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Compute the raster offset of the specified example in the example sets image.
  const int exampleOffset = exampleSetOffset + exampleIdx;

  // Look up the size of the specified example set.
  const int exampleSetSize = exampleSetSizes[exampleSetIdx];

  float density = 0.0f;

  // If the example is valid, loop over all of the examples in its set and
  // compute the density based on the examples that are within 3 * sigma of it
  // (points further away would only make a small contribution to the density).
  if(exampleIdx < exampleSetSize)
  {
    const float threeSigmaSq = (3.0f * sigma) * (3.0f * sigma);
    const float minusOneOverTwoSigmaSq = -1.0f / (2.0f * sigma * sigma);

    const ExampleType centreExample = exampleSets[exampleOffset];
    for(int i = 0; i < exampleSetSize; ++i)
    {
      const ExampleType otherExample = exampleSets[exampleSetOffset + i];

      // Note: ExampleType must have a distance_squared function defined for it.
      const float normSq = distance_squared(centreExample, otherExample);
      if(normSq < threeSigmaSq)
      {
        density += expf(normSq * minusOneOverTwoSigmaSq);
      }
    }
  }

  densities[exampleOffset] = density;
}

/**
 * \brief Computes the parent and initial cluster indices to assign to the specified example as part of the neighbour-linking step
 *        of the really quick shift (RQS) algorithm.
 *
 * \note Each example becomes part of a subtree in which each example has as its parent the closest example with higher density.
 *       For details about RQS, see the paper by Fulkerson and Soatto: http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf
 *
 * \param exampleSetIdx           The index of the example set containing the example.
 * \param exampleIdx              The index of the example within its example set.
 * \param exampleSets             An image containing the sets of examples to be clustered (one set per row). The width of
 *                                the image specifies the maximum number of examples that can be contained in each set.
 * \param exampleSetCapacity      The maximum number of examples in an example set.
 * \param exampleSetSizes         The number of valid examples in each example set.
 * \param densities               An image containing the density of each example (one set per row, one density value per column).
 * \param tauSq                   The square of the maximum distance allowed between examples if they are to be linked.
 * \param parents                 An image in which to store a parent index for each example. This will generally be the index of
 *                                another example in the same set, but may be that of the example itself if it is a subtree root.
 * \param clusterIndices          An image in which to store an initial cluster index for each example. The initial cluster index
 *                                for an example will be set to -1 unless the example is a subtree root.
 * \param nbClustersPerExampleSet An array in which to keep track of the number of clusters in each example set. Must contain zeros
 *                                at the point at which the function is called.
 */
template <typename ExampleType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void compute_parent(int exampleSetIdx, int exampleIdx, const ExampleType *exampleSets, int exampleSetCapacity, const int *exampleSetSizes,
                           const float *densities, float tauSq, int *parents, int *clusterIndices, int *nbClustersPerExampleSet)
{
  // Compute the linear offset to the beginning of the data associated with the specified example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Compute the raster offset of the specified example in the example sets image.
  const int exampleOffset = exampleSetOffset + exampleIdx;

  // Look up the size of the specified example set.
  const int exampleSetSize = exampleSetSizes[exampleSetIdx];

  // Unless it becomes part of a subtree, each example starts as its own parent.
  int parentIdx = exampleIdx;

  // The index of the cluster associated with the specified example (-1 except for subtree roots).
  int clusterIdx = -1;

  // If the specified example is valid:
  if(exampleIdx < exampleSetSize)
  {
    // Read in the example and its density from global memory.
    const ExampleType centreExample = exampleSets[exampleOffset];
    const float centreDensity = densities[exampleOffset];

    // We are only interested in examples whose distance to the specified example is less than tau.
    float minDistanceSq = tauSq;

    // For each other example in the specified example's set:
    for(int i = 0; i < exampleSetSize; ++i)
    {
      if(i == exampleIdx) continue;

      // Read in the other example and its density from global memory.
      const ExampleType otherExample = exampleSets[exampleSetOffset + i];
      const float otherDensity = densities[exampleSetOffset + i];

      // Compute the squared distance between the specified example and the other example.
      // Note: ExampleType must have a distance_squared function defined for it.
      const float otherDistSq = distance_squared(centreExample, otherExample);

      // We are looking for the closest example with a higher density (doesn't matter by how much) than that of the specified example.
      if(otherDensity > centreDensity && otherDistSq < minDistanceSq)
      {
        minDistanceSq = otherDistSq;
        parentIdx = i;
      }
    }

    // If the specified example is still its own parent (i.e. it is a subtree root), we didn't find any close
    // example with a higher density, so grab a unique cluster index for the example.
    if(parentIdx == exampleIdx)
    {
#ifdef __CUDACC__
      clusterIdx = atomicAdd(&nbClustersPerExampleSet[exampleSetIdx], 1);
#else
    #ifdef WITH_OPENMP
      #pragma omp atomic capture
    #endif
      clusterIdx = nbClustersPerExampleSet[exampleSetIdx]++;
#endif
    }
  }

  // Write the parent of the specified example to global memory.
  parents[exampleOffset] = parentIdx;

  // Write the cluster index associated with the example to global memory. (This will be -1 unless the example is a subtree root).
  clusterIndices[exampleOffset] = clusterIdx;
}

/**
 * \brief Computes the parameters for and stores the specified selected cluster for the specified example set.
 *
 * \note  The examples in the set must already have been grouped into clusters by the other functions in this file,
 *        and suitable clusters must have been selected for creation.
 * \note  The actual cluster parameters depend on the ClusterType; for this reason, cluster creation is delegated to a function
 *        called create_cluster_from_examples, which must be defined (elsewhere) for the relevant ExampleType and ClusterType pair.
 *
 * \param exampleSetIdx       The index of the example set for which the selected cluster is being created.
 * \param selectedClusterIdx  The index of the selected cluster to create (this is an index into the selected clusters array).
 * \param exampleSets         An image containing the sets of examples that have been clustered (one set per row). The width
 *                            of the image specifies the maximum number of examples that can be contained in each set.
 * \param exampleSetSizes     The number of valid examples in each example set.
 * \param exampleSetCapacity  The maximum size of each example set.
 * \param clusterIndices      An image containing the cluster indices for the examples.
 * \param selectedClusters    The indices of the clusters selected for each example set.
 * \param maxSelectedClusters The maximum number of clusters to extract from each example set.
 * \param clusterContainers   The output containers in which to store the clusters created for each example set.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void create_selected_cluster(int exampleSetIdx, int selectedClusterIdx, const ExampleType *exampleSets, const int *exampleSetSizes, int exampleSetCapacity,
                                    const int *clusterIndices, const int *selectedClusters, int maxSelectedClusters, Array<ClusterType,MAX_CLUSTERS> *clusterContainers)
{
  // Compute the linear offset to the beginning of the data associated with the selected clusters for the specified example set.
  const int selectedClustersOffset = exampleSetIdx * maxSelectedClusters;

  // Look up the real cluster index of the specified selected cluster (e.g. if this is the second selected
  // cluster for the example set, and selectedClusters[1] = 23, then the real cluster index is 23).
  const int clusterIdx = selectedClusters[selectedClustersOffset + selectedClusterIdx];

  // If the specified selected cluster is valid:
  if(clusterIdx >= 0)
  {
    // Compute the linear offset to the beginning of the data associated with the specified example set
    // in the example sets and cluster indices images.
    const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

    // Get a reference to the output clusters array for the specified example set.
    Array<ClusterType,MAX_CLUSTERS>& outputClusters = clusterContainers[exampleSetIdx];

    // Compute the index in the output clusters array at which to store the selected cluster once it has been created.
    int outputClusterIdx = -1;

#ifdef __CUDACC__
    outputClusterIdx = atomicAdd(&outputClusters.size, 1);
#else
  #ifdef WITH_OPENMP
    #pragma omp atomic capture
  #endif
    outputClusterIdx = outputClusters.size++;
#endif

    // Create the cluster and write it into the output clusters array at the specified location.
    create_cluster_from_examples(
      clusterIdx,
      exampleSets + exampleSetOffset,
      clusterIndices + exampleSetOffset,
      exampleSetSizes[exampleSetIdx],
      outputClusters.elts[outputClusterIdx]
    );
  }
}

/**
 * \brief Resets a cluster container.
 *
 * \param clusterContainers A pointer to the cluster containers.
 * \param exampleSetIdx     The index of the example set whose cluster container we want to reset.
 */
template <typename ClusterType, int MAX_CLUSTERS>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void reset_cluster_container(Array<ClusterType,MAX_CLUSTERS> *clusterContainers, int exampleSetIdx)
{
  // It is sufficient to just reset the size of the container (i.e. the number of clusters) to zero.
  // There is no need to modify the actual clusters, since they will be overwritten later.
  clusterContainers[exampleSetIdx].size = 0;
}

/**
 * \brief Resets the temporary variables associated with the specified example set.
 *
 * \param exampleSetIdx           The index of the example set for which we are resetting the temporary variables.
 * \param exampleSetCapacity      The maximum size of each example set.
 * \param nbClustersPerExampleSet The number of clusters extracted from each example set.
 * \param clusterSizes            An image containing the sizes of the extracted clusters (for all example sets).
 * \param clusterSizeHistograms   The histograms of cluster sizes for the different example sets.
 */
_CPU_AND_GPU_CODE_
inline void reset_temporaries_for_set(int exampleSetIdx, int exampleSetCapacity, int *nbClustersPerExampleSet, int *clusterSizes, int *clusterSizeHistograms)
{
  // Reset the number of clusters extracted from the specified example set to zero.
  nbClustersPerExampleSet[exampleSetIdx] = 0;

  // Compute the linear offset to the beginning of the data associated with the specified example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Reset the cluster sizes and histogram values associated with the specified example set.
  for(int i = 0; i < exampleSetCapacity; ++i)
  {
    clusterSizes[exampleSetOffset + i] = 0;
    clusterSizeHistograms[exampleSetOffset + i] = 0;
  }
}

/**
 * \brief Selects the largest clusters for the specified example set and writes their indices into the selected clusters image.
 *
 * \param exampleSetIdx           The index of the example set for which to select clusters.
 * \param clusterSizes            An image containing the sizes of the extracted clusters (for all example sets).
 * \param clusterSizeHistograms   The histograms of cluster sizes for the different example sets.
 * \param nbClustersPerExampleSet The number of clusters extracted from each example set.
 * \param exampleSetCapacity      The mamimum size of each example set.
 * \param maxSelectedClusters     The maximum number of clusters to keep for each example set.
 * \param minClusterSize          The minimum size of cluster to keep.
 * \param selectedClusters        An image in which to store the indices of the clusters selected for each example set.
 */
_CPU_AND_GPU_CODE_
inline void select_clusters_for_set(int exampleSetIdx, const int *clusterSizes, const int *clusterSizeHistograms, const int *nbClustersPerExampleSet,
                                    int exampleSetCapacity, int maxSelectedClusters, int minClusterSize, int *selectedClusters)
{
  // Compute the linear offset to the beginning of the data associated with the specified example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Look up the number of valid clusters associated with the specified example set.
  const int nbValidClusters = nbClustersPerExampleSet[exampleSetIdx];

  // Compute the linear offset to the beginning of the data associated with the selected clusters for the specified example set.
  const int selectedClustersOffset = exampleSetIdx * maxSelectedClusters;

  // Reset the selected clusters for the specified example set (by setting all selected cluster indices to an invalid value).
  for(int i = 0; i < maxSelectedClusters; ++i)
  {
    selectedClusters[selectedClustersOffset + i] = -1;
  }

  // Starting from the largest clusters, scan downwards in the histogram to find the minimum size of cluster
  // we need to consider in order to try to select maxSelectedClusters clusters. Note that we will not be
  // able to select maxSelectedClusters if there are fewer suitable clusters than that to start with; if
  // that happens, we simply keep all of the suitable clusters we do have.
  int nbSelectedClusters = 0;
  int nbSmallestClustersToKeep = 0;
  int minSelectedClusterSize = exampleSetCapacity;
  while(minSelectedClusterSize > minClusterSize && nbSelectedClusters < maxSelectedClusters)
  {
    --minSelectedClusterSize;
    nbSmallestClustersToKeep = MIN(clusterSizeHistograms[exampleSetOffset + minSelectedClusterSize], maxSelectedClusters - nbSelectedClusters);
    nbSelectedClusters += nbSmallestClustersToKeep;
  }

  // If we couldn't find any suitable clusters at all, early out.
  if(nbSelectedClusters == 0) return;

  // Now walk through all of the clusters we do have, selecting (a) all of those whose size is strictly greater
  // than the minimum selected cluster size, and (b) as many as necessary of those whose size is exactly equal
  // to the minimum selected cluster size.
  nbSelectedClusters = 0;
  int nbSmallestClustersKept = 0;
  for(int clusterIdx = 0; clusterIdx < nbValidClusters; ++clusterIdx)
  {
    const int clusterSize = clusterSizes[exampleSetOffset + clusterIdx];
    if(clusterSize > minSelectedClusterSize ||
       (clusterSize == minSelectedClusterSize && nbSmallestClustersKept++ < nbSmallestClustersToKeep))
    {
      selectedClusters[selectedClustersOffset + nbSelectedClusters++] = clusterIdx;
    }
  }

  // Sort the selected clusters in non-increasing order of size using a simple selection sort.
  // Note: Selection sort is quadratic, but the number of clusters is small enough for now that it doesn't matter.
  for(int i = 0; i < nbSelectedClusters; ++i)
  {
    // Find a cluster with maximum size in selectedClusters[i..nbSelectedClusters).
    int maxSize = clusterSizes[exampleSetOffset + selectedClusters[selectedClustersOffset + i]];
    int maxIdx = i;

    for(int j = i + 1; j < nbSelectedClusters; ++j)
    {
      int size = clusterSizes[exampleSetOffset + selectedClusters[selectedClustersOffset + j]];
      if(size > maxSize)
      {
        maxSize = size;
        maxIdx = j;
      }
    }

    // If selectedClusters[i] wasn't the maximal cluster, swap it with the cluster that was.
    if(maxIdx != i)
    {
      int temp = selectedClusters[selectedClustersOffset + i];
      selectedClusters[selectedClustersOffset + i] = selectedClusters[selectedClustersOffset + maxIdx];
      selectedClusters[selectedClustersOffset + maxIdx] = temp;
    }
  }
}

/**
 * \brief Updates the cluster size histogram for the specified example set based on the size of the specified cluster.
 *
 * \note The cluster size histograms will be used later to select the largest clusters in each example set.
 *
 * \param exampleSetIdx           The index of the example set whose histogram should be updated.
 * \param clusterIdx              The index of the cluster whose size should be used to update the histogram.
 * \param clusterSizes            An image containing the sizes of the extracted clusters (for all example sets).
 * \param clusterSizeHistograms   An image storing a cluster size histogram for each example set under consideration (one histogram per row).
 * \param exampleSetCapacity      The maximum number of elements in each example set.
 */
_CPU_AND_GPU_CODE_
inline void update_cluster_size_histogram(int exampleSetIdx, int clusterIdx, const int *clusterSizes, int *clusterSizeHistograms, int exampleSetCapacity)
{
  // Compute the linear offset to the beginning of the data associated with the specified example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Look up the size of the specified cluster.
  const int clusterSize = clusterSizes[exampleSetOffset + clusterIdx];

  // Atomically increment the corresponding bin in the histogram.
  // Note: The __CUDA_ARCH__ check is needed because this function is not a template.
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
  atomicAdd(&clusterSizeHistograms[exampleSetOffset + clusterSize], 1);
#else
#ifdef WITH_OPENMP
  #pragma omp atomic
#endif
  clusterSizeHistograms[exampleSetOffset + clusterSize]++;
#endif
}

}

#endif
