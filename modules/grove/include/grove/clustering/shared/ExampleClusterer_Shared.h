/**
 * grove: ExampleClusterer_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER_SHARED
#define H_GROVE_EXAMPLECLUSTERER_SHARED

#include <ORUtils/PlatformIndependence.h>

#include "../../util/Array.h"

namespace grove {

/**
 * \brief Compute an histogram of cluster sizes for each example set. Used later to select the largest clusters.
 *
 * \param clusterSizes            Pointer to the memory area wherein are stored the sizes of the clusters.
 * \param nbClustersPerExampleSet Pointer to an array wherein is stored the number of clusters extracted from each
 * example set.
 * \param clusterSizesHistogram   Pointer to the memory wherein the histogram of sizes will be stored.
 *                                One column for each element in the example sets (we might have either a single cluster
 *                                of maximum size or exampleSets.width clusters of size 1), one row per example set.
 * \param exampleSetCapacity      The maximum number of elements in each example set.
 * \param exampleSetIdx           The index of the current example set.
 * \param clusterIdx              The index of the current cluster.
 */
_CPU_AND_GPU_CODE_
inline void example_clusterer_compute_cluster_histogram(const int *clusterSizes,
                                                        const int *nbClustersPerExampleSet,
                                                        int *clusterSizesHistogram,
                                                        int exampleSetCapacity,
                                                        int exampleSetIdx,
                                                        int clusterIdx)
{
  // Linear offset to the start of the current example set (or its associated data).
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;
  // Number of valid clusters for the current example set.
  const int nbValidClusters = nbClustersPerExampleSet[exampleSetIdx];

  // If the current cluster is invalid early out.
  if (clusterIdx >= nbValidClusters) return;

  // Grab the size for the current cluster.
  const int clusterSize = clusterSizes[exampleSetOffset + clusterIdx];

// Atomically increment the associated bin.
#if defined(__CUDACC__) && defined(__CUDA_ARCH__) // Non templated function, need the __CUDA_ARCH__ check.
  atomicAdd(&clusterSizesHistogram[exampleSetOffset + clusterSize], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic
#endif
  clusterSizesHistogram[exampleSetOffset + clusterSize]++;
#endif
}

/**
 * \brief Compute the density of examples around a certain element of the example sets.
 *
 * \param exampleSets        A pointer to the sets of examples (one set per row, one example per column).
 * \param exampleSetSizes    A pointer to the actual sizes of each example set (number of valid elements in each row of
 *                           exampleSets).
 * \param densities          A pointer to the memory wherein to store the density of each example (one example set per
 *                           row, one density value per column).
 * \param exampleSetCapacity The maximum size of each example set (number of columns in exampleSets).
 * \param exampleSetIdx      The index of the current example set.
 * \param elementIdx         The index of the element for which we are computing the density.
 * \param sigma              Sigma of the gaussian used when computing the density.
 */
template <typename ExampleType>
_CPU_AND_GPU_CODE_TEMPLATE_ inline void example_clusterer_compute_density(const ExampleType *exampleSets,
                                                                          const int *exampleSetSizes,
                                                                          float *densities,
                                                                          int exampleSetCapacity,
                                                                          int exampleSetIdx,
                                                                          int elementIdx,
                                                                          float sigma)
{
  // Compute the linear offset to the beginning of the data associated to the current example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;
  // The size of the current example set.
  const int exampleSetSize = exampleSetSizes[exampleSetIdx];
  // Offset of the current element from the beginning of the exampleSets array.
  const int elementOffset = exampleSetOffset + elementIdx;

  // Points farther away than three sigma have small contribution to the density.
  const float threeSigmaSq = (3.f * sigma) * (3.f * sigma);
  const float minusOneOverTwoSigmaSq = -1.f / (2.f * sigma * sigma);

  float density = 0.f;

  if (elementIdx < exampleSetSize)
  {
    const ExampleType centerExample = exampleSets[elementOffset];

    for (int i = 0; i < exampleSetSize; ++i)
    {
      const ExampleType otherExample = exampleSets[exampleSetOffset + i];

      // ExampleType MUST have a distanceSquared function defined for it.
      const float normSq = distanceSquared(centerExample, otherExample);

      // we ignore points farther away than three sigma.
      if (normSq < threeSigmaSq)
      {
        density += expf(normSq * minusOneOverTwoSigmaSq);
      }
    }
  }

  // Store the density in the output variable.
  densities[elementOffset] = density;
}

/**
 * \brief Compute the cluster parameters associated to a set of examples grouped by the other functions in this file.
 *
 * \note  The actual cluster parameters depend on the ClusterType and for this reason are left to the
 *        createClusterFromExamples function that MUST be defined for the current ExampleType.
 *
 * \param exampleSets         A pointer to the examples to cluster. One row per example set, one column per example.
 * \param exampleSetSizes     A pointer to the sizes of each example set.
 * \param clusterIndices      A pointer to the indices of the clusters associated to each example.
 * \param selectedClusters    A pointer to the cluster indices selected for each example set.
 * \param clusterContainers   A pointer to the output variables wherein to store the cluster parameters.
 * \param exampleSetCapacity  The maximum number of examples in each example set.
 * \param exampleSetIdx       The index of the current example set.
 * \param maxSelectedClusters The maximum number of clusters to extract from each example set.
 * \param clusterIdx          The index of the current cluster.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
_CPU_AND_GPU_CODE_TEMPLATE_ inline void
    example_clusterer_compute_modes(const ExampleType *exampleSets,
                                    const int *exampleSetSizes,
                                    const int *clusterIndices,
                                    const int *selectedClusters,
                                    Array<ClusterType, MAX_CLUSTERS> *clusterContainers,
                                    int exampleSetCapacity,
                                    int exampleSetIdx,
                                    int maxSelectedClusters,
                                    int clusterIdx)
{
  // Linear offset to the first selected cluster of the current example set.
  const int selectedClustersOffset = exampleSetIdx * maxSelectedClusters;
  // Unique identifier to the current cluster.
  const int selectedClusterId = selectedClusters[selectedClustersOffset + clusterIdx];

  // If this is a valid cluster.
  if (selectedClusterId >= 0)
  {
    // Grab a reference to the cluster container for the current example set.
    Array<ClusterType, MAX_CLUSTERS> &currentClusterContainer = clusterContainers[exampleSetIdx];

    // Atomically get the output index associated to the cluster.
    int outputClusterIdx = -1;

#ifdef __CUDACC__
    outputClusterIdx = atomicAdd(&currentClusterContainer.size, 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
    outputClusterIdx = currentClusterContainer.size++;
#endif

    // Grab the size of the current example set.
    const int exampleSetSize = exampleSetSizes[exampleSetIdx];

    // Offset in the examples and clusterIndices array where we can find the first element associated to the current
    // example set.
    const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

    // Pointers to the actual examples and clustersIndices associated to the current example set.
    const ExampleType *exampleSetExamples = exampleSets + exampleSetOffset;
    const int *exampleSetClusterIndices = clusterIndices + exampleSetOffset;

    // Build the actual cluster by calling the createClusterFromExamples function that MUST be defined
    // for the current ExampleType.
    createClusterFromExamples(exampleSetExamples,
                              exampleSetClusterIndices,
                              exampleSetSize,
                              selectedClusterId,
                              currentClusterContainer.elts[outputClusterIdx]);
  }
}

/**
 * \brief Find the cluster index for each example in the example sets.
 *
 * \note  example_clusterer_link_neighbors split all the examples in subtrees and allocated a cluster index to the root
 *        of each subtree. With this function we navigate each example's subtree until we find the root and copy the
 *        cluster index. We also update a counter storing the size of each cluster.
 *
 * \param parents            A pointer to a memory area wherein is stored the parent for each example in the example sets.
 * \param clusterIndices     A pointer to a memory area wherein is stored the index of the cluster associated to each example.
 * \param clusterSizes       A pointer to a memory area wherein is stored the size of each cluster.
 * \param exampleSetCapacity The maximum size of each example set.
 * \param exampleSetIdx      The index of the current example set.
 * \param elementIdx         The index of the current element.
 */
_CPU_AND_GPU_CODE_
inline void example_clusterer_identify_clusters(const int *parents,
                                                int *clusterIndices,
                                                int *clusterSizes,
                                                int exampleSetCapacity,
                                                int exampleSetIdx,
                                                int elementIdx)
{
  // Linear offset to the first example (or associated data) of the example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;
  // Offset to the current element.
  const int elementOffset = exampleSetOffset + elementIdx;

  // Walk up on the tree until we find the root of the current subtree.

  // No need to check if the current element is valid
  // example_clusterer_link_neighbors sets the parent for invalid elements to themselves
  int parentIdx = parents[elementOffset];
  int currentIdx = elementIdx;

  while (parentIdx != currentIdx)
  {
    currentIdx = parentIdx;
    parentIdx = parents[exampleSetOffset + parentIdx];
  }

  // Found the root of the subtree, get its cluster index.
  const int clusterIdx = clusterIndices[exampleSetOffset + parentIdx];

  // Save the cluster index into the current element's cluster index variable.
  clusterIndices[elementOffset] = clusterIdx;

  // If it's a valid cluster then atomically increase its size (might be invalid if we started from an invalid example).
  if (clusterIdx >= 0)
  {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__) // Non templated function, need the __CUDA_ARCH__ check.
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
 * \brief This function allows the linking of neighboring examples in a "tree" structure.
 *
 * \note  Each example becomes part of a subtree where each element has as parent the closest example
 *        with higher density. For details, see the RQS paper by Fulkerson and Soatto.
 *        http://vision.ucla.edu/~brian/papers/fulkerson10really.pdf
 *
 * \param exampleSets             The examples to link in subtrees (rectangular array, one row per example set, one column per example).
 * \param exampleSetSizes         The actual size of each example set. One element per row in exampleSets.
 * \param densities               The densities associated to each example.
 * \param parents                 Output array where each element represents the parent of an example in the exampleSet.
 *                                Set to the element index itself if the example is the root of a subtree.
 * \param clusterIndices          Index of the cluster associated to each subtree.
 *                                Set to -1 unless the element is root of a subtree.
 * \param nbClustersPerExampleSet Will contain the number of subtrees in each example set.
 *                                Must be 0 when calling the function.
 * \param exampleSetCapacity      The maximum number of examples in an example set. Width of exampleSets.
 * \param exampleSetIdx           Index of the current example set.
 * \param elementIdx              Index of the element to process.
 * \param tauSq                   Maximum (squared) distance between examples to consider them linked together.
 */
template <typename ExampleType>
_CPU_AND_GPU_CODE_TEMPLATE_ inline void example_clusterer_link_neighbors(const ExampleType *exampleSets,
                                                                         const int *exampleSetSizes,
                                                                         const float *densities,
                                                                         int *parents,
                                                                         int *clusterIndices,
                                                                         int *nbClustersPerExampleSet,
                                                                         int exampleSetCapacity,
                                                                         int exampleSetIdx,
                                                                         int elementIdx,
                                                                         float tauSq)
{
  // Linear offset to the first element of the current example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;
  // Actual size of the current example set.
  const int exampleSetSize = exampleSetSizes[exampleSetIdx];
  // Linear offset of the current element wrt. the beginning of the exampleSets array.
  const int elementOffset = exampleSetOffset + elementIdx;

  // Unless it becomes part of a subtree, each element starts as its own parent.
  int parentIdx = elementIdx;
  // Index of the cluster associated to the current element.
  int clusterIdx = -1;

  // Proceed only if the current element is actually valid.
  if (elementIdx < exampleSetSize)
  {
    // Copy the current element in a temporary variable.
    const ExampleType centerExample = exampleSets[elementOffset];
    // Density of examples around the current element.
    const float centerDensity = densities[elementOffset];

    // We are only interested in examples closer to the current element than a tau distance.
    float minDistance = tauSq;

    // Check all the other examples.
    for (int i = 0; i < exampleSetSize; ++i)
    {
      // Ignore the element being processed.
      if (i == elementIdx) continue;

      // Grab a copy of the other example and its density.
      const ExampleType otherExample = exampleSets[exampleSetOffset + i];
      const float otherDensity = densities[exampleSetOffset + i];

      // Compute the squared distance between the current example and the other.
      // ExampleType MUST have a distanceSquared function defined for it.
      const float normSq = distanceSquared(centerExample, otherExample);

      // We are looking for the *closest* example with a higher density (doesn't matter how much) than the current
      // example's one.
      if (normSq < minDistance && centerDensity < otherDensity)
      {
        minDistance = normSq;
        parentIdx = i;
      }
    }

    // Current element is the root of a subtree (didn't find any close example with higher density than itself).
    // Grab a unique cluster index.
    if (parentIdx == elementIdx)
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

  // Store the parent of the current element (set to itself if the root of a subtree).
  parents[elementOffset] = parentIdx;
  // Store the cluster index (-1 unless the current element is the root of a subtree).
  clusterIndices[elementOffset] = clusterIdx;
}

/**
 * \brief Resets a cluster container.
 *
 * \param clusterContainers A pointer to the cluster containers.
 * \param exampleSetIdx     The index of the example set whose cluster container we want to reset.
 */
template <typename ClusterType, int MAX_CLUSTERS>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_clusterer_reset_cluster_container(Array<ClusterType,MAX_CLUSTERS> *clusterContainers, int exampleSetIdx)
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
 * \param clusterSizesHistogram   The histogram of cluster sizes.
 */
_CPU_AND_GPU_CODE_
inline void example_clusterer_reset_temporaries(int exampleSetIdx, int exampleSetCapacity, int *nbClustersPerExampleSet, int *clusterSizes, int *clusterSizesHistogram)
{
  // Reset the number of clusters extracted from this example set to zero.
  nbClustersPerExampleSet[exampleSetIdx] = 0;

  // Compute the memory offset to the beginning of any data associated with this example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;

  // Reset the cluster sizes and histogram values associated with the current example set.
  for(int i = 0; i < exampleSetCapacity; ++i)
  {
    clusterSizes[exampleSetOffset + i] = 0;
    clusterSizesHistogram[exampleSetOffset + i] = 0;
  }
}

/**
 * \brief Select the largest clusters from each example set.
 *
 * \param clusterSizes            A pointer to the sizes of each extracted cluster.
 * \param clusterSizesHistogram   A pointer to the histogram of cluster sizes for each example set.
 * \param nbClustersPerExampleSet A pointer to the number of clusters found in each example set.
 * \param selectedClusters        A pointer to the memory area where we will store the indices of the clusters selected
 *                                from each example set.
 * \param exampleSetCapacity      The mamimum size of each example set.
 * \param exampleSetIdx           The index of the current example set.
 * \param maxSelectedClusters     The maximum number of clusters to keep in each example set.
 * \param minClusterSize          The minimum size of a cluster to be kept.
 */
_CPU_AND_GPU_CODE_
inline void example_clusterer_select_clusters(const int *clusterSizes,
                                              const int *clusterSizesHistogram,
                                              const int *nbClustersPerExampleSet,
                                              int *selectedClusters,
                                              int exampleSetCapacity,
                                              int exampleSetIdx,
                                              int maxSelectedClusters,
                                              int minClusterSize)
{
  // Linear index to the first example (and associated data) of the current example set.
  const int exampleSetOffset = exampleSetIdx * exampleSetCapacity;
  // Number of clusters found in the current example set.
  const int nbValidClusters = nbClustersPerExampleSet[exampleSetIdx];
  // Linear offset to the current cluster in the output array.
  const int selectedClustersOffset = exampleSetIdx * maxSelectedClusters;

  // Reset output (set all cluster indices to invalid values).
  for (int i = 0; i < maxSelectedClusters; ++i)
  {
    selectedClusters[selectedClustersOffset + i] = -1;
  }

  // Scan the histogram from the top end to find the minimum cluster size we want to select.
  // We want up to maxSelectedClusters clusters.
  int nbSelectedClusters = 0;
  int selectedClusterSize = exampleSetCapacity - 1;

  for (; selectedClusterSize >= minClusterSize && nbSelectedClusters < maxSelectedClusters; --selectedClusterSize)
  {
    nbSelectedClusters += clusterSizesHistogram[exampleSetOffset + selectedClusterSize];
  }

  // If we couldn't find any cluster early out. Means that the current example set was empty.
  if (nbSelectedClusters == 0) return;

  // nbSelectedClusters might be greater than maxSelectedClusters if more clusters had the same size during the alst
  // check, need to keep this into account: at first add all clusters with size strictly greater than minClusterSize,
  // then perform another loop over the clusters add as many clusters with size equal to selectedClusterSize as possible

  nbSelectedClusters = 0;

  // First pass, strictly greater.
  for (int i = 0; i < nbValidClusters && nbSelectedClusters < maxSelectedClusters; ++i)
  {
    // If the current cluster size is greater than the threshold then keep it.
    if (clusterSizes[exampleSetOffset + i] > selectedClusterSize)
    {
      selectedClusters[selectedClustersOffset + nbSelectedClusters++] = i;
    }
  }

  // Second pass, equal, keep as many clusters as possible.
  for (int i = 0; i < nbValidClusters && nbSelectedClusters < maxSelectedClusters; ++i)
  {
    if (clusterSizes[exampleSetOffset + i] == selectedClusterSize)
    {
      selectedClusters[selectedClustersOffset + nbSelectedClusters++] = i;
    }
  }

  // Sort clusters by descending number of inliers.
  // Note: this implementation is quadratic but the number of clusters is small enough to not care for now.
  for (int i = 0; i < nbSelectedClusters; ++i)
  {
    int maxSize = clusterSizes[exampleSetOffset + selectedClusters[selectedClustersOffset + i]];
    int maxIdx = i;

    for (int j = i + 1; j < nbSelectedClusters; ++j)
    {
      int size = clusterSizes[exampleSetOffset + selectedClusters[selectedClustersOffset + j]];
      if (size > maxSize)
      {
        maxSize = size;
        maxIdx = j;
      }
    }

    // Swap.
    if (maxIdx != i)
    {
      int temp = selectedClusters[selectedClustersOffset + i];
      selectedClusters[selectedClustersOffset + i] = selectedClusters[selectedClustersOffset + maxIdx];
      selectedClusters[selectedClustersOffset + maxIdx] = temp;
    }
  }
}

}

#endif
