/**
 * grove: ExampleClusterer_Shared.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERER_SHARED
#define H_GROVE_EXAMPLECLUSTERER_SHARED

#include <ORUtils/PlatformIndependence.h>

namespace grove {

_CPU_AND_GPU_CODE_
inline void example_clusterer_reset_temporaries(
    int *clustersPerReservoir,
    int *clusterSizes,
    int *clusterSizesHistogram,
    int reservoirIdx,
    int reservoirCapacity)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;

  // Reset number of clusters per reservoir
  clustersPerReservoir[reservoirIdx] = 0;

  // Reset cluster sizes and histogram
  for (int i = 0; i < reservoirCapacity; ++i)
  {
    clusterSizes[reservoirOffset + i] = 0;
    clusterSizesHistogram[reservoirOffset + i] = 0;
  }
}

template <typename ExampleType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_clusterer_compute_density(const ExampleType *examples,
    const int *reservoirSizes, float *densities, int reservoirCapacity,
    int reservoirIdx, int elementIdx, float sigma)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int reservoirSize = reservoirSizes[reservoirIdx];
  const int elementOffset = reservoirOffset + elementIdx;

  // Points farther away have small contribution to the density.
  const float threeSigmaSq = (3.f * sigma) * (3.f * sigma);
  const float minusOneOverTwoSigmaSq = -1.f / (2.f * sigma * sigma);

  float density = 0.f;

  if (elementIdx < reservoirSize)
  {
    const Vector3f centerPosition(examples[elementOffset].position);

    for (int i = 0; i < reservoirSize; ++i)
    {
      const Vector3f examplePosition(examples[reservoirOffset + i].position);
      const Vector3f diff = examplePosition - centerPosition;
      const float normSq = dot(diff, diff);

      if (normSq < threeSigmaSq)
      {
        density += expf(normSq * minusOneOverTwoSigmaSq);
      }
    }
  }

  densities[elementOffset] = density;
}

template <typename ExampleType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_clusterer_link_neighbors(const ExampleType *examples,
    const int *reservoirSizes, const float *densities, int *parents,
    int *clusterIndices, int *nbClustersPerReservoir, int reservoirCapacity,
    int reservoirIdx, int elementIdx, float tauSq)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int reservoirSize = reservoirSizes[reservoirIdx];
  const int elementOffset = reservoirOffset + elementIdx;

  int parentIdx = elementIdx;
  int clusterIdx = -1;

  if (elementIdx < reservoirSize)
  {
    const Vector3f centerPosition(examples[elementOffset].position);
    const float centerDensity = densities[elementOffset];
    float minDistance = tauSq;

    for (int i = 0; i < reservoirSize; ++i)
    {
      if (i == elementIdx)
        continue;

      const Vector3f examplePosition(examples[reservoirOffset + i].position);
      const float exampleDensity = densities[reservoirOffset + i];

      const Vector3f diff = examplePosition - centerPosition;
      const float normSq = dot(diff, diff);

      if (normSq < minDistance && centerDensity < exampleDensity)
      {
        minDistance = normSq;
        parentIdx = i;
      }
    }

    // current element is the root of a subtree, get a unique cluster index
    if (parentIdx == elementIdx)
    {
#ifdef __CUDACC__
      clusterIdx = atomicAdd(&nbClustersPerReservoir[reservoirIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
      clusterIdx = nbClustersPerReservoir[reservoirIdx]++;
#endif
    }
  }

  parents[elementOffset] = parentIdx;
  clusterIndices[elementOffset] = clusterIdx;
}

_CPU_AND_GPU_CODE_
inline void example_clusterer_identify_clusters(const int *parents, int *clusterIndices, int *clusterSizes,
    int reservoirCapacity, int reservoirIdx, int elementIdx)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int elementOffset = reservoirOffset + elementIdx;

  // No need to check if the current element is valid
  // ck_link_neighbors sets the parent for invalid elements to themselves
  int parentIdx = parents[elementOffset];
  int currentIdx = elementIdx;
  while (parentIdx != currentIdx)
  {
    currentIdx = parentIdx;
    parentIdx = parents[reservoirOffset + parentIdx];
  }

  // found the root of the subtree, get its cluster idx
  const int clusterIdx = clusterIndices[reservoirOffset + parentIdx];
  clusterIndices[elementOffset] = clusterIdx;

  // If it's a valid cluster then increase its size
  if (clusterIdx >= 0)
  {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__) // Non templated function, need the __CUDA_ARCH__ check.
    atomicAdd(&clusterSizes[reservoirOffset + clusterIdx], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic
#endif
    clusterSizes[reservoirOffset + clusterIdx]++;
#endif
  }
}

_CPU_AND_GPU_CODE_
inline void example_clusterer_compute_cluster_histogram(const int *clusterSizes,
    const int *nbClustersPerReservoir, int *clusterSizesHistogram,
    int reservoirCapacity, int reservoirIdx, int clusterIdx)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int validClusters = nbClustersPerReservoir[reservoirIdx];

  if (clusterIdx >= validClusters)
    return;

  const int clusterSize = clusterSizes[reservoirOffset + clusterIdx];

#if defined(__CUDACC__) && defined(__CUDA_ARCH__) // Non templated function, need the __CUDA_ARCH__ check.
  atomicAdd(&clusterSizesHistogram[reservoirOffset + clusterSize], 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic
#endif
  clusterSizesHistogram[reservoirOffset + clusterSize]++;
#endif
}

_CPU_AND_GPU_CODE_
inline void example_clusterer_select_clusters(const int *clusterSizes,
    const int *clusterSizesHistogram, const int *nbClustersPerReservoir,
    int *selectedClusters, int reservoirCapacity, int maxSelectedClusters, int minClusterSize, int reservoirIdx)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int validClusters = nbClustersPerReservoir[reservoirIdx];
  const int selectedClustersOffset = reservoirIdx * maxSelectedClusters;

  // Reset output
  for (int i = 0; i < maxSelectedClusters; ++i)
  {
    selectedClusters[selectedClustersOffset + i] = -1;
  }

  // Scan the histogram from the top to find the minimum cluster size we want to select
  int nbSelectedClusters = 0;
  int selectedClusterSize = reservoirCapacity - 1;
  for (; selectedClusterSize >= minClusterSize && nbSelectedClusters < maxSelectedClusters; --selectedClusterSize)
  {
    nbSelectedClusters += clusterSizesHistogram[reservoirOffset
        + selectedClusterSize];
  }

  // Empty reservoir
  if (nbSelectedClusters == 0)
    return;

  // nbSelectedClusters might be greater than maxSelectedClusters if more clusters had the same size,
  // need to keep this into account: at first add all clusters with size greater than minClusterSize
  // then another loop over the clusters add as many clusters with size == selectedClusterSize as possible

  nbSelectedClusters = 0;

  // first loop, >
  for (int i = 0; i < validClusters && nbSelectedClusters < maxSelectedClusters; ++i)
  {
    if (clusterSizes[reservoirOffset + i] > selectedClusterSize)
    {
      selectedClusters[selectedClustersOffset + nbSelectedClusters++] = i;
    }
  }

  // second loop, ==
  for (int i = 0; i < validClusters && nbSelectedClusters < maxSelectedClusters;
      ++i)
  {
    if (clusterSizes[reservoirOffset + i] == selectedClusterSize)
    {
      selectedClusters[selectedClustersOffset + nbSelectedClusters++] = i;
    }
  }

  // Sort clusters by descending number of inliers
  // Quadratic but small enough to not care for now
  for (int i = 0; i < nbSelectedClusters; ++i)
  {
    int maxSize = clusterSizes[reservoirOffset
        + selectedClusters[selectedClustersOffset + i]];
    int maxIdx = i;

    for (int j = i + 1; j < nbSelectedClusters; ++j)
    {
      int size = clusterSizes[reservoirOffset
          + selectedClusters[selectedClustersOffset + j]];
      if (size > maxSize)
      {
        maxSize = size;
        maxIdx = j;
      }
    }

    // Swap
    if (maxIdx != i)
    {
      int temp = selectedClusters[selectedClustersOffset + i];
      selectedClusters[selectedClustersOffset + i] =
          selectedClusters[selectedClustersOffset + maxIdx];
      selectedClusters[selectedClustersOffset + maxIdx] = temp;
    }
  }
}

// TODO: this will need specialization for 2D keypoints without colour.
template <typename ExampleType, typename ClusterType>
_CPU_AND_GPU_CODE_TEMPLATE_
inline void example_clusterer_compute_modes(const ExampleType *examples,
    const int *reservoirSizes, const int *clusterIndices,
    const int *selectedClusters, ClusterType *predictions,
    int reservoirCapacity, int maxSelectedClusters, int reservoirIdx, int clusterIdx)
{
  const int reservoirOffset = reservoirIdx * reservoirCapacity;
  const int reservoirSize = reservoirSizes[reservoirIdx];
  const int selectedClustersOffset = reservoirIdx * maxSelectedClusters;

  ClusterType &reservoirPrediction = predictions[reservoirIdx];

  if (clusterIdx == 0)
    reservoirPrediction.nbModes = 0;

#ifdef __CUDACC__
  __syncthreads();
#else
#ifdef WITH_OPENMP
#pragma omp barrier
#endif
#endif

  const int selectedClusterId = selectedClusters[selectedClustersOffset
      + clusterIdx];
  if (selectedClusterId >= 0)
  {
    // compute position and colour mean
    int sampleCount = 0;
    Vector3f positionMean(0.f);
    Vector3f colourMean(0.f);

    // Iterate over all examples and use only those belonging to selectedClusterId
    for (int sampleIdx = 0; sampleIdx < reservoirSize; ++sampleIdx)
    {
      const int sampleCluster = clusterIndices[reservoirOffset + sampleIdx];
      if (sampleCluster == selectedClusterId)
      {
        const ExampleType &sample = examples[reservoirOffset + sampleIdx];

        ++sampleCount;
        positionMean += sample.position;
        colourMean += sample.colour.toFloat();
      }
    }

    //this mode is invalid..
    if (sampleCount <= 1) // Should never reach this point since we check minClusterSize earlier
      return;

    positionMean /= static_cast<float>(sampleCount);
    colourMean /= static_cast<float>(sampleCount);

    // Now iterate again and compute the covariance
    Matrix3f positionCovariance;
    positionCovariance.setZeros();

    for (int sampleIdx = 0; sampleIdx < reservoirSize; ++sampleIdx)
    {
      const int sampleCluster = clusterIndices[reservoirOffset + sampleIdx];
      if (sampleCluster == selectedClusterId)
      {
        const ExampleType &sample = examples[reservoirOffset + sampleIdx];

        for (int i = 0; i < 3; ++i)
        {
          for (int j = 0; j < 3; ++j)
          {
            positionCovariance.m[i * 3 + j] += (sample.position.v[i]
                - positionMean.v[i])
                * (sample.position.v[j] - positionMean.v[j]);
          }
        }
      }
    }

    positionCovariance /= static_cast<float>(sampleCount - 1);
    const float positionDeterminant = positionCovariance.det();

    // Get the mode idx
    int modeIdx = -1;

#ifdef __CUDACC__
    modeIdx = atomicAdd(&reservoirPrediction.nbModes, 1);
#else
#ifdef WITH_OPENMP
#pragma omp atomic capture
#endif
    modeIdx = reservoirPrediction.nbModes++;
#endif

    // Fill the mode
    // This does not compile without an additional template parameter
    // ScoreMode &outMode = reservoirPrediction.modes[modeIdx];
    reservoirPrediction.modes[modeIdx].nbInliers = sampleCount;
    reservoirPrediction.modes[modeIdx].position = positionMean;
    reservoirPrediction.modes[modeIdx].determinant = positionDeterminant;
    positionCovariance.inv(reservoirPrediction.modes[modeIdx].positionInvCovariance);
    reservoirPrediction.modes[modeIdx].colour = colourMean.toUChar();
  }
}

}

#endif
