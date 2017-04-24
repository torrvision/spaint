/**
 * grove: ExampleClusterer_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ExampleClusterer_CPU.h"

#include <iostream>

#include "../shared/ExampleClusterer_Shared.h"

namespace grove {

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleClusterer_CPU(float sigma,
                                                                                   float tau,
                                                                                   uint32_t maxClusterCount,
                                                                                   uint32_t minClusterSize)
  : ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>(sigma, tau, maxClusterCount, minClusterSize)
{
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
const ExampleType *ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::get_pointer_to_example_set(
    const ExampleImage_CPtr &exampleSets, uint32_t setIdx) const
{
  const int setCapacity = exampleSets->noDims.width;
  return exampleSets->GetData(MEMORYDEVICE_CPU) + setIdx * setCapacity;
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
const int *ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::get_pointer_to_example_set_size(
    const ITMIntMemoryBlock_CPtr &exampleSetsSize, uint32_t setIdx) const
{
  return exampleSetsSize->GetData(MEMORYDEVICE_CPU) + setIdx;
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
typename ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::Clusters *
    ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::get_pointer_to_cluster(
        const ClustersBlock_Ptr &clusters, uint32_t clusterIdx) const
{
  return clusters->GetData(MEMORYDEVICE_CPU) + clusterIdx;
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::reset_temporaries(uint32_t exampleSetCapacity,
                                                                                     uint32_t exampleSetCount)
{
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    example_clusterer_reset_temporaries(
        nbClustersPerExampleSet, clusterSizes, clusterSizesHistogram, setIdx, exampleSetCapacity);
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::compute_cluster_parameters(
    const ExampleType *examples,
    const int *exampleSetSizes,
    Clusters *clustersData,
    uint32_t maxClusterCount,
    uint32_t exampleSetCapacity,
    uint32_t exampleSetCount)
{
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *selectedClusters = this->m_selectedClusters->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    for (uint32_t clusterIdx = 0; clusterIdx < maxClusterCount; ++clusterIdx)
    {
      example_clusterer_compute_modes(examples,
                                      exampleSetSizes,
                                      clusterIndices,
                                      selectedClusters,
                                      clustersData,
                                      exampleSetCapacity,
                                      maxClusterCount,
                                      setIdx,
                                      clusterIdx);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::select_clusters(uint32_t maxClusterCount,
                                                                                   uint32_t minClusterSize,
                                                                                   uint32_t exampleSetCapacity,
                                                                                   uint32_t exampleSetCount)
{
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);
  int *selectedClusters = this->m_selectedClusters->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    example_clusterer_select_clusters(clusterSizes,
                                      clusterSizesHistogram,
                                      nbClustersPerExampleSet,
                                      selectedClusters,
                                      exampleSetCapacity,
                                      maxClusterCount,
                                      minClusterSize,
                                      setIdx);
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::compute_cluster_size_histograms(
    uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    for (uint32_t clusterIdx = 0; clusterIdx < exampleSetCapacity; ++clusterIdx)
    {
      example_clusterer_compute_cluster_histogram(
          clusterSizes, nbClustersPerExampleSet, clusterSizesHistogram, exampleSetCapacity, setIdx, clusterIdx);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::identify_clusters(uint32_t exampleSetCapacity,
                                                                                     uint32_t exampleSetCount)
{
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *parents = this->m_parents->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    for (uint32_t elementIdx = 0; elementIdx < exampleSetCapacity; ++elementIdx)
    {
      example_clusterer_identify_clusters(
          parents, clusterIndices, clusterSizes, exampleSetCapacity, setIdx, elementIdx);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::link_neighbors(const ExampleType *examples,
                                                                                  const int *exampleSetSizes,
                                                                                  uint32_t exampleSetCapacity,
                                                                                  uint32_t exampleSetCount,
                                                                                  float tauSq)
{
  const float *densities = this->m_densities->GetData(MEMORYDEVICE_CPU);
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *parents = this->m_parents->GetData(MEMORYDEVICE_CPU);
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    for (uint32_t elementIdx = 0; elementIdx < exampleSetCapacity; ++elementIdx)
    {
      example_clusterer_link_neighbors(examples,
                                       exampleSetSizes,
                                       densities,
                                       parents,
                                       clusterIndices,
                                       nbClustersPerExampleSet,
                                       exampleSetCapacity,
                                       setIdx,
                                       elementIdx,
                                       tauSq);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::compute_density(const ExampleType *examples,
                                                                                   const int *exampleSetSizes,
                                                                                   uint32_t exampleSetCapacity,
                                                                                   uint32_t exampleSetCount,
                                                                                   float sigma)
{
  float *densities = this->m_densities->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    for (uint32_t elementIdx = 0; elementIdx < exampleSetCapacity; ++elementIdx)
    {
      example_clusterer_compute_density(
          examples, exampleSetSizes, densities, exampleSetCapacity, setIdx, elementIdx, sigma);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType, ClusterType, MAX_CLUSTERS>::reset_clusters(Clusters *clustersData,
                                                                                  uint32_t clustersCount) const
{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int predictionIdx = 0; predictionIdx < static_cast<int>(clustersCount); ++predictionIdx)
  {
    example_clusterer_reset_cluster_container(clustersData, predictionIdx);
  }
}
}
