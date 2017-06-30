/**
 * grove: ExampleClusterer_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ExampleClusterer_CPU.h"

#include "../shared/ExampleClusterer_Shared.h"

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::ExampleClusterer_CPU(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize)
: ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>(sigma, tau, maxClusterCount, minClusterSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::compute_cluster_parameters(const ExampleType *examples, const int *exampleSetSizes,
                                                                                            Clusters *clustersData, uint32_t maxClusterCount,
                                                                                            uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *selectedClusters = this->m_selectedClusters->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int exampleSetIdx = 0; exampleSetIdx < static_cast<int>(exampleSetCount); ++exampleSetIdx)
  {
    for(uint32_t clusterIdx = 0; clusterIdx < maxClusterCount; ++clusterIdx)
    {
      compute_modes(
        examples, exampleSetSizes, clusterIndices, selectedClusters, clustersData,
        exampleSetCapacity, exampleSetIdx, maxClusterCount, clusterIdx
      );
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::compute_cluster_size_histograms(uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    for(uint32_t clusterIdx = 0; clusterIdx < exampleSetCapacity; ++clusterIdx)
    {
      compute_cluster_histogram(clusterSizes, nbClustersPerExampleSet, clusterSizesHistogram, exampleSetCapacity, setIdx, clusterIdx);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::compute_clusters(uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *parents = this->m_parents->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int exampleSetIdx = 0; exampleSetIdx < static_cast<int>(exampleSetCount); ++exampleSetIdx)
  {
    for(uint32_t exampleIdx = 0; exampleIdx < exampleSetCapacity; ++exampleIdx)
    {
      compute_cluster(exampleSetIdx, exampleIdx, exampleSetCapacity, parents, clusterIndices, clusterSizes);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::compute_densities(const ExampleType *examples, const int *exampleSetSizes, uint32_t exampleSetCapacity,
                                                                                   uint32_t exampleSetCount, float sigma)
{
  float *densities = this->m_densities->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int exampleSetIdx = 0; exampleSetIdx < static_cast<int>(exampleSetCount); ++exampleSetIdx)
  {
    for(uint32_t exampleIdx = 0; exampleIdx < exampleSetCapacity; ++exampleIdx)
    {
      compute_density(exampleSetIdx, exampleIdx, examples, exampleSetSizes, exampleSetCapacity, sigma, densities);
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::compute_parents(const ExampleType *exampleSets, const int *exampleSetSizes, uint32_t exampleSetCapacity,
                                                                                 uint32_t exampleSetCount, float tauSq)
{
  const float *densities = this->m_densities->GetData(MEMORYDEVICE_CPU);

  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *parents = this->m_parents->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int exampleSetIdx = 0; exampleSetIdx < static_cast<int>(exampleSetCount); ++exampleSetIdx)
  {
    for(uint32_t exampleIdx = 0; exampleIdx < exampleSetCapacity; ++exampleIdx)
    {
      compute_parent(
        exampleSetIdx, exampleIdx, exampleSets, exampleSetCapacity, exampleSetSizes,
        densities, tauSq, parents, clusterIndices, nbClustersPerExampleSet
      );
    }
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
typename ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::Clusters *
ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::get_pointer_to_cluster(const ClustersBlock_Ptr &clusters, uint32_t clusterIdx) const
{
  return clusters->GetData(MEMORYDEVICE_CPU) + clusterIdx;
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
const ExampleType *ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::get_pointer_to_example_set(const ExampleImage_CPtr& exampleSets, uint32_t setIdx) const
{
  const int setCapacity = exampleSets->noDims.width;
  return exampleSets->GetData(MEMORYDEVICE_CPU) + setIdx * setCapacity;
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
const int *ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr& exampleSetSizes, uint32_t setIdx) const
{
  return exampleSetSizes->GetData(MEMORYDEVICE_CPU) + setIdx;
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::reset_clusters(Clusters *clustersData, uint32_t exampleSetCount) const
{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int exampleSetIdx = 0; exampleSetIdx < static_cast<int>(exampleSetCount); ++exampleSetIdx)
  {
    reset_cluster_container(clustersData, exampleSetIdx);
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::reset_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int setIdx = 0; setIdx < static_cast<int>(exampleSetCount); ++setIdx)
  {
    reset_temporaries_for_set(setIdx, exampleSetCapacity, nbClustersPerExampleSet, clusterSizes, clusterSizesHistogram);
  }
}

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer_CPU<ExampleType,ClusterType,MAX_CLUSTERS>::select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize,
                                                                                 uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);
  int *nbClustersPerExampleSet = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *selectedClusters = this->m_selectedClusters->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int exampleSetIdx = 0; exampleSetIdx < static_cast<int>(exampleSetCount); ++exampleSetIdx)
  {
    select_clusters_for_set(
      clusterSizes, clusterSizesHistogram, nbClustersPerExampleSet, selectedClusters,
      exampleSetCapacity, exampleSetIdx, maxClusterCount, minClusterSize
    );
  }
}

}
