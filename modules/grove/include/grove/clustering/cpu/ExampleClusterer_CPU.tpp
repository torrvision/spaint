/**
 * grove: ExampleClusterer_CPU.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "ExampleClusterer_CPU.h"

#include <iostream>

#include "../shared/ExampleClusterer_Shared.h"

namespace grove {

template <typename ExampleType, typename ClusterType>
ExampleClusterer_CPU<ExampleType, ClusterType>::ExampleClusterer_CPU(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize)
  : ExampleClusterer<ExampleType, ClusterType>(sigma, tau, maxClusterCount, minClusterSize)
{}

template<typename ExampleType, typename ClusterType>
const ExampleType *ExampleClusterer_CPU<ExampleType, ClusterType>::get_pointer_to_example_set(const ExampleImage_CPtr &exampleReservoirs, uint32_t reservoirIdx) const
{
  const int reservoirCapacity = exampleReservoirs->noDims.width;
  return exampleReservoirs->GetData(MEMORYDEVICE_CPU) + reservoirIdx * reservoirCapacity;
}

template<typename ExampleType, typename ClusterType>
const int *ExampleClusterer_CPU<ExampleType, ClusterType>::get_pointer_to_example_set_size(const ITMIntMemoryBlock_CPtr &exampleReservoirsSize, uint32_t reservoirIdx) const
{
  return exampleReservoirsSize->GetData(MEMORYDEVICE_CPU) + reservoirIdx;
}

template<typename ExampleType, typename ClusterType>
ClusterType *ExampleClusterer_CPU<ExampleType, ClusterType>::get_pointer_to_prediction(const ClusterBlock_Ptr &predictions, uint32_t predictionIdx) const
{
  return predictions->GetData(MEMORYDEVICE_CPU) + predictionIdx;
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::reset_temporaries(uint32_t reservoirCapacity, uint32_t reservoirCount)
{
  int *nbClustersPerReservoir = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    example_clusterer_reset_temporaries(nbClustersPerReservoir, clusterSizes, clusterSizesHistogram,
                                        reservoirIdx, reservoirCapacity);
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::compute_cluster_parameters(const ExampleType *examples, const int *reservoirSizes, ClusterType *predictionsData, uint32_t maxClusterCount, uint32_t reservoirCapacity, uint32_t reservoirCount)
{
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *selectedClusters = this->m_selectedClusters->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    for(uint32_t clusterIdx = 0; clusterIdx < maxClusterCount; ++clusterIdx)
    {
      example_clusterer_compute_modes(examples, reservoirSizes, clusterIndices,
                                      selectedClusters, predictionsData, reservoirCapacity,
                                      maxClusterCount, reservoirIdx, clusterIdx);
    }
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::select_clusters(uint32_t maxClusterCount, uint32_t minClusterSize, uint32_t reservoirCapacity, uint32_t reservoirCount)
{
  int *nbClustersPerReservoir = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);
  int *selectedClusters = this->m_selectedClusters->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    example_clusterer_select_clusters(clusterSizes, clusterSizesHistogram, nbClustersPerReservoir,
                                      selectedClusters, reservoirCapacity, maxClusterCount,
                                      minClusterSize, reservoirIdx);
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::compute_cluster_size_histograms(uint32_t reservoirCapacity, uint32_t reservoirCount)
{
  int *nbClustersPerReservoir = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *clusterSizesHistogram = this->m_clusterSizesHistogram->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    for(uint32_t clusterIdx = 0; clusterIdx < reservoirCapacity; ++clusterIdx)
    {
      example_clusterer_compute_cluster_histogram(clusterSizes, nbClustersPerReservoir, clusterSizesHistogram,
                                                  reservoirCapacity, reservoirIdx, clusterIdx);
    }
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::identify_clusters(uint32_t reservoirCapacity, uint32_t reservoirCount)
{
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);
  int *clusterSizes = this->m_clusterSizes->GetData(MEMORYDEVICE_CPU);
  int *parents = this->m_parents->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    for(uint32_t elementIdx = 0; elementIdx < reservoirCapacity; ++elementIdx)
    {
  example_clusterer_identify_clusters(parents, clusterIndices, clusterSizes,
                                      reservoirCapacity, reservoirIdx, elementIdx);
    }
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::link_neighbors(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float tauSq)
{
  const float *densities = this->m_densities->GetData(MEMORYDEVICE_CPU);
  int *nbClustersPerReservoir = this->m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU);
  int *parents = this->m_parents->GetData(MEMORYDEVICE_CPU);
  int *clusterIndices = this->m_clusterIdx->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    for(uint32_t elementIdx = 0; elementIdx < reservoirCapacity; ++elementIdx)
    {
      example_clusterer_link_neighbors(examples, reservoirSizes, densities, parents,
                                       clusterIndices, nbClustersPerReservoir, reservoirCapacity,
                                       reservoirIdx, elementIdx, tauSq);
    }
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::compute_density(const ExampleType *examples, const int *reservoirSizes, uint32_t reservoirCapacity, uint32_t reservoirCount, float sigma)
{
  float *densities = this->m_densities->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int reservoirIdx = 0; reservoirIdx < static_cast<int>(reservoirCount); ++reservoirIdx)
  {
    for(uint32_t elementIdx = 0; elementIdx < reservoirCapacity; ++elementIdx)
    {
      example_clusterer_compute_density(examples, reservoirSizes, densities,
                                        reservoirCapacity, reservoirIdx, elementIdx, sigma);
    }
  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer_CPU<ExampleType, ClusterType>::reset_predictions(ClusterType *predictionsData, uint32_t reservoirCount) const
{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for(int predictionIdx = 0; predictionIdx < static_cast<int>(reservoirCount); ++predictionIdx)
  {
    example_clusterer_reset_predictions(predictionsData, predictionIdx);
  }
}

}
