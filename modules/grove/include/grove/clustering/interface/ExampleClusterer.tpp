/**
 * grove: ExampleClusterer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleClusterer.h"

#include <iostream>

#include <itmx/MemoryBlockFactory.h>
using namespace itmx;

namespace grove {

template <typename ExampleType, typename ClusterType>
ExampleClusterer<ExampleType, ClusterType>::ExampleClusterer(float sigma,
                                                             float tau,
                                                             uint32_t maxClusterCount,
                                                             uint32_t minClusterSize)
  : m_maxClusterCount(maxClusterCount), m_minClusterSize(minClusterSize), m_sigma(sigma), m_tau(tau)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  // Everything is empty for now, we'll resize them as soon as we know the actual sizes.
  m_densities = mbf.make_image<float>();
  m_parents = mbf.make_image<int>();
  m_clusterIdx = mbf.make_image<int>();
  m_clusterSizes = mbf.make_image<int>();
  m_clusterSizesHistogram = mbf.make_image<int>();
  m_selectedClusters = mbf.make_image<int>();
  m_nbClustersPerExampleSet = mbf.make_block<int>();
}

template <typename ExampleType, typename ClusterType>
ExampleClusterer<ExampleType, ClusterType>::~ExampleClusterer()
{
}

template <typename ExampleType, typename ClusterType>
void ExampleClusterer<ExampleType, ClusterType>::find_modes(const ExampleImage_CPtr &exampleSets,
                                                            const ITMIntMemoryBlock_CPtr &exampleSetsSize,
                                                            ClusterBlock_Ptr &clusterContainers,
                                                            uint32_t startIdx,
                                                            uint32_t count)
{
  const uint32_t nbExampleSets = exampleSets->noDims.height;
  const uint32_t exampleSetCapacity = exampleSets->noDims.width;

  if (startIdx + count > nbExampleSets) throw std::invalid_argument("startIdx + count > nbExampleSets");

  // Allocate local variables (a NOP except during the first run, smaller memory requirements do not cause
  // reallocations).
  allocate_temporaries(exampleSetCapacity, count);

  // Grab a pointer to the beginning of the first exampleSet of interest (having index startIdx).
  const ExampleType *exampleSetsData = get_pointer_to_example_set(exampleSets, startIdx);
  // Grab a pointer to the size of the first exampleSet of interest.
  const int *exampleSetsSizeData = get_pointer_to_example_set_size(exampleSetsSize, startIdx);
  // Grab a pointer to the first prediction to compute.
  ClusterType *predictionsData = get_pointer_to_prediction(clusterContainers, startIdx);

  // Reset working variables.
  reset_temporaries(exampleSetCapacity, count);

  // Reset output predictions.
  reset_predictions(predictionsData, count);

  // Compute densities.
  compute_density(exampleSetsData, exampleSetsSizeData, exampleSetCapacity, count, m_sigma);

  // Link neighbors.
  link_neighbors(exampleSetsData, exampleSetsSizeData, exampleSetCapacity, count, m_tau * m_tau);

  // Identify clusters.
  identify_clusters(exampleSetCapacity, count);

  // Compute cluster size histograms.
  compute_cluster_size_histograms(exampleSetCapacity, count);

  // Select best clusters.
  select_clusters(m_maxClusterCount, m_minClusterSize, exampleSetCapacity, count);

  // Finally, compute modes for each cluster and store the predictions.
  compute_modes(exampleSetsData, exampleSetsSizeData, predictionsData, m_maxClusterCount, exampleSetCapacity, count);

// Debug.
#if 0
  m_nbClustersPerExampleSet->UpdateHostFromDevice();
  m_clusterSizes->UpdateHostFromDevice();
  exampleSetsSize->UpdateHostFromDevice();

  for (uint32_t i = 0; i < count; ++i)
  {
    std::cout << "Example set " << i + startIdx << " has "
              << m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU)[i + startIdx] << " clusters and "
              << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[i + startIdx] << " elements.\n";

    for (int j = 0; j < m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU)[i + startIdx]; ++j)
    {
      std::cout << "\tCluster " << j << ": "
                << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[(i + startIdx) * exampleSetCapacity + j] << " elements.\n";
    }
  }
#endif
}

template <typename ExampleType, typename ClusterType>
void ExampleClusterer<ExampleType, ClusterType>::allocate_temporaries(uint32_t exampleSetCapacity,
                                                                      uint32_t exampleSetCount)
{
  const Vector2i temporariesSize(static_cast<int>(exampleSetCapacity), static_cast<int>(exampleSetCount));

  // We perform the check instead of relying on ChangeDims performing a NOP because reservoirCount
  // might be smaller than the current size and we want to avoid reallocating the blocks.
  if (m_densities->noDims.x < temporariesSize.x || m_densities->noDims.y < temporariesSize.y)
  {
    // Happens only once
    m_densities->ChangeDims(temporariesSize);
    m_parents->ChangeDims(temporariesSize);
    m_clusterIdx->ChangeDims(temporariesSize);
    m_clusterSizes->ChangeDims(temporariesSize);
    m_clusterSizesHistogram->ChangeDims(temporariesSize);

    m_selectedClusters->ChangeDims(Vector2i(m_maxClusterCount, exampleSetCount));
    m_nbClustersPerExampleSet->Resize(exampleSetCount);
  }
}

} // namespace grove
