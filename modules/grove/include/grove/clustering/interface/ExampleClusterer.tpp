/**
 * grove: ExampleClusterer.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleClusterer.h"

#include <iostream>

#include <itmx/base/MemoryBlockFactory.h>

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS>::ExampleClusterer(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize)
: m_maxClusterCount(maxClusterCount), m_minClusterSize(minClusterSize), m_sigma(sigma), m_tau(tau)
{
  if(maxClusterCount > MAX_CLUSTERS)
  {
    throw std::invalid_argument("Error: maxClusterCount > MAX_CLUSTERS");
  }

  itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();

  // Initially, everything is empty. We resize things as soon as we know the actual sizes.
  m_clusterIdx = mbf.make_image<int>();
  m_clusterSizes = mbf.make_image<int>();
  m_clusterSizesHistogram = mbf.make_image<int>();
  m_densities = mbf.make_image<float>();
  m_nbClustersPerExampleSet = mbf.make_block<int>();
  m_parents = mbf.make_image<int>();
  m_selectedClusters = mbf.make_image<int>();
}

//#################### DESTRUCTOR ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::~ExampleClusterer()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::find_modes(const ExampleImage_CPtr& exampleSets, const ITMIntMemoryBlock_CPtr& exampleSetsSize,
                                                                        ClustersBlock_Ptr& clusterContainers, uint32_t startIdx, uint32_t count)
{
  const uint32_t nbExampleSets = exampleSets->noDims.height;
  const uint32_t exampleSetCapacity = exampleSets->noDims.width;

  if(startIdx + count > nbExampleSets) throw std::invalid_argument("startIdx + count > nbExampleSets");

  // Allocate local variables (a NOP except during the first run, smaller memory requirements do not cause
  // reallocations).
  allocate_temporaries(exampleSetCapacity, count);

  // Grab a pointer to the beginning of the first exampleSet of interest.
  const ExampleType *exampleSetsData = get_pointer_to_example_set(exampleSets, startIdx);
  // Grab a pointer to the size of the first exampleSet of interest.
  const int *exampleSetsSizeData = get_pointer_to_example_set_size(exampleSetsSize, startIdx);
  // Grab a pointer to the first cluster to compute.
  Clusters *clustersData = get_pointer_to_cluster(clusterContainers, startIdx);

  // Reset working variables.
  reset_temporaries(exampleSetCapacity, count);

  // Reset output predictions.
  reset_clusters(clustersData, count);

  // Compute densities.
  compute_density(exampleSetsData, exampleSetsSizeData, exampleSetCapacity, count, m_sigma);

  // Link neighbors in a tree structure and cut the branches according to the maximum distance tau.
  link_neighbors(exampleSetsData, exampleSetsSizeData, exampleSetCapacity, count, m_tau * m_tau);

  // Identify clusters (assign identifiers to the clusters).
  identify_clusters(exampleSetCapacity, count);

  // Compute cluster size histograms (used to select the largest clusters).
  compute_cluster_size_histograms(exampleSetCapacity, count);

  // Select largest clusters.
  select_clusters(m_maxClusterCount, m_minClusterSize, exampleSetCapacity, count);

  // Finally, compute parameters for each cluster and store the predictions.
  compute_cluster_parameters(exampleSetsData, exampleSetsSizeData, clustersData, m_maxClusterCount, exampleSetCapacity, count);

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

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::allocate_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  const Vector2i temporariesSize(static_cast<int>(exampleSetCapacity), static_cast<int>(exampleSetCount));

  // We perform the check instead of relying on ChangeDims performing a NOP because reservoirCount
  // might be smaller than the current size and we want to avoid reallocating the blocks.
  if(m_densities->noDims.x < temporariesSize.x || m_densities->noDims.y < temporariesSize.y)
  {
    // The following variables have a column per each element of the example sets and a row per each example set to cluster.
    m_clusterIdx->ChangeDims(temporariesSize);
    m_clusterSizes->ChangeDims(temporariesSize);
    m_clusterSizesHistogram->ChangeDims(temporariesSize);
    m_densities->ChangeDims(temporariesSize);
    m_parents->ChangeDims(temporariesSize);

    // One column per each potential cluster to output and a row for each example set.
    m_selectedClusters->ChangeDims(Vector2i(m_maxClusterCount, exampleSetCount));
    // One element per example set.
    m_nbClustersPerExampleSet->Resize(exampleSetCount);
  }
}

}
