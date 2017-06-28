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
  // Check the preconditions.
  if(maxClusterCount > MAX_CLUSTERS)
  {
    throw std::invalid_argument("Error: maxClusterCount > MAX_CLUSTERS");
  }

  // Initialise the temporary variables that are used as part of a find_modes call.
  // Initially, all are empty. We resize them later, once we know the right sizes.
  itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();

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
void ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::find_modes(const ExampleImage_CPtr& exampleSets, const ITMIntMemoryBlock_CPtr& exampleSetSizes,
                                                                        uint32_t exampleSetStart, uint32_t exampleSetCount, ClustersBlock_Ptr& clusterContainers)
{
  const uint32_t nbExampleSets = exampleSets->noDims.height;
  const uint32_t exampleSetCapacity = exampleSets->noDims.width;

  if(exampleSetStart + exampleSetCount > nbExampleSets)
  {
    throw std::invalid_argument("Error: exampleSetStart + exampleSetCount > nbExampleSets");
  }

  // Reallocate the temporary variables needed for the call as necessary. In practice, this tends to be a no-op
  // for all calls to find_modes except the first, since we only need to reallocate if more memory is required,
  // and the way in which find_modes is usually called tends not to cause this to happen.
  reallocate_temporaries(exampleSetCapacity, exampleSetCount);

  // Reset the temporary variables needed for the call.
  reset_temporaries(exampleSetCapacity, exampleSetCount);

  // Reset the output clusters for each example set of interest.
  Clusters *clustersData = get_pointer_to_cluster(clusterContainers, exampleSetStart);
  reset_clusters(clustersData, exampleSetCount);

  // Compute the density of examples around each example in the example sets of interest.
  const ExampleType *exampleSetsData = get_pointer_to_example_set(exampleSets, exampleSetStart);
  const int *exampleSetSizesData = get_pointer_to_example_set_size(exampleSetSizes, exampleSetStart);
  compute_density(exampleSetsData, exampleSetSizesData, exampleSetCapacity, exampleSetCount, m_sigma);

  // Link neighbouring examples in a tree structure and cut the branches according to the maximum distance tau.
  link_neighbors(exampleSetsData, exampleSetSizesData, exampleSetCapacity, exampleSetCount, m_tau * m_tau);

  // Identify clusters (assign identifiers to the clusters).
  identify_clusters(exampleSetCapacity, exampleSetCount);

  // Compute cluster size histograms (used to select the largest clusters).
  compute_cluster_size_histograms(exampleSetCapacity, exampleSetCount);

  // Select largest clusters.
  select_clusters(m_maxClusterCount, m_minClusterSize, exampleSetCapacity, exampleSetCount);

  // Finally, compute parameters for each cluster and store the predictions.
  compute_cluster_parameters(exampleSetsData, exampleSetSizesData, clustersData, m_maxClusterCount, exampleSetCapacity, exampleSetCount);

// Debug.
#if 0
  m_nbClustersPerExampleSet->UpdateHostFromDevice();
  m_clusterSizes->UpdateHostFromDevice();
  exampleSetSizes->UpdateHostFromDevice();

  for (uint32_t i = 0; i < exampleSetCount; ++i)
  {
    std::cout << "Example set " << i + exampleSetStart << " has "
              << m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU)[i + exampleSetStart] << " clusters and "
              << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[i + exampleSetStart] << " elements.\n";

    for (int j = 0; j < m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU)[i + exampleSetStart]; ++j)
    {
      std::cout << "\tCluster " << j << ": "
                << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[(i + exampleSetStart) * exampleSetCapacity + j] << " elements.\n";
    }
  }
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
void ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS>::reallocate_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount)
{
  // Get the current size of (most of) the temporary images.
  const Vector2i oldImgSize = m_densities->noDims;

  // Work out the new size needed for (most of) the temporary images.
  const Vector2i newImgSize(static_cast<int>(exampleSetCapacity), static_cast<int>(exampleSetCount));

  // If the new image size is larger (in either dimension), reallocate the temporaries.
  // We perform this check explicitly instead of relying on ChangeDims performing a no-op
  // because exampleSetCount might be smaller than the current size and we want to avoid
  // reallocating the blocks.
  if(newImgSize.width > oldImgSize.width || newImgSize.height > oldImgSize.height)
  {
    // The following images have a row for each example set to cluster and a column for each element of the example sets.
    m_clusterIdx->ChangeDims(newImgSize);
    m_clusterSizes->ChangeDims(newImgSize);
    m_clusterSizesHistogram->ChangeDims(newImgSize);
    m_densities->ChangeDims(newImgSize);
    m_parents->ChangeDims(newImgSize);

    // The selected clusters image has a row for each example set and a column for each potential cluster to output.
    m_selectedClusters->ChangeDims(Vector2i(m_maxClusterCount, exampleSetCount));

    // Finally, we store a single number for each example set that denotes the number of valid clusters in the set.
    m_nbClustersPerExampleSet->Resize(exampleSetCount);
  }
}

}
