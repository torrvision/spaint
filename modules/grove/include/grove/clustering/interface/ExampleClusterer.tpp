/**
 * grove: ExampleClusterer.tpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleClusterer.h"

#include <iostream>

#include <itmx/base/MemoryBlockFactory.h>

namespace grove {

//#################### CONSTRUCTORS ####################

template <typename ExampleType, typename ClusterType, int MaxClusters>
ExampleClusterer<ExampleType, ClusterType, MaxClusters>::ExampleClusterer(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize)
: m_maxClusterCount(maxClusterCount), m_minClusterSize(minClusterSize), m_sigma(sigma), m_tau(tau)
{
  // Check the preconditions.
  if(maxClusterCount > MaxClusters)
  {
    throw std::invalid_argument("Error: maxClusterCount > MaxClusters");
  }

  // Initialise the temporary variables that are used as part of a cluster_examples call.
  // Initially, all are empty. We resize them later, once we know the right sizes.
  itmx::MemoryBlockFactory& mbf = itmx::MemoryBlockFactory::instance();

  m_clusterIndices = mbf.make_image<int>();
  m_clusterSizeHistograms = mbf.make_image<int>();
  m_clusterSizes = mbf.make_image<int>();
  m_densities = mbf.make_image<float>();
  m_nbClustersPerExampleSet = mbf.make_block<int>();
  m_parents = mbf.make_image<int>();
  m_selectedClusters = mbf.make_image<int>();
}

//#################### DESTRUCTOR ####################

template <typename ExampleType, typename ClusterType, int MaxClusters>
ExampleClusterer<ExampleType,ClusterType,MaxClusters>::~ExampleClusterer()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MaxClusters>
void ExampleClusterer<ExampleType,ClusterType,MaxClusters>::cluster_examples(const ExampleImage_CPtr& exampleSets, const ITMIntMemoryBlock_CPtr& exampleSetSizes,
                                                                             uint32_t exampleSetStart, uint32_t exampleSetCount, ClusterContainers_Ptr& clusterContainers)
{
  const uint32_t nbExampleSets = exampleSets->noDims.height;
  const uint32_t exampleSetCapacity = exampleSets->noDims.width;

  if(exampleSetStart + exampleSetCount > nbExampleSets)
  {
    throw std::invalid_argument("Error: exampleSetStart + exampleSetCount > nbExampleSets");
  }

  // Reallocate the temporary variables needed for the call as necessary. In practice, this tends to be a no-op for
  // all calls to cluster_examples except the first, since we only need to reallocate if more memory is required,
  // and the way in which cluster_examples is usually called tends not to cause this to happen.
  reallocate_temporaries(exampleSetCapacity, exampleSetCount);

  // Reset the temporary variables needed for the call.
  reset_temporaries(exampleSetCapacity, exampleSetCount);

  // Reset the cluster containers for each example set of interest.
  ClusterContainer *clusterContainersPtr = get_pointer_to_cluster_container(clusterContainers, exampleSetStart);
  reset_cluster_containers(clusterContainersPtr, exampleSetCount);

  // Compute the density of examples around each example in the example sets of interest.
  const ExampleType *exampleSetsData = get_pointer_to_example_set(exampleSets, exampleSetStart);
  const int *exampleSetSizesData = get_pointer_to_example_set_size(exampleSetSizes, exampleSetStart);
  compute_densities(exampleSetsData, exampleSetSizesData, exampleSetCapacity, exampleSetCount);

  // Compute the parent and initial cluster indices to assign to each example as part of the neighbour-linking
  // step of the really quick shift (RQS) algorithm. The algorithm links neighbouring examples in a tree structure,
  // separating example clusters based on a distance tau.
  compute_parents(exampleSetsData, exampleSetSizesData, exampleSetCapacity, exampleSetCount, m_tau * m_tau);

  // Compute the final cluster indices to assign to each example by following the parent links just computed.
  compute_cluster_indices(exampleSetCapacity, exampleSetCount);

  // Compute a histogram of the cluster sizes for each example set (these are used to select the largest clusters).
  compute_cluster_size_histograms(exampleSetCapacity, exampleSetCount);

  // Select the largest clusters for each example set (up to a maximum limit).
  select_clusters(exampleSetCapacity, exampleSetCount);

  // Finally, compute the parameters for and store each selected cluster for each example set.
  create_selected_clusters(exampleSetsData, exampleSetSizesData, exampleSetCapacity, exampleSetCount, clusterContainersPtr);

#if 0
  // For debugging purposes only.
  m_nbClustersPerExampleSet->UpdateHostFromDevice();
  m_clusterSizes->UpdateHostFromDevice();
  exampleSetSizes->UpdateHostFromDevice();

  for(uint32_t i = 0; i < exampleSetCount; ++i)
  {
    std::cout << "Example set " << i + exampleSetStart << " has "
              << m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU)[i + exampleSetStart] << " clusters and "
              << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[i + exampleSetStart] << " elements.\n";

    for(int j = 0; j < m_nbClustersPerExampleSet->GetData(MEMORYDEVICE_CPU)[i + exampleSetStart]; ++j)
    {
      std::cout << "\tCluster " << j << ": "
                << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[(i + exampleSetStart) * exampleSetCapacity + j] << " elements.\n";
    }
  }
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename ExampleType, typename ClusterType, int MaxClusters>
void ExampleClusterer<ExampleType,ClusterType,MaxClusters>::reallocate_temporaries(uint32_t exampleSetCapacity, uint32_t exampleSetCount)
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
    m_clusterIndices->ChangeDims(newImgSize);
    m_clusterSizes->ChangeDims(newImgSize);
    m_clusterSizeHistograms->ChangeDims(newImgSize);
    m_densities->ChangeDims(newImgSize);
    m_parents->ChangeDims(newImgSize);

    // The selected clusters image has a row for each example set and a column for each potential cluster to output.
    m_selectedClusters->ChangeDims(Vector2i(m_maxClusterCount, exampleSetCount));

    // Finally, we store a single number for each example set that denotes the number of valid clusters in the set.
    m_nbClustersPerExampleSet->Resize(exampleSetCount);
  }
}

}
