/**
 * grove: ExampleClusterer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleClusterer.h"

#include <tvgutil/itm/MemoryBlockFactory.h>
using namespace tvgutil;

namespace grove {

template <typename ExampleType, typename ClusterType>
ExampleClusterer<ExampleType, ClusterType>::ExampleClusterer(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize) :
    m_sigma(sigma), m_tau(tau), m_maxClusterCount(maxClusterCount), m_minClusterSize(minClusterSize)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_densities = mbf.make_image<float>();
  m_parents = mbf.make_image<int>();
  m_clusterIdx = mbf.make_image<int>();
  m_clusterSizes = mbf.make_image<int>();
  m_clusterSizesHistogram = mbf.make_image<int>();
  m_selectedClusters = mbf.make_image<int>();
  m_nbClustersPerReservoir = mbf.make_block<int>();
}

template <typename ExampleType, typename ClusterType>
ExampleClusterer<ExampleType, ClusterType>::~ExampleClusterer()
{}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer<ExampleType, ClusterType>::find_modes(const ExampleClusterer::ExampleImage_CPtr &exampleReservoirs,
                                  const ITMIntMemoryBlock_CPtr &keypointReservoirsSize,
                                  ExampleClusterer::ClusterBlock_Ptr &predictions,
                                  uint32_t startIdx,
                                  uint32_t count)
{
  const uint32_t nbReservoirs = exampleReservoirs->noDims.height;
  const uint32_t reservoirCapacity = exampleReservoirs->noDims.width;

  if (startIdx + count > nbReservoirs)
    throw std::runtime_error("startIdx + count > nbReservoirs");

  // Allocate local variables (a NOP except during the first run).
  allocate_temporaries(reservoirCapacity, count);

  // Grab a pointer to the beginning of the first reservoir of interest (having index startIdx).
  const ExampleType *examples = get_pointer_to_reservoir(exampleReservoirs, startIdx);
  // Grab a pointer to the size of the first reservoir of interest.
  const int *reservoirSizes = get_pointer_to_reservoir_size(keypointReservoirsSize, startIdx);
  // Grab a pointer to the first prediction to compute.
  ClusterType *predictionsData = get_pointer_to_prediction(predictions, startIdx);

  // Reset working variables
  reset_temporaries(reservoirCapacity, count);

  // Reset output predictions
  reset_predictions(predictionsData, count);

  // Compute densities
  compute_density(examples, reservoirSizes, reservoirCapacity, count, m_sigma);

  // Link neighbors
  link_neighbors(examples, reservoirSizes, reservoirCapacity, count, m_tau * m_tau);

  // Identify clusters
  identify_clusters(reservoirCapacity, count);

  // Compute cluster size histogram
  compute_cluster_size_histograms(reservoirCapacity, count);

  // Select best clusters
  select_clusters(m_maxClusterCount, m_minClusterSize, reservoirCapacity, count);

  // Finally, compute modes for each cluster and store the predictions
  compute_modes(examples, reservoirSizes, predictionsData, m_maxClusterCount, reservoirCapacity, count);

//  m_nbClustersPerReservoir->UpdateHostFromDevice();
//  m_clusterSizes->UpdateHostFromDevice();
//  reservoirs->get_reservoirs_size()->UpdateHostFromDevice();
//
//  for (int i = 0; i < count; ++i)
//  {
//    std::cout << "Reservoir " << i + startIdx << " has "
//        << m_nbClustersPerReservoir->GetData(MEMORYDEVICE_CPU)[i + startIdx]
//        << " clusters and "
//        << reservoirs->get_reservoirs_size()->GetData(MEMORYDEVICE_CPU)[i
//            + startIdx] << " elements." << std::endl;
//    for (int j = 0;
//        j < m_nbClustersPerReservoir->GetData(MEMORYDEVICE_CPU)[i + startIdx];
//        ++j)
//    {
//      std::cout << "\tCluster " << j << ": "
//          << m_clusterSizes->GetData(MEMORYDEVICE_CPU)[(i + startIdx)
//              * reservoirCapacity + j] << " elements." << std::endl;
//    }
  //  }
}

template<typename ExampleType, typename ClusterType>
void ExampleClusterer<ExampleType, ClusterType>::allocate_temporaries(uint32_t reservoirCapacity, uint32_t reservoirCount)
{
  const Vector2i temporariesSize(reservoirCapacity, reservoirCount);

  // We perform the check instead of relying on ChangeDims performing a NOP because reservoirCount
  // might be smaller than the current size and we want to avoid reallocating the blocks.
  if(m_densities->noDims.x < temporariesSize.x || m_densities->noDims.y < temporariesSize.y)
  {
    // Happens only once
    m_densities->ChangeDims(temporariesSize);
    m_parents->ChangeDims(temporariesSize);
    m_clusterIdx->ChangeDims(temporariesSize);
    m_clusterSizes->ChangeDims(temporariesSize);
    m_clusterSizesHistogram->ChangeDims(temporariesSize);

    m_selectedClusters->ChangeDims(Vector2i(m_maxClusterCount, reservoirCount));
    m_nbClustersPerReservoir->Resize(reservoirCount);
  }
}

}
