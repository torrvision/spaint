/**
 * grove: ExampleClusterer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "ExampleClusterer.h"

#include <spaint/util/MemoryBlockFactory.h>
using spaint::MemoryBlockFactory;

namespace grove {

template <typename ExampleType, typename ClusterType>
ExampleClusterer<ExampleType, ClusterType>::ExampleClusterer(float sigma, float tau, uint32_t minClusterSize) :
    m_sigma(sigma), m_tau(tau), m_minClusterSize(minClusterSize)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_densities = mbf.make_image<float>();
  m_parents = mbf.make_image<int>();
  m_clusterIdx = mbf.make_image<int>();
  m_clusterSizes = mbf.make_image<int>();
  m_clusterSizesHistogram = mbf.make_image<int>();
  m_selectedClusters = mbf.make_image<int>();
  m_nbClustersPerReservoir = mbf.make_image<int>();
}

template <typename ExampleType, typename ClusterType>
ExampleClusterer<ExampleType, ClusterType>::~ExampleClusterer()
{}

}
