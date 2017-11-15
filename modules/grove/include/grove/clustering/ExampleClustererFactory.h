/**
 * grove: ExampleClustererFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERFACTORY
#define H_GROVE_EXAMPLECLUSTERERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ExampleClusterer.h"

namespace grove {

/**
 * \brief This struct can be used to construct example clusterers.
 */
template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS>
struct ExampleClustererFactory
{
  //#################### TYPEDEFS ####################

  typedef ExampleClusterer<ExampleType,ClusterType,MAX_CLUSTERS> Clusterer;
  typedef boost::shared_ptr<Clusterer> Clusterer_Ptr;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes an example clusterer.
   *
   * \param sigma            The sigma of the Gaussian used when computing the example densities.
   * \param tau              The maximum distance there can be between two examples that are part of the same cluster.
   * \param maxClusterCount  The maximum number of clusters retained for each set of examples (all clusters are estimated
   *                         but only the maxClusterCount largest ones are returned). Must be <= MAX_CLUSTERS.
   * \param minClusterSize   The minimum size of cluster to keep.
   * \param deviceType       The device on which the example clusterer should operate.
   * \return                 The example clusterer.
   *
   * \throws std::invalid_argument If maxClusterCount > MAX_CLUSTERS.
   */
  static Clusterer_Ptr make_clusterer(float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize, ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
