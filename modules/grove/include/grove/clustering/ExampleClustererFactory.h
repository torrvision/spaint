/**
 * grove: ExampleClustererFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_EXAMPLECLUSTERERFACTORY
#define H_GROVE_EXAMPLECLUSTERERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ExampleClusterer.h"

namespace grove
{

template <typename ExampleType, typename ClusterType, int MAX_CLUSTERS = 10>
class ExampleClustererFactory
{
public:
  typedef ExampleClusterer<ExampleType, ClusterType, MAX_CLUSTERS> Clusterer;
  typedef boost::shared_ptr<Clusterer> Clusterer_Ptr;
  typedef boost::shared_ptr<const Clusterer> Clusterer_CPtr;

  static Clusterer_Ptr make_clusterer(ITMLib::ITMLibSettings::DeviceType deviceType,
                                      float sigma, float tau, uint32_t maxClusterCount, uint32_t minClusterSize);
};

}

#endif
