/**
 * grove: DecisionForestFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_DECISIONFORESTFACTORY
#define H_GROVE_DECISIONFORESTFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/DecisionForest.h"

namespace grove {

template <typename DescriptorType, int TreeCount>
class DecisionForestFactory
{
public:
  typedef DecisionForest<DescriptorType, TreeCount> Forest;
  typedef boost::shared_ptr<Forest> Forest_Ptr;
  typedef boost::shared_ptr<const Forest> Forest_CPtr;

  static Forest_Ptr make_forest(
      ITMLib::ITMLibSettings::DeviceType deviceType, const std::string& fileName);
};

}

#endif
