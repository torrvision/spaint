/**
 * grove: RansacFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RANSACFACTORY
#define H_GROVE_RANSACFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/PreemptiveRansac.h"

namespace grove {

class RansacFactory
{
public:
  static PreemptiveRansac_Ptr make_preemptive_ransac(ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
