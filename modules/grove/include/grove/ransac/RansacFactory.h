/**
 * grove: RansacFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RANSACFACTORY
#define H_GROVE_RANSACFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/PreemptiveRansac.h"

namespace grove {

/**
 * \brief This class can be used to create instances of the PreemptiveRansac class.
 */
class RansacFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Creates an instance of PreemptiveRansac using a certain device.
   *
   * \param deviceType The device type.
   *
   * \return An instance of PreemptiveRansac.
   */
  static PreemptiveRansac_Ptr make_preemptive_ransac(ITMLib::ITMLibSettings::DeviceType deviceType);
};
}

#endif
