/**
 * grove: PreemptiveRansacFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_PREEMPTIVERANSACFACTORY
#define H_GROVE_PREEMPTIVERANSACFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include <tvgutil/misc/SettingsContainer.h>

#include "interface/PreemptiveRansac.h"

namespace grove {

/**
 * \brief This struct can be used to create instances of preemptive RANSAC.
 */
struct PreemptiveRansacFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Creates an instance of preemptive RANSAC.
   *
   * \param settings    The settings to use to configure the instance of preemptive RANSAC.
   * \param deviceType  The device on which the instance of preemptive RANSAC should operate.
   * \return            The instance of preemptive RANSAC.
   */
  static PreemptiveRansac_Ptr make_preemptive_ransac(const tvgutil::SettingsContainer_CPtr& settings, ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
