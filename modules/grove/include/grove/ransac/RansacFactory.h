/**
 * grove: RansacFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_GROVE_RANSACFACTORY
#define H_GROVE_RANSACFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include <tvgutil/misc/SettingsContainer.h>

#include "interface/PreemptiveRansac.h"

namespace grove {

/**
 * \brief This struct can be used to create instances of classes that perform variants of the RANSAC algorithm.
 */
struct RansacFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Creates a pre-emptive RANSAC.
   *
   * \param settings    The settings used to configure the pre-emptive RANSAC.
   * \param deviceType  The device on which the pre-emptive RANSAC should operate.
   * \return            The pre-emptive RANSAC.
   */
  static PreemptiveRansac_Ptr make_preemptive_ransac(const tvgutil::SettingsContainer_CPtr& settings, ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
