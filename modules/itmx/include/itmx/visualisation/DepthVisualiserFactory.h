/**
 * itmx: DepthVisualiserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_ITMX_DEPTHVISUALISERFACTORY
#define H_ITMX_DEPTHVISUALISERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/DepthVisualiser.h"

namespace itmx {

/**
 * \brief This struct can be used to construct depth visualisers.
 */
struct DepthVisualiserFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a depth visualiser.
   *
   * \param deviceType  The device on which the visualiser should operate.
   * \return            The visualiser.
   */
  static DepthVisualiser_CPtr make_depth_visualiser(ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
