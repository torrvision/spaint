/**
 * spaint: VisualiserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_VISUALISERFACTORY
#define H_SPAINT_VISUALISERFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/DepthVisualiser.h"
#include "interface/SemanticVisualiser.h"

namespace spaint {

/**
 * \brief This struct can be used to construct visualisers.
 */
struct VisualiserFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a depth visualiser.
   *
   * \param deviceType  The device on which the visualiser should operate.
   * \return            The visualiser.
   */
  static DepthVisualiser_CPtr make_depth_visualiser(ITMLib::ITMLibSettings::DeviceType deviceType);

  /**
   * \brief Makes a semantic visualiser.
   *
   * \param maxLabelCount The maximum number of labels that can be in use.
   * \param deviceType    The device on which the visualiser should operate.
   * \return              The visualiser.
   */
  static SemanticVisualiser_CPtr make_semantic_visualiser(size_t maxLabelCount, ITMLib::ITMLibSettings::DeviceType deviceType);
};

}

#endif
