/**
 * spaint: SemanticVisualiserFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_SPAINT_SEMANTICVISUALISERFACTORY
#define H_SPAINT_SEMANTICVISUALISERFACTORY

#include <ORUtils/DeviceType.h>

#include "interface/SemanticVisualiser.h"

namespace spaint {

/**
 * \brief This struct can be used to construct semantic visualisers.
 */
struct SemanticVisualiserFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a semantic visualiser.
   *
   * \param maxLabelCount The maximum number of labels that can be in use.
   * \param deviceType    The device on which the visualiser should operate.
   * \return              The visualiser.
   */
  static SemanticVisualiser_CPtr make_semantic_visualiser(size_t maxLabelCount, DeviceType deviceType);
};

}

#endif
