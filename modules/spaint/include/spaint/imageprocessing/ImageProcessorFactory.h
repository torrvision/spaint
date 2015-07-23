/**
 * spaint: ImageProcessorFactory.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_SPAINT_IMAGEPROCESSORFACTORY
#define H_SPAINT_IMAGEPROCESSORFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ImageProcessor.h"

namespace spaint {

/**
 * \brief This struct can be used to construct image processors.
 */
struct ImageProcessorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes an image processor.
   *
   * \param deviceType  The device on which the image processor should operate.
   * \return            The image processor.
   */
  static ImageProcessor_CPtr make_image_processor(ITMLibSettings::DeviceType deviceType);
};

}

#endif
