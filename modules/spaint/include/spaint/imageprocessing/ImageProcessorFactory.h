/**
 * spaint: ImageProcessorFactory.h
 */

#ifndef H_SPAINT_IMAGEPROCESSORFACTORY
#define H_SPAINT_IMAGEPROCESSORFACTORY

#include <ITMLib/Utils/ITMLibSettings.h>

#include "interface/ImageProcessor.h"

namespace spaint {

/**
 * \brief TODO
 */
struct ImageProcessorFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief TODO
   */
  static ImageProcessor_CPtr make_image_processor(ITMLibSettings::DeviceType deviceType);
};

}

#endif
