/**
 * spaint: TouchUtil.h
 */

#ifndef H_SPAINT_TOUCHUTIL
#define H_SPAINT_TOUCHUTIL

#include "../ocv/OpenCVUtil.h"

#include <arrayfire.h>

#include <rafl/core/RandomForest.h>

namespace spaint {

/**
 * \brief This class contains functions that help us to extract descriptors from candidate components.
 */
class TouchUtil
{
public:
  /**
   * \brief Calculates a global histogram descriptor from an image.
   *
   * \param img  The image from which to calculate a histogram descriptor.
   * \return     The histogram descriptor.
   */
  static rafl::Descriptor_CPtr calculate_histogram_descriptor(const af::array& img);
};

}

#endif
