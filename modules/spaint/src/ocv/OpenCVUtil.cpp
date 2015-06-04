/**
 * spaint: OpenCVUtil.cpp
 */

#include "ocv/OpenCVUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void OpenCVUtil::show_scaled_greyscale_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor)
{
  show_scaled_greyscale_figure(windowName, inputData, width, height, order, ScaleByFactor(scaleFactor));
}

}
