/**
 * spaint: OpenCVUtil.cpp
 */

#include "ocv/OpenCVUtil.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

// TODO

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

unsigned char OpenCVUtil::clamp_pixel_value(float pixelValue)
{
  if(pixelValue < 0.0f) pixelValue = 0.0f;
  if(pixelValue > 255.0f) pixelValue = 255.0f;
  return static_cast<unsigned char>(pixelValue);
}

}
