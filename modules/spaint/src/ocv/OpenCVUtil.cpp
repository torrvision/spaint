/**
 * spaint: OpenCVUtil.cpp
 */

#include "ocv/OpenCVUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

cv::Mat3b OpenCVUtil::make_image_rgb_cpu(const float *rgbData, int width, int height)
{
  cv::Mat3b result = cv::Mat3b::zeros(height, width);
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      result(y,x) = cv::Vec3b(
        static_cast<unsigned char>(rgbData[2]),
        static_cast<unsigned char>(rgbData[1]),
        static_cast<unsigned char>(rgbData[0])
      );
      rgbData += 3;
    }
  }
  return result;
}

void OpenCVUtil::show_scaled_greyscale_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor)
{
  show_scaled_greyscale_figure(windowName, inputData, width, height, order, ScaleByFactor(scaleFactor));
}

}
