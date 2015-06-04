/**
 * spaint: OpenCVUtil.cpp
 */

#include "ocv/OpenCVUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

void OpenCVUtil::display_image_and_scale(ITMFloatImage *infiniTAMImage, float scaleFactor, const std::string& windowName)
{
  int width = infiniTAMImage->noDims.x;
  int height = infiniTAMImage->noDims.y;

  // Get the infiniTAM image pointer.
  infiniTAMImage->UpdateHostFromDevice();
  float *itmImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);

  // Create an OpenCV image and get the data pointer.
  cv::Mat ocvImage = cv::Mat::zeros(height, width, CV_8UC1);
  unsigned char *ocvImageDataPtr = ocvImage.data;

  // Inner loop to set the OpenCV Image Pixels.
  for(int i = 0, pixelCount = width * height; i < pixelCount; ++i)
  {
    float intensity = *itmImageDataPtr++ * scaleFactor;
    if(intensity < 0) intensity = 0;
    if(intensity > 255) intensity = 255;
    *ocvImageDataPtr++ = static_cast<unsigned char>(intensity);
  }

  // Display the image.
  cv::imshow(windowName, ocvImage);
}

void OpenCVUtil::display_image_scale_to_range(ITMFloatImage *infiniTAMImage, const std::string& windowName)
{
  int width = infiniTAMImage->noDims.x;
  int height = infiniTAMImage->noDims.y;

  // Get the minimum and maximum values in the infiniTAM image.
  infiniTAMImage->UpdateHostFromDevice();
  float *itmImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);
  float *itmImageDataEnd = itmImageDataPtr + width * height;
  float minValue = *std::min_element(itmImageDataPtr, itmImageDataEnd);
  float maxValue = *std::max_element(itmImageDataPtr, itmImageDataEnd);

  // Calculate the mapping to image values between 0 and 255.
  float range = maxValue - minValue;
  float scale = 255.0f / range;

  // Create an OpenCV image and get the data pointer.
  cv::Mat ocvImage = cv::Mat::zeros(height, width, CV_8UC1);
  unsigned char *ocvImageDataPtr = ocvImage.data;

  // Inner loop to set the OpenCV Image Pixels.
  for(int i = 0, pixelCount = width * height; i < pixelCount; ++i)
  {
    *ocvImageDataPtr++ = static_cast<unsigned char>((*itmImageDataPtr++ - minValue) * scale);
  }

  // Display the image.
  cv::imshow(windowName, ocvImage);
}

void OpenCVUtil::show_figure(const std::string& windowName, unsigned char *pixels, int width, int height, Order order)
{
  cv::Mat img;
  if(order == COL_MAJOR)
  {
    int step = height * sizeof(unsigned char);
    img = cv::Mat(width, height, CV_8UC1, pixels, step);
    cv::transpose(img, img);
  }
  else
  {
    int step = width * sizeof(unsigned char);
    img = cv::Mat(height, width, CV_8UC1, pixels, step);
  }
  cv::imshow(windowName, img);
}

}
