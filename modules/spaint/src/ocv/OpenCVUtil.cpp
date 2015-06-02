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

  // Inner loop to set the OpenCV Image Pixes.
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
    {
      float intensity = OpenCVUtil::get_itm_mat_32SC1(itmImageDataPtr, x, y, width) * scaleFactor;
      if(intensity < 0) intensity = 0;
      if(intensity > 255) intensity = 255;
      set_ocv_mat_8UC1(ocvImageDataPtr, x, y, width, static_cast<unsigned char>(intensity));
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
  std::pair<float, float> minAndMax = itm_mat_32SC1_min_max_calculator(itmImageDataPtr, width, height);

  // Calculate the mapping to image values between 0 and 255.
  float range = minAndMax.second - minAndMax.first;
  float scale = 255.0f / range;

  // Create an OpenCV image and get the data pointer.
  cv::Mat ocvImage = cv::Mat::zeros(height, width, CV_8UC1);
  unsigned char *ocvImageDataPtr = ocvImage.data;

  // Inner loop to set the OpenCV Image Pixels.
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
    {
      float intensity = (OpenCVUtil::get_itm_mat_32SC1(itmImageDataPtr, x, y, width) - minAndMax.first) * scale;
      set_ocv_mat_8UC1(ocvImageDataPtr, x, y, width, static_cast<unsigned char>(intensity));
    }

  // Display the image.
  cv::imshow(windowName, ocvImage);
}

void OpenCVUtil::ocvfig(const std::string& windowName, unsigned char *pixels, int width, int height, Order order)
{
  if(order == COL_MAJOR)
  {
    int step = height * sizeof(unsigned char);
    cv::Mat img(width, height, CV_8UC1, pixels, step);
    cv::transpose(img, img);
    cv::imshow(windowName, img);
  }
  else
  {
    int step = width * sizeof(unsigned char);
    cv::Mat img(height, width, CV_8UC1, pixels, step);
    cv::imshow(windowName, img);
  }
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

void OpenCVUtil::set_ocv_mat_8UC1(unsigned char *ocvImageDataPtr, int x, int y, int width, unsigned char value)
{
  ocvImageDataPtr[y * width + x] = value;
}

std::pair<float, float> OpenCVUtil::itm_mat_32SC1_min_max_calculator(float *itmImageDataPtr, int width, int height)
{
  std::vector<float> tmpImgVector(itmImageDataPtr, itmImageDataPtr + width * height);
  float minElement = *std::min_element(tmpImgVector.begin(), tmpImgVector.end());
  float maxElement = *std::max_element(tmpImgVector.begin(), tmpImgVector.end());
  return std::make_pair(minElement, maxElement);
}

float OpenCVUtil::get_itm_mat_32SC1(float *itmImageDataPtr, int x, int y, int width)
{
  return itmImageDataPtr[y * width + x];
}

}
