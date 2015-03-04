/**
 * spaint: OCVdebugger.h
 */

#ifndef H_SPAINT_OCVDEBUGGER
#define H_SPAINT_OCVDEBUGGER

#include <algorithm>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

class OCVdebugger{
public:
  static void display_image_and_scale(ITMFloatImage *infiniTAMImage, float scaleFactor, const std::string& windowName)
  {
    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the infiniTAM image pointer.
    float *ITMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);

    //Create an OpenCV image and get the data pointer.
    cv::Mat OCVImage = cv::Mat::zeros(height, width, CV_8UC1);
    int8_t *OCVImageDataPtr = (int8_t*)OCVImage.data;

    //Inner loop to set the OpenCV Image Pixes.
    for(int y = 0; y < height; ++y)
      for(int x = 0; x < width; ++x)
      {
        float intensity = get_itm_mat_32SC1(ITMImageDataPtr, x, y, width) * scaleFactor;
        if((intensity < 0) || (intensity > 255)) intensity = 0;
        set_ocv_mat_8UC1(OCVImageDataPtr, x, y, width, static_cast<int8_t>(intensity));
      }

    // Display the image.
    const int waitTimeMilliseconds = 10;
    display_image_and_wait(OCVImage, windowName, waitTimeMilliseconds);
  }

  static void display_image_scale_to_range(ITMFloatImage *infiniTAMImage, const std::string& windowName)
  {
    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the minimum and maximum values in the infiniTAM image.
    float *ITMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);
    std::pair<float, float> minAndMax = itm_mat_32SC1_min_max_calculator(ITMImageDataPtr, width, height);

    // Calculate the mapping to image values between 0 and 255.
    float range = minAndMax.second - minAndMax.first;
    float alpha = 255.0 / range;
    float beta = -minAndMax.first * 255.0 / range;

    // Create an OpenCV image and get the data pointer.
    cv::Mat OCVImage = cv::Mat::zeros(height, width, CV_8UC1);
    int8_t *OCVImageDataPtr = (int8_t*)OCVImage.data;
    
    // Inner loop to set the OpenCV Image Pixels.
    for(int y = 0; y < height; ++y)
      for(int x = 0; x < width; ++x)
      {
        float intensity = get_itm_mat_32SC1(ITMImageDataPtr, x, y, width) * alpha + beta;
        set_ocv_mat_8UC1(OCVImageDataPtr, x, y, width, static_cast<int8_t>(intensity));
      }

    // Display the image.
    const int waitTimeMilliseconds = 10;
    display_image_and_wait(OCVImage, windowName, waitTimeMilliseconds);
  }

private:
  static void set_ocv_mat_8UC1(int8_t *OCVImageDataPtr, int x, int y, int width, int8_t value)
  {
    OCVImageDataPtr[y * width + x] = value;
  }

  static float get_itm_mat_32SC1(float *ITMImageDataPtr, int x, int y, int width)
  {
    return ITMImageDataPtr[y * width + x];
  }

  static std::pair<float, float> itm_mat_32SC1_min_max_calculator(float *ITMImageDataPtr, int width, int height)
  {
    std::vector<float> tmpImgVector(ITMImageDataPtr, ITMImageDataPtr + width * height);
    auto result = std::minmax_element(tmpImgVector.begin(), tmpImgVector.end());
    return std::make_pair(*result.first, *result.second);
  }

  static void display_image_and_wait(const cv::Mat& image, const std::string& windowName, int waitTimeMilliseconds)
  {
    cv::imshow(windowName, image);
    cv::waitKey(waitTimeMilliseconds);
  }
};

}

#endif

