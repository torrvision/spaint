/**
 * spaint: OpenCVExtra.h
 */

#ifndef H_SPAINT_OPENCVEXTRA
#define H_SPAINT_OPENCVEXTRA

#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

class OpenCVExtra
{
public:
  enum Order
  {
    ROW_MAJOR,
    COL_MAJOR
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
/*  static void imshow_float_and_scale(const std::string& windowName, const cv::Mat& image, float scaleFactor)
  {
    cv::Mat canvas;
    image.convertTo(canvas, CV_8U, scaleFactor, 0.0f);
    cv::imshow(windowName, canvas);
  }

  static void imshow_float_scale_to_range(const std::string& windowName, const cv::Mat& image)
  {
    double minValue, maxValue;
    cv::Mat tmp;
    image.copyTo(tmp);
    cv::minMaxLoc(tmp, &minValue, &maxValue);
    double range = maxValue - minValue;

    cv::Mat canvas;

    image.convertTo(canvas, CV_8U, 255.0/range, -minValue * 255.0 / range);

    cv::imshow(windowName, canvas);
  }
*/
  static void display_image_and_scale(ITMFloatImage *infiniTAMImage, float scaleFactor, const std::string& windowName)
  {
    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the infiniTAM image pointer.
    infiniTAMImage->UpdateHostFromDevice();
    float *ITMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);

    //Create an OpenCV image and get the data pointer.
    cv::Mat OCVImage = cv::Mat::zeros(height, width, CV_8UC1);
    int8_t *OCVImageDataPtr = (int8_t*)OCVImage.data;

    //Inner loop to set the OpenCV Image Pixes.
    for(int y = 0; y < height; ++y)
      for(int x = 0; x < width; ++x)
      {
        float intensity = OpenCVExtra::get_itm_mat_32SC1(ITMImageDataPtr, x, y, width) * scaleFactor;
        if((intensity < 0) || (intensity > 255)) intensity = 0;
        set_ocv_mat_8UC1(OCVImageDataPtr, x, y, width, static_cast<int8_t>(intensity));
      }

    // Display the image.
    cv::imshow(windowName, OCVImage);
  }

  static void display_image_scale_to_range(ITMFloatImage *infiniTAMImage, const std::string& windowName)
  {
    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the minimum and maximum values in the infiniTAM image.
    infiniTAMImage->UpdateHostFromDevice();
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
        float intensity = OpenCVExtra::get_itm_mat_32SC1(ITMImageDataPtr, x, y, width) * alpha + beta;
        set_ocv_mat_8UC1(OCVImageDataPtr, x, y, width, static_cast<int8_t>(intensity));
      }

    // Display the image.
    cv::imshow(windowName, OCVImage);
  }

  static void ocvfig(const std::string& windowName, unsigned char *pixels, int width, int height, Order order) 
  {
    if(order == Order::COL_MAJOR)
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

private:
  static void set_ocv_mat_8UC1(int8_t *OCVImageDataPtr, int x, int y, int width, int8_t value)
  {
    OCVImageDataPtr[y * width + x] = value;
  }

  static std::pair<float, float> itm_mat_32SC1_min_max_calculator(float *ITMImageDataPtr, int width, int height)
  {
    std::vector<float> tmpImgVector(ITMImageDataPtr, ITMImageDataPtr + width * height);
    auto result = std::minmax_element(tmpImgVector.begin(), tmpImgVector.end());
    return std::make_pair(*result.first, *result.second);
  }

  static void set_ocv_mat_32F(float *opencvImageDataPtr, int x, int y, int width, float value)
  {
    opencvImageDataPtr[y * width + x] = value;
  }

  static float get_itm_mat_32SC1(float *InfiniTAMImageDataPtr, int x, int y, int width)
  {
    return InfiniTAMImageDataPtr[y * width + x];
  }
};

}

#endif

