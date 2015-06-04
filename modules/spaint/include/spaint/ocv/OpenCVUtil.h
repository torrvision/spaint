/**
 * spaint: OpenCVUtil.h
 */

#ifndef H_SPAINT_OPENCVUTIL
#define H_SPAINT_OPENCVUTIL

#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace spaint {

/**
 * \brief This class provides helper functions to visualise InfiniTAM and ArrayFire images using OpenCV.
 */
class OpenCVUtil
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration containing two possible ways of arranging multidimensional arrays in a single linear array.
   */
  enum Order
  {
    ROW_MAJOR,
    COL_MAJOR
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  template <typename T>
  static T scale_identity(T t)
  {
    return t;
  }

  struct ScaleByFactor
  {
    float factor;

    ScaleByFactor(float factor_)
    : factor(factor_)
    {}

    float operator()(float f) const
    {
      return f * factor;
    }
  };

  struct ScaleToRange
  {
    float minValue;
    float scale;

    ScaleToRange(const float *inputData, int width, int height)
    {
      const float *inputEnd = inputData + width * height;
      float maxValue = *std::max_element(inputData, inputEnd);
      minValue = *std::min_element(inputData, inputEnd);
      scale = 255.0f / (maxValue - minValue);
    }

    float operator()(float f) const
    {
      return (f - minValue) * scale;
    }
  };

  template <typename T, typename ScaleFunc>
  static cv::Mat1b make_greyscale_image(const T *inputData, int width, int height, Order order, ScaleFunc scaleFunc)
  {
    cv::Mat1b result = cv::Mat::zeros(height, width, CV_8UC1);
    unsigned char *outputData = result.data;

    int pixelCount = width * height;
    if(order == ROW_MAJOR)
    {
      for(int i = 0; i < pixelCount; ++i)
      {
        *outputData++ = clamp_pixel_value(scaleFunc(*inputData++));
      }
    }
    else // order == COL_MAJOR
    {
      for(int y = 0; y < height; ++y)
      {
        for(int x = 0; x < width; ++x)
        {
          *outputData++ = clamp_pixel_value(scaleFunc(inputData[x * height + y]));
        }
      }
    }

    return result;
  }

  template <typename T>
  static cv::Mat1b make_greyscale_image(const T *inputData, int width, int height, Order order)
  {
    return make_greyscale_image(inputData, width, height, order, &scale_identity<T>);
  }

  template <typename T, typename ScaleFunc>
  static void show_figure(const std::string& windowName, const T *inputData, int width, int height, Order order, ScaleFunc scaleFunc)
  {
    cv::imshow(windowName, make_greyscale_image(inputData, width, height, order, scaleFunc));
  }

  template <typename T>
  static void show_figure(const std::string& windowName, const T *inputData, int width, int height, Order order)
  {
    show_figure(windowName, inputData, width, height, order, &scale_identity<T>);
  }

  static void show_scaled_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor)
  {
    show_figure(windowName, inputData, width, height, order, ScaleByFactor(scaleFactor));
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  static unsigned char clamp_pixel_value(float pixelValue);
};

}

#endif
