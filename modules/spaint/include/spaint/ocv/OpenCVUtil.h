/**
 * spaint: OpenCVUtil.h
 */

#ifndef H_SPAINT_OPENCVUTIL
#define H_SPAINT_OPENCVUTIL

#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ITMLib/Utils/ITMLibDefines.h>

namespace spaint {

/**
 * \brief This struct provides helper functions to visualise InfiniTAM and ArrayFire images using OpenCV.
 */
struct OpenCVUtil
{
  //#################### ENUMERATIONS ####################

  /**
   * \brief An enumeration containing two possible ways of arranging multidimensional arrays in a single linear array.
   */
  enum Order
  {
    ROW_MAJOR,
    COL_MAJOR
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /*
   * \brief Displays an image and scales the pixel values by a specified scaling factor.
   *
   * \param infiniTAMImage  The InfiniTAM image.
   * \param scaleFactor     The factor by which to scale the image pixels.
   * \param windowName      The name of the window in which to display the resulting image.
   */
  static void display_image_and_scale(const std::string& windowName, float *pixelData, int width, int height, float scaleFactor);

  /*
   * \brief Displays an image and scales the pixel values to occupy the entire range [0-255].
   *
   * \param infiniTAMImage  The InfiniTAM image.
   * \param windowName      The name of the window in which to display the resulting image.
   */
  static void display_image_scale_to_range(ITMFloatImage *infiniTAMImage, const std::string& windowName);

  /**
   * \brief Makes an OpenCV image from pixel data in unsigned char format and displays it in a window with the specified name.
   *
   * \param windowName  The name to give the window.
   * \param pixelData   The image's pixel data.
   * \param width       The image's width.
   * \param height      The image's height.
   * \param order       Whether the pixel values are arranged in column-major or row-major order.
   */
  static void show_figure(const std::string& windowName, unsigned char *pixelData, int width, int height, Order order);

  static unsigned char clamp_pixel_value(float pixelValue)
  {
    if(pixelValue < 0.0f) pixelValue = 0.0f;
    if(pixelValue > 255.0f) pixelValue = 255.0f;
    return static_cast<unsigned char>(pixelValue);
  }

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
  static cv::Mat1b make_greyscale_image(T *inputData, int width, int height, Order order, ScaleFunc scaleFunc)
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
          outputData[x * height + y] = clamp_pixel_value(scaleFunc(*inputData++));
        }
      }
    }

    return result;
  }

  template <typename T>
  static cv::Mat1b make_greyscale_image(T *inputData, int width, int height, Order order)
  {
    return make_greyscale_image(inputData, width, height, order, &scale_identity<T>);
  }
};

}

#endif
