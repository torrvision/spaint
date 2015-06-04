/**
 * spaint: OpenCVUtil.h
 */

#ifndef H_SPAINT_OPENCVUTIL
#define H_SPAINT_OPENCVUTIL

#include <stdexcept>

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
  static void display_image_and_scale(ITMFloatImage *infiniTAMImage, float scaleFactor, const std::string& windowName);

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
   * \param pixels      A pointer to the first pixel in the image.
   * \param width       The width of the image.
   * \param height      The hwight of the image.
   * \param order       Whether the pixel values are arranged in column-major or row-major order.
   */
  static void show_figure(const std::string& windowName, unsigned char *pixels, int width, int height, Order order);
};

}

#endif
