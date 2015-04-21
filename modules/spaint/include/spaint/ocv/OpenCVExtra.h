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

/**
 * \brief This class provides helper functions to visualise InfiniTAM and ArrayFire images in OpenCV.
 */
class OpenCVExtra
{
  //#################### PUBLIC ENUMERATIONS ####################
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
  /*
   * \brief Displays an image and scales the pixel values by a specified scaling factor.
   *
   * \param infiniTAMImage  The InfiniTAM image.
   * \param scaleFactor     The factor by which to scale the image pixels.
   * \param windowName      The name of the window in which to display the resulting image.
   */
  static void display_image_and_scale(ITMFloatImage *infiniTAMImage, float scaleFactor, const std::string& windowName)
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
        float intensity = OpenCVExtra::get_itm_mat_32SC1(itmImageDataPtr, x, y, width) * scaleFactor;
        if(intensity < 0) intensity = 0;
        if(intensity > 255) intensity = 255;
        set_ocv_mat_8UC1(ocvImageDataPtr, x, y, width, static_cast<unsigned char>(intensity));
      }

    // Display the image.
    cv::imshow(windowName, ocvImage);
  }

  /*
   * \brief Displays an image and scales the pixel values to occupy the entire range [0-255].
   *
   * \param infiniTAMImage  The InfiniTAM image.
   * \param windowName      The name of the window in which to display the resulting image.
   */
  static void display_image_scale_to_range(ITMFloatImage *infiniTAMImage, const std::string& windowName)
  {
    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the minimum and maximum values in the infiniTAM image.
    infiniTAMImage->UpdateHostFromDevice();
    float *itmImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);
    std::pair<float, float> minAndMax = itm_mat_32SC1_min_max_calculator(itmImageDataPtr, width, height);

    // Calculate the mapping to image values between 0 and 255.
    float range = minAndMax.second - minAndMax.first;
    float scale = 255.0 / range;

    // Create an OpenCV image and get the data pointer.
    cv::Mat ocvImage = cv::Mat::zeros(height, width, CV_8UC1);
    unsigned char *ocvImageDataPtr = ocvImage.data;

    // Inner loop to set the OpenCV Image Pixels.
    for(int y = 0; y < height; ++y)
      for(int x = 0; x < width; ++x)
      {
        float intensity = (OpenCVExtra::get_itm_mat_32SC1(itmImageDataPtr, x, y, width) - minAndMax.first) * scale;
        set_ocv_mat_8UC1(ocvImageDataPtr, x, y, width, static_cast<unsigned char>(intensity));
      }

    // Display the image.
    cv::imshow(windowName, ocvImage);
  }

  /**
   * \brief Displays an image in a window from an array of pixel values.
   *
   * \param windowName   The name of the window.
   * \param pixels       A pointer to the first pixel element in the image.
   * \param width        The width of the image.
   * \param height       The hwight of the image.
   * \param order        Whether the pixel values are arrange din column-major or row-major order.
   */
  static void ocvfig(const std::string& windowName, unsigned char *pixels, int width, int height, Order order)
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

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Sets an OpenCV image pixel.
   *
   * \param ocvImageDataPtr   The pointer to the first element in the image.
   * \param x                 The x position in the image.
   * \param y                 The y position in wht image.
   * \param width             The width of the image.
   * \param value             The pixel value to assign to the specified image pixel.
   */
  static void set_ocv_mat_8UC1(unsigned char *ocvImageDataPtr, int x, int y, int width, unsigned char value)
  {
    ocvImageDataPtr[y * width + x] = value;
  }

  /**
   * \brief Calculates the minimum and maximum values in an InfiniTAM image.
   *
   * \param itmImageDataPtr   The InfiniTAM image data pointer.
   * \param width             The width of the image.
   * \param height            The height of the image.
   * \return                  A pair of values indicating the minimum and maximum values found.
   */
  static std::pair<float, float> itm_mat_32SC1_min_max_calculator(float *itmImageDataPtr, int width, int height)
  {
    std::vector<float> tmpImgVector(itmImageDataPtr, itmImageDataPtr + width * height);
    float minElement = *std::min_element(tmpImgVector.begin(), tmpImgVector.end());
    float maxElement = *std::max_element(tmpImgVector.begin(), tmpImgVector.end());
    return std::make_pair(minElement, maxElement);
  }

  /**
   * \brief Gets the value contained at a specified position in an InfiniTAM image.
   *
   * \param itmImageDataPtr   The pointer to the first element in the InfiniTAM image.
   * \param x                       The x position in the image.
   * \param y                       The y position in the image.
   * \param width                   The width of the image.
   * \return                        The value contained at the specified location.
   */
  static float get_itm_mat_32SC1(float *itmImageDataPtr, int x, int y, int width)
  {
    return itmImageDataPtr[y * width + x];
  }
};

}

#endif

