/**
 * spaint: OpenCVUtil.h
 */

#ifndef H_SPAINT_OPENCVUTIL
#define H_SPAINT_OPENCVUTIL

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace spaint {

/**
 * \brief This class provides helper functions to visualise image data using OpenCV.
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
    COL_MAJOR,
    ROW_MAJOR
  };

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief A functor that scales values by the specified factor.
   */
  struct ScaleByFactor
  {
    /** The factor by which to scale values. */
    float m_factor;

    /**
     * \brief Constructs a function that scales values by the specified factor.
     *
     * \param factor  The factor by which to scale values.
     */
    explicit ScaleByFactor(float factor)
    : m_factor(factor)
    {}

    /**
     * \brief Scales the specified value by the factor associated with this functor.
     *
     * \return  The result of scaling the specified value by the factor associated with this functor.
     */
    float operator()(float value) const
    {
      return value * m_factor;
    }
  };

  /**
   * \brief A functor that implements a linear mapping from an input range [minInputValue,maxInputValue] derived from an array of input data
   *        to the range [minOutputValue,maxOutputValue].
   */
  struct ScaleDataToRange
  {
    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

    /** The lower bound of the input range. */
    float m_minInputValue;

    /** The lower bound of the output range. */
    float m_minOutputValue;

    /** The ratio between the size of the output range and the size of the input range (e.g. if the output range is twice the size then this would equal 2). */
    float m_scalingFactor;

    //~~~~~~~~~~~~~~~~~~~~ CONSTRUCTORS ~~~~~~~~~~~~~~~~~~~~

    /**
     * \brief Constructs a functor that implements a linear mapping from the range [minInputValue,maxInputValue] derived from an array of input data
     *        to the range [minOutputValue,maxOutputValue].
     *
     * \param inputData       The array of input data from which to derive the input range.
     * \param inputDateSize   The size of the array of input data.
     * \param minOutputValue  The lower bound of the output range.
     * \param maxOutputValue  The upper bound of the output range.
     */
    ScaleDataToRange(const float *inputData, int inputDataSize, float minOutputValue, float maxOutputValue)
    : m_minOutputValue(minOutputValue)
    {
      const float *inputDataEnd = inputData + inputDataSize;
      float maxInputValue = *std::max_element(inputData, inputDataEnd);
      m_minInputValue = *std::min_element(inputData, inputDataEnd);
      m_scalingFactor = (maxOutputValue - minOutputValue) / (maxInputValue - m_minInputValue);
    }

    /**
     * \brief Maps the specified input value into the output range.
     *
     * \param inputValue  A value in the input range.
     * \return            The result of mapping the specified input value into the output range.
     */
    float operator()(float inputValue) const
    {
      return m_minOutputValue + (inputValue - m_minInputValue) * m_scalingFactor;
    }
  };

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a greyscale OpenCV image from some pixel data in the specified format.
   *
   * \param inputData The pixel data for the image.
   * \param width     The width of the image.
   * \param height    The height of the image.
   * \param order     Whether the pixel data is in row-major or column-major order.
   * \return          The OpenCV image.
   */
  template <typename T>
  static cv::Mat1b make_greyscale_image(const T *inputData, int width, int height, Order order)
  {
    return make_greyscale_image(inputData, width, height, order, &identity<T>);
  }

  /**
   * \brief Makes a greyscale OpenCV image from some pixel data in the specified format,
   *        applying the specified scaling function to each pixel value as it goes.
   *
   * \param inputData The pixel data for the image.
   * \param width     The width of the image.
   * \param height    The height of the image.
   * \param order     Whether the pixel data is in row-major or column-major order.
   * \param scaleFunc The scaling function.
   * \return          The OpenCV image.
   */
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

  /**
   * \brief Makes an RGB image of the specified size from some pixel data.
   *
   * \param rgbData The pixel data for the image, in the format [R1,G1,B1,R2,G2,B2,...].
   * \param width   The width of the image.
   * \param height  The height of the image.
   * \return        The image.
   */
  static cv::Mat3b make_rgb_image(const float *rgbData, int width, int height);

  /**
   * \brief Makes an image with a black border around it.
   *
   * \param image       The image.
   * \param paddingSize The size of the black border to be placed around the image (in pixels).
   */
  static cv::Mat3b pad_image(const cv::Mat& image, size_t paddingSize);

  /**
   * \brief Makes a greyscale OpenCV image from some pixel data in the specified format and shows it in a named window.
   *
   * \param windowName  The name to give the window.
   * \param inputData   The pixel data for the image.
   * \param width       The width of the image.
   * \param height      The height of the image.
   * \param order       Whether the pixel data is in row-major or column-major order.
   */
  template <typename T>
  static void show_greyscale_figure(const std::string& windowName, const T *inputData, int width, int height, Order order)
  {
    show_scaled_greyscale_figure(windowName, inputData, width, height, order, &identity<T>);
  }

  /**
   * \brief Makes a greyscale OpenCV image from some pixel data in the specified format, applying the specified scaling
   *        factor to each pixel value as it goes, and shows the resulting image in a named window.
   *
   * \param windowName  The name to give the window.
   * \param inputData   The pixel data for the image.
   * \param width       The width of the image.
   * \param height      The height of the image.
   * \param order       Whether the pixel data is in row-major or column-major order.
   * \param scaleFactor The scaling factor.
   */
  static void show_scaled_greyscale_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor);

  /**
   * \brief Makes a greyscale OpenCV image from some pixel data in the specified format, applying the specified scaling
   *        function to each pixel value as it goes, and shows the resulting image in a named window.
   *
   * \param windowName  The name to give the window.
   * \param inputData   The pixel data for the image.
   * \param width       The width of the image.
   * \param height      The height of the image.
   * \param order       Whether the pixel data is in row-major or column-major order.
   * \param scaleFunc   The scaling function.
   */
  template <typename T, typename ScaleFunc>
  static void show_scaled_greyscale_figure(const std::string& windowName, const T *inputData, int width, int height, Order order, ScaleFunc scaleFunc)
  {
    cv::imshow(windowName, make_greyscale_image(inputData, width, height, order, scaleFunc));
  }

  /**
   * \brief TODO.
   */
  static cv::Mat3b tile_image_patches(const std::vector<cv::Mat3b>& images, size_t imageWidth, size_t imageHeight, size_t patchWidth, size_t patchHeight, size_t paddingSize = 2);

  /**
   * \brief TODO.
   */
  static cv::Mat3b tile_image_patches_bounded(const std::vector<cv::Mat3b>& images, size_t tileCols, size_t tileRows, size_t patchWidth, size_t patchHeight, size_t paddingSize = 2);

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Clamps the specified pixel value to the range [0,255] and converts it to an unsigned char.
   *
   * \param pixelValue  The pixel value.
   * \return            The clamped pixel value as an unsigned char.
   */
  template <typename T>
  static unsigned char clamp_pixel_value(T pixelValue)
  {
    if(pixelValue < T(0)) pixelValue = T(0);
    if(pixelValue > T(255)) pixelValue = T(255);
    return static_cast<unsigned char>(pixelValue);
  }

  /**
   * \brief Returns the value it is passed.
   *
   * \param value A value.
   * \return      The same value.
   */
  template <typename T>
  static T identity(T value)
  {
    return value;
  }
};

}

#endif
