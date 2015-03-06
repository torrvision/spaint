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
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  static cv::Mat ITM2UMATNOCOPY(ITMFloatImage *infiniTAMImage)
  {
    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the InfiniTAM image data pointer.
    float *infiniTAMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);

    std::vector<float> tmpImgVector(infiniTAMImageDataPtr, infiniTAMImageDataPtr + width * height);

    const bool copyData = false;
    cv::Mat opencvImage(tmpImgVector, copyData);
    opencvImage.reshape(1, height);

    return opencvImage;
  }

  static void ITM2MAT(ITMFloatImage *infiniTAMImage, cv::Mat *opencvImage)
  {
    if(opencvImage->type() != CV_32F)
    {
      throw std::runtime_error("opencv image must be single channel floating point!\n");
    }

    int width = infiniTAMImage->noDims.x;
    int height = infiniTAMImage->noDims.y;

    // Get the InfiniTAM image data pointer.
    infiniTAMImage->UpdateHostFromDevice();
    float *infiniTAMImageDataPtr = infiniTAMImage->GetData(MEMORYDEVICE_CPU);

    // Get the OpenCV image data pointer.
    float *opencvImageDataPtr = (float*)opencvImage->data;

    // Inner loop to set the OpenCV data.
    for(int y = 0; y < height; ++y)
      for(int x = 0; x < width; ++x)
      {
        float intensity = get_itm_mat_32SC1(infiniTAMImageDataPtr, x, y, width);
        set_ocv_mat_32F(opencvImageDataPtr, x, y, width, intensity);
      }
  }

  static void imshow_float_and_scale(const std::string& windowName, const cv::Mat& image, float scaleFactor)
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

