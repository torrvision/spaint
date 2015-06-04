/**
 * spaint: OpenCVUtil.cpp
 */

#include "ocv/OpenCVUtil.h"

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

cv::Mat3b OpenCVUtil::make_image_rgb_cpu(const float *rgbData, int width, int height)
{
  cv::Mat3b result = cv::Mat3b::zeros(height, width);
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      result(y,x) = cv::Vec3b(
        static_cast<unsigned char>(rgbData[2]),
        static_cast<unsigned char>(rgbData[1]),
        static_cast<unsigned char>(rgbData[0])
      );
      rgbData += 3;
    }
  }
  return result;
}

void OpenCVUtil::show_scaled_greyscale_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor)
{
  show_scaled_greyscale_figure(windowName, inputData, width, height, order, ScaleByFactor(scaleFactor));
}

cv::Mat3b OpenCVUtil::tile_images(const std::vector<cv::Mat3b>& images, int tiledWidth, int tiledHeight, size_t scaleFactor)
{
  // Pre-process the vector of images.
  std::vector<cv::Mat3b> processedImages;
  for(int i = 0, size = images.size(); i < size; ++i)
  {
    cv::Mat3b scaledImage(images[i].rows * scaleFactor, images[i].cols * scaleFactor, images[i].depth());
    cv::resize(images[i], scaledImage, scaledImage.size(), 0, 0, CV_INTER_NN);

    int cols = scaledImage.cols;
    int rows = scaledImage.rows;

    const int borderSize = 3;
    cv::Mat3b paddedImage(rows + borderSize * 2, cols + borderSize * 2, images[i].depth());
    cv::Rect roi(borderSize, borderSize, cols, rows);
    scaledImage.copyTo(paddedImage(roi));
    processedImages.push_back(paddedImage);
  }

  int maxTiles = tiledWidth * tiledHeight;

  int patchHeight = processedImages[0].rows;
  int patchWidth = processedImages[0].cols;
  cv::Mat3b tiledImage = cv::Mat3b::zeros(patchHeight * tiledHeight, patchWidth * tiledWidth);

  for(int i = 0; i < maxTiles; ++i)
  {
    int x = (i % tiledWidth) * patchWidth;
    int y = (i / tiledWidth) * patchHeight;
    cv::Rect roi(x, y, patchWidth, patchHeight);
    processedImages[i].copyTo(tiledImage(roi));
  }

  return tiledImage;
}

}
