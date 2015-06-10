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

#if 0
std::vector<cv::Mat3b> OpenCVUtil::pad_images(const std::vector<cv::Mat3b>& images, size_t paddingSize)
{
  std::vector<cv::Mat3b> paddedImages;
  for(size_t i = 0, size = images.size(); i < size; ++i)
  {
    paddedImages.push_back(pad_image(images[i]));
  }

  return paddedImages;
}
#endif

cv::Mat3b OpenCVUtil::pad_image(const cv::Mat3b& image, size_t paddingSize)
{
  cv::Mat3b paddedImage(image.rows + paddingSize * 2, image.cols + paddingSize * 2, image.depth());
  cv::Rect roi(paddingSize, paddingSize, image.cols, image.rows);
  image.copyTo(paddedImage(roi));
  return paddedImage;
}

cv::Mat3b OpenCVUtil::tile_image_patches(const std::vector<cv::Mat3b>& images, size_t tiledWidth, size_t tiledHeight, size_t patchWidth, size_t patchHeight)
{
  // Pre-process the vector of images.
  std::vector<cv::Mat3b> processedImages;
  for(int i = 0, size = images.size(); i < size; ++i)
  {
    //cv::Mat3b scaledImage(images[i].rows * scaleFactor, images[i].cols * scaleFactor, images[i].depth());
    cv::Mat3b scaledImage(patchHeight, patchWidth, images[i].depth());
    cv::resize(images[i], scaledImage, scaledImage.size(), 0, 0, CV_INTER_NN);

    const int paddingSize = 3;
    cv::Mat3b paddedImage = pad_image(scaledImage, paddingSize);
#if 0
    cv::Mat3b paddedImage(patchHeight + borderSize * 2, patchWidth + borderSize * 2, images[i].depth());
    cv::Rect roi(borderSize, borderSize, patchWidth, patchHeight);
    scaledImage.copyTo(paddedImage(roi));
#endif
    processedImages.push_back(paddedImage);
  }

  int maxTiles = tiledWidth * tiledHeight;

  int tileHeight = processedImages[0].rows;
  int tileWidth = processedImages[0].cols;
  cv::Mat3b tiledImage = cv::Mat3b::zeros(tileHeight * tiledHeight, tileWidth * tiledWidth);

  for(int i = 0; i < maxTiles; ++i)
  {
    int x = (i % tiledWidth) * tileWidth;
    int y = (i / tiledWidth) * tileHeight;
    cv::Rect roi(x, y, tileWidth, tileHeight);
    processedImages[i].copyTo(tiledImage(roi));
  }

  return tiledImage;
}

}
