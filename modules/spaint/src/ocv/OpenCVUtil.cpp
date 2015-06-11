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

cv::Mat3b OpenCVUtil::pad_image(const cv::Mat& image, size_t paddingSize)
{
  cv::Mat3b paddedImage(image.rows + paddingSize * 2, image.cols + paddingSize * 2, image.depth());
  cv::Rect roi(paddingSize, paddingSize, image.cols, image.rows);
  image.copyTo(paddedImage(roi));
  return paddedImage;
}

void OpenCVUtil::show_scaled_greyscale_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor)
{
  show_scaled_greyscale_figure(windowName, inputData, width, height, order, ScaleByFactor(scaleFactor));
}

cv::Mat3b OpenCVUtil::tile_regular_image_patches(const std::vector<cv::Mat3b>& images, size_t tileCols, size_t tileRows, size_t patchWidth, size_t patchHeight, size_t paddingSize)
{
  // Pre-process the vector of images.
  std::vector<cv::Mat3b> processedImages;
  for(int i = 0, size = images.size(); i < size; ++i)
  {
    cv::Mat3b scaledImage(patchHeight, patchWidth, images[i].depth());
    cv::resize(images[i], scaledImage, scaledImage.size(), 0, 0, CV_INTER_NN);
    processedImages.push_back(pad_image(scaledImage, paddingSize));
  }

  int maxTiles = tileCols * tileRows;

  int tileHeight = patchHeight + 2 * paddingSize;
  int tileWidth = patchWidth + 2 * paddingSize;
  cv::Mat3b tiledImage = cv::Mat3b::zeros(tileHeight * tileRows, tileWidth * tileCols);

  for(int i = 0; i < maxTiles; ++i)
  {
    int x = (i % tileCols) * tileWidth;
    int y = (i / tileCols) * tileHeight;
    cv::Rect roi(x, y, tileWidth, tileHeight);
    processedImages[i].copyTo(tiledImage(roi));
  }

  return tiledImage;
}

cv::Mat3b OpenCVUtil::tile_regular_image_patches_within_image_bounds(const std::vector<cv::Mat3b>& images, size_t imageWidth, size_t imageHeight, size_t patchWidth, size_t patchHeight, size_t paddingSize)
{
  size_t tileWidth = patchWidth + 2 * paddingSize;
  size_t tileHeight = patchHeight + 2 * paddingSize;
  if(imageWidth < tileWidth || imageHeight < tileHeight)
  {
    throw std::runtime_error("The image dimensions are too small to tile patches of the specified size.");
  }
  size_t tileCols = imageWidth / tileWidth;
  size_t tileRows = imageHeight / tileHeight ;

  return tile_regular_image_patches(images, tileCols, tileRows, patchHeight, patchWidth, paddingSize);
}

}
