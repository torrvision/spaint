/**
 * spaint: OpenCVUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "ocv/OpenCVUtil.h"

#include <stdexcept>

namespace spaint {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

cv::Mat3b OpenCVUtil::make_rgb_image(const float *rgbData, int width, int height)
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

cv::Mat3b OpenCVUtil::make_rgb_image(const Vector4u *rgbData, int width, int height)
{
  cv::Mat3b result = cv::Mat3b::zeros(height, width);
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      result(y,x) = cv::Vec3b(
        static_cast<unsigned char>(rgbData->b),
        static_cast<unsigned char>(rgbData->g),
        static_cast<unsigned char>(rgbData->r)
      );
      ++rgbData;
    }
  }
  return result;
}

cv::Mat3b OpenCVUtil::pad_image(const cv::Mat3b& image, int paddingSize)
{
  cv::Mat3b paddedImage = cv::Mat3b::zeros(image.rows + paddingSize * 2, image.cols + paddingSize * 2);
  cv::Rect roi(paddingSize, paddingSize, image.cols, image.rows);
  image.copyTo(paddedImage(roi));
  return paddedImage;
}

void OpenCVUtil::show_scaled_greyscale_figure(const std::string& windowName, const float *inputData, int width, int height, Order order, float scaleFactor)
{
  show_scaled_greyscale_figure(windowName, inputData, width, height, order, ScaleByFactor(scaleFactor));
}

cv::Mat3b OpenCVUtil::tile_image_patches(const std::vector<cv::Mat3b>& images, int tileCols, int tileRows, int patchWidth, int patchHeight, int paddingSize)
{
  // Scale and pad the input images to make the tiles.
  std::vector<cv::Mat3b> tiles;
  for(size_t i = 0, size = images.size(); i < size; ++i)
  {
    cv::Mat3b scaledImage(patchHeight, patchWidth);
    cv::resize(images[i], scaledImage, scaledImage.size(), 0.0, 0.0, CV_INTER_NN);
    tiles.push_back(pad_image(scaledImage, paddingSize));
  }

  // Make a blank output image of the right size to which the tiles can be copied.
  const int tileHeight = patchHeight + 2 * paddingSize;
  const int tileWidth = patchWidth + 2 * paddingSize;
  cv::Mat3b tiledImage = cv::Mat3b::zeros(tileHeight * tileRows, tileWidth * tileCols);

  // Copy the tiles across to the output image.
  for(int i = 0, tileCount = tileCols * tileRows; i < tileCount; ++i)
  {
    int x = (i % tileCols) * tileWidth;
    int y = (i / tileCols) * tileHeight;
    cv::Rect roi(x, y, tileWidth, tileHeight);
    tiles[i].copyTo(tiledImage(roi));
  }

  return tiledImage;
}

cv::Mat3b OpenCVUtil::tile_image_patches_bounded(const std::vector<cv::Mat3b>& images, int imageWidth, int imageHeight, int patchWidth, int patchHeight, int paddingSize)
{
  // Compute the size of each tile.
  int tileWidth = patchWidth + 2 * paddingSize;
  int tileHeight = patchHeight + 2 * paddingSize;

  // If the image is too small to show even a single column or single row of tiles, throw.
  if(imageWidth < tileWidth || imageHeight < tileHeight)
  {
    throw std::runtime_error("The image dimensions are too small to tile patches of the specified size");
  }

  // Determine how many columns and rows of tiles we can fit into an output image of the specified size.
  int tileCols = imageWidth / tileWidth;
  int tileRows = imageHeight / tileHeight ;

  // Tile the patches and return the output image.
  return tile_image_patches(images, tileCols, tileRows, patchHeight, patchWidth, paddingSize);
}

}
