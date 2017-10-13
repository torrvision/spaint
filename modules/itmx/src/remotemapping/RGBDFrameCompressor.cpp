/**
 * itmx: RGBDFrameCompressor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/RGBDFrameCompressor.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "base/MemoryBlockFactory.h"

#include <iostream>

namespace itmx {

RGBDFrameCompressor::RGBDFrameCompressor(const Vector2i &rgbImageSize, const Vector2i &depthImageSize)
{
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();

  // Preallocate data.
  m_uncompressedDepthImage = mbf.make_image<short>(depthImageSize);
  m_uncompressedRgbImage = mbf.make_image<Vector4u>(rgbImageSize);

  m_uncompressedDepthMat.create(depthImageSize.y, depthImageSize.x, CV_16UC1);
  m_uncompressedRgbMat.create(rgbImageSize.y, rgbImageSize.x, CV_8UC3); // Three channel, will use cvtColor to fill it.
}

void RGBDFrameCompressor::compress_rgbd_frame(
    const RGBDFrameMessage &uncompressedFrame,
    CompressedRGBDFrameHeaderMessage &compressedHeader,
    CompressedRGBDFrameMessage &compressedFrame)
{
  // First, copy the metadata.
  compressedFrame.set_frame_index(uncompressedFrame.extract_frame_index());
  compressedFrame.set_pose(uncompressedFrame.extract_pose());

  // Then, extract the images from the uncompressed message.
  uncompressedFrame.extract_depth_image(m_uncompressedDepthImage.get());
  uncompressedFrame.extract_rgb_image(m_uncompressedRgbImage.get());

  // Copy the images onto OpenCV's matrices.
  cv::Mat depthWrapper(m_uncompressedDepthImage->noDims.y, m_uncompressedDepthImage->noDims.x, CV_16SC1, m_uncompressedDepthImage->GetData(MEMORYDEVICE_CPU));
  depthWrapper.convertTo(m_uncompressedDepthMat, CV_16U);

  cv::Mat rgbWrapper(m_uncompressedRgbImage->noDims.y, m_uncompressedRgbImage->noDims.x, CV_8UC4, m_uncompressedRgbImage->GetData(MEMORYDEVICE_CPU));
  cv::cvtColor(rgbWrapper, m_uncompressedRgbMat, CV_RGBA2BGR);

  // Now compress the images, depth to PNG, colour to PNG (or JPG).
  cv::imencode(".png", m_uncompressedDepthMat, m_compressedDepthBytes);
  cv::imencode(".jpg", m_uncompressedRgbMat, m_compressedRgbBytes);

  // Now prepare the compressed header.
  compressedHeader.set_depth_image_size(m_compressedDepthBytes.size());
  compressedHeader.set_rgb_image_size(m_compressedRgbBytes.size());

  // Finally, prepare the compressed message.
  compressedFrame.set_compressed_image_sizes(compressedHeader);
  compressedFrame.set_depth_image_data(m_compressedDepthBytes);
  compressedFrame.set_rgb_image_data(m_compressedRgbBytes);
}

void RGBDFrameCompressor::uncompress_rgbd_frame(const CompressedRGBDFrameMessage &compressedFrame, RGBDFrameMessage &uncompressedFrame)
{
  // Copy metadata.
  uncompressedFrame.set_frame_index(compressedFrame.extract_frame_index());
  uncompressedFrame.set_pose(compressedFrame.extract_pose());

  std::cout << "Uncompressed metadata" << std::endl;

  // Extract the compressed byte vectors.
  compressedFrame.extract_depth_image_data(m_compressedDepthBytes);
  compressedFrame.extract_rgb_image_data(m_compressedRgbBytes);

  std::cout << "Extracted bytes" << std::endl;

  // Uncompress the images.
  m_uncompressedDepthMat = cv::imdecode(m_compressedDepthBytes, cv::IMREAD_ANYDEPTH, &m_uncompressedDepthMat);
  m_uncompressedRgbMat = cv::imdecode(m_compressedRgbBytes, cv::IMREAD_COLOR, &m_uncompressedRgbMat);

  std::cout << "Decoded images. " <<  m_uncompressedDepthMat.size() << " -- " << m_uncompressedDepthImage->noDims << std::endl;
  std::cout << "Depth depth: " << m_uncompressedDepthMat.depth() << " tpye: " << m_uncompressedDepthMat.type() << std::endl;

  // Copy the images to ITM's images.
  cv::Mat depthWrapper(m_uncompressedDepthImage->noDims.y, m_uncompressedDepthImage->noDims.x, CV_16SC1, m_uncompressedDepthImage->GetData(MEMORYDEVICE_CPU));
  m_uncompressedDepthMat.convertTo(depthWrapper, CV_16S);

  cv::Mat rgbWrapper(m_uncompressedRgbImage->noDims.y, m_uncompressedRgbImage->noDims.x, CV_8UC4, m_uncompressedRgbImage->GetData(MEMORYDEVICE_CPU));
  cv::cvtColor(m_uncompressedRgbMat, rgbWrapper, CV_BGR2RGBA);

  // Store the images in the uncompressed message.
  uncompressedFrame.set_depth_image(m_uncompressedDepthImage);
  uncompressedFrame.set_rgb_image(m_uncompressedRgbImage);
}

}
