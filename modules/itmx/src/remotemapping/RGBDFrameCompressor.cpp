/**
 * itmx: RGBDFrameCompressor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/RGBDFrameCompressor.h"

#include <stdexcept>

#ifdef WITH_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include "base/MemoryBlockFactory.h"

namespace itmx {

//#################### NESTED TYPES ####################

struct RGBDFrameCompressor::Impl
{
  /** A vector containing the results of depth compression. */
  std::vector<uint8_t> compressedDepthBytes;

  /** A vector containing the results of RGB compression. */
  std::vector<uint8_t> compressedRgbBytes;

  /** The type of compression algorithm to use for the depth images. */
  DepthCompressionType depthCompressionType;

  /** The type of compression algorithm to use for the RGB images. */
  RGBCompressionType rgbCompressionType;

  /** An image storing the temporary uncompressed depth data. */
  ITMShortImage_Ptr uncompressedDepthImage;

#ifdef WITH_OPENCV
  /** An OpenCV image storing the temporary uncompressed depth data. */
  cv::Mat uncompressedDepthMat;
#endif

  /** An image storing the temporary uncompressed RGB data. */
  ITMUChar4Image_Ptr uncompressedRgbImage;

#ifdef WITH_OPENCV
  /** An OpenCV image storing the temporary uncompressed RGB data. */
  cv::Mat uncompressedRgbMat;
#endif
};

//#################### CONSTRUCTORS ####################

RGBDFrameCompressor::RGBDFrameCompressor(const Vector2i& rgbImageSize, const Vector2i& depthImageSize, RGBCompressionType rgbCompressionType, DepthCompressionType depthCompressionType)
: m_impl(new Impl)
{
  MemoryBlockFactory& mbf = MemoryBlockFactory::instance();

  m_impl->depthCompressionType = depthCompressionType;
  m_impl->rgbCompressionType = rgbCompressionType;
  m_impl->uncompressedDepthImage = mbf.make_image<short>(depthImageSize);
  m_impl->uncompressedRgbImage = mbf.make_image<Vector4u>(rgbImageSize);

  // If we're using the PNG compression from OpenCV to compress depth images, allocate a temporary OpenCV image accordingly.
  // The format of this image needs to be CV_16U to properly encode a depth image as PNG. We will use convertTo to fill
  // this image from an ITMShortImage.
  if(depthCompressionType == DepthCompressionType::DEPTH_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    m_impl->uncompressedDepthMat.create(depthImageSize.y, depthImageSize.x, CV_16UC1);
#else
    throw std::invalid_argument("Error: Cannot compress depth images to PNG format. Reconfigure in CMake with the WITH_OPENCV option set to on.");
#endif
  }

  // If we're using either the JPG or PNG compression from OpenCV to compress RGB images, allocate a temporary OpenCV image accordingly.
  // The image we allocate will be a three channel image, and we will use cvtColor to fill it.
  if(rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_JPG || rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    m_impl->uncompressedRgbMat.create(rgbImageSize.y, rgbImageSize.x, CV_8UC3);
#else
    throw std::invalid_argument("Error: Cannot compress RGB images to PNG or JPG format. Reconfigure in CMake with the WITH_OPENCV option set to on.");
#endif
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RGBDFrameCompressor::compress_rgbd_frame(const RGBDFrameMessage& uncompressedFrame, CompressedRGBDFrameHeaderMessage& compressedHeader, CompressedRGBDFrameMessage& compressedFrame)
{
  // First, copy the metadata.
  compressedFrame.set_frame_index(uncompressedFrame.extract_frame_index());
  compressedFrame.set_pose(uncompressedFrame.extract_pose());

  // Then, extract the images from the uncompressed message.
  uncompressedFrame.extract_depth_image(m_impl->uncompressedDepthImage.get());
  uncompressedFrame.extract_rgb_image(m_impl->uncompressedRgbImage.get());

  // Perform the compression.
  compress_depth_image();
  compress_rgb_image();

  // Now, prepare the compressed header.
  compressedHeader.set_depth_image_size(static_cast<uint32_t>(m_impl->compressedDepthBytes.size()));
  compressedHeader.set_rgb_image_size(static_cast<uint32_t>(m_impl->compressedRgbBytes.size()));

  // Finally, prepare the compressed frame.
  compressedFrame.set_compressed_image_sizes(compressedHeader);
  compressedFrame.set_depth_image_data(m_impl->compressedDepthBytes);
  compressedFrame.set_rgb_image_data(m_impl->compressedRgbBytes);
}

void RGBDFrameCompressor::uncompress_rgbd_frame(const CompressedRGBDFrameMessage& compressedFrame, RGBDFrameMessage& uncompressedFrame)
{
  // First, copy the metadata.
  uncompressedFrame.set_frame_index(compressedFrame.extract_frame_index());
  uncompressedFrame.set_pose(compressedFrame.extract_pose());

  // Then, extract the compressed byte vectors.
  compressedFrame.extract_depth_image_data(m_impl->compressedDepthBytes);
  compressedFrame.extract_rgb_image_data(m_impl->compressedRgbBytes);

  // Perform the uncompression.
  uncompress_depth_image();
  uncompress_rgb_image();

  // Finally, store the images into the uncompressed message.
  uncompressedFrame.set_depth_image(m_impl->uncompressedDepthImage);
  uncompressedFrame.set_rgb_image(m_impl->uncompressedRgbImage);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void RGBDFrameCompressor::compress_depth_image()
{
  if(m_impl->depthCompressionType == DepthCompressionType::DEPTH_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    // If we're using PNG compresson, first wrap the InfiniTAM depth image as an OpenCV image.
    cv::Mat depthWrapper(
      m_impl->uncompressedDepthImage->noDims.y,
      m_impl->uncompressedDepthImage->noDims.x,
      CV_16SC1,
      m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU)
    );

    // Then, convert the format to CV_16U (this is necessary to properly encode the image in PNG format).
    depthWrapper.convertTo(m_impl->uncompressedDepthMat, CV_16U);

    // Finally, compress the image, storing the compressed representation in an internal buffer.
    cv::imencode(".png", m_impl->uncompressedDepthMat, m_impl->compressedDepthBytes);
#endif
  }
  else
  {
    // If we're not using PNG compression, simply copy the raw bytes of the image into the internal buffer.
    m_impl->compressedDepthBytes.resize(m_impl->uncompressedDepthImage->dataSize * sizeof(short));
    memcpy(m_impl->compressedDepthBytes.data(), m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedDepthBytes.size());
  }
}

void RGBDFrameCompressor::compress_rgb_image()
{
  if(m_impl->rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_NONE)
  {
    // If we're not using compression, simply copy the raw bytes of the image into an internal buffer.
    m_impl->compressedRgbBytes.resize(m_impl->uncompressedRgbImage->dataSize * sizeof(Vector4u));
    memcpy(m_impl->compressedRgbBytes.data(), m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedRgbBytes.size());
  }
  else
  {
#ifdef WITH_OPENCV
    // Otherwise, first wrap the InfiniTAM RGB image as an OpenCV image.
    cv::Mat rgbWrapper(
      m_impl->uncompressedRgbImage->noDims.y,
      m_impl->uncompressedRgbImage->noDims.x,
      CV_8UC4,
      m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU)
    );

    // Then, make a copy of this image in which we reorder the colours and drop the alpha channel.
    cv::cvtColor(rgbWrapper, m_impl->uncompressedRgbMat, CV_RGBA2BGR);

    // Finally, compress the image using the appropriate format, storing the compressed representation in the internal buffer.
    const std::string outputFormat = m_impl->rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_JPG ? ".jpg" : ".png";
    cv::imencode(outputFormat, m_impl->uncompressedRgbMat, m_impl->compressedRgbBytes);
#endif
  }
}

void RGBDFrameCompressor::uncompress_depth_image()
{
  if(m_impl->depthCompressionType == DepthCompressionType::DEPTH_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    // If we're using PNG compression, first decode the image into a preallocated internal buffer.
    m_impl->uncompressedDepthMat = cv::imdecode(m_impl->compressedDepthBytes, cv::IMREAD_ANYDEPTH, &m_impl->uncompressedDepthMat);

    // Then, copy the image back into an InfiniTAM image. Note that as part of this process,
    // we convert the format back from CV_16U (as returned to cv::imdecode) to CV_16S (the
    // format InfiniTAM is expecting).
    cv::Mat depthWrapper(
      m_impl->uncompressedDepthImage->noDims.y,
      m_impl->uncompressedDepthImage->noDims.x, CV_16SC1,
      m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU)
    );

    m_impl->uncompressedDepthMat.convertTo(depthWrapper, CV_16S);
#endif
  }
  else
  {
    // Otherwise, first check that the size of the uncompressed image matches that of the compressed data.
    if(m_impl->uncompressedDepthImage->dataSize * sizeof(short) != m_impl->compressedDepthBytes.size())
    {
      throw std::runtime_error("Depth image size in the compressed message does not match the uncompressed depth image size.");
    }

    // If it does, simply copy the bytes across.
    memcpy(m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedDepthBytes.data(), m_impl->compressedDepthBytes.size());
  }
}

void RGBDFrameCompressor::uncompress_rgb_image()
{
  if(m_impl->rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_NONE)
  {
    // If we're not using compression, check that the size of the uncompressed image matches that of the compressed data.
    if(m_impl->uncompressedRgbImage->dataSize * sizeof(Vector4u) != m_impl->compressedRgbBytes.size())
    {
      throw std::runtime_error("RGB image size in the compressed message does not match the uncompressed RGB image size.");
    }

    // If it does, simply copy the bytes across.
    memcpy(m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedRgbBytes.data(), m_impl->compressedRgbBytes.size());
  }
  else
  {
#ifdef WITH_OPENCV
    // Otherwise, first decode the image into a preallocated internal buffer.
    m_impl->uncompressedRgbMat = cv::imdecode(m_impl->compressedRgbBytes, cv::IMREAD_COLOR, &m_impl->uncompressedRgbMat);

    // Then, copy the image back into an InfiniTAM image. Note that as part of this process,
    // we reorder the bytes and re-add the alpha channel.
    cv::Mat rgbWrapper(
      m_impl->uncompressedRgbImage->noDims.y,
      m_impl->uncompressedRgbImage->noDims.x, CV_8UC4,
      m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU)
    );

    cv::cvtColor(m_impl->uncompressedRgbMat, rgbWrapper, CV_BGR2RGBA);
#endif
  }
}

}
