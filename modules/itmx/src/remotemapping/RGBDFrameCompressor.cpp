/**
 * itmx: RGBDFrameCompressor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights
 * reserved.
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

//#################### PRIVATE NESTED TYPES ####################

struct RGBDFrameCompressor::Impl
{
  /** A vector containing the results of depth compression. */
  std::vector<uint8_t> compressedDepthBytes;

  /** A vector containing the results of rgb compression. */
  std::vector<uint8_t> compressedRgbBytes;

  /** The type of compression algorithm to use with the depth images. */
  DepthCompressionType depthCompressionType;

  /** The type of compression algorithm to use with the rgb images. */
  RGBCompressionType rgbCompressionType;

  /** An image storing the temporary uncompressed depth data. */
  ITMShortImage_Ptr uncompressedDepthImage;

  /** An image storing the temporary uncompressed colour data. */
  ITMUChar4Image_Ptr uncompressedRgbImage;

#ifdef WITH_OPENCV
  /** A Mat storing the temporary uncompressed depth data. */
  cv::Mat uncompressedDepthMat;

  /** A Mat storing the temporary uncompressed colour data. */
  cv::Mat uncompressedRgbMat;
#endif
};

//#################### CONSTRUCTORS ####################

RGBDFrameCompressor::RGBDFrameCompressor(const Vector2i &rgbImageSize,
                                         const Vector2i &depthImageSize,
                                         DepthCompressionType depthCompressionType,
                                         RGBCompressionType rgbCompressionType)
{
  MemoryBlockFactory &mbf = MemoryBlockFactory::instance();

  m_impl.reset(new Impl);

  m_impl->depthCompressionType = depthCompressionType;
  m_impl->rgbCompressionType = rgbCompressionType;

  // Handle depth compression type.
  if (depthCompressionType == DepthCompressionType::DEPTH_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    // Needs to be CV_16U to properly encode into PNG. Will have to use convertTo to fill this Mat with ITMShortImages.
    m_impl->uncompressedDepthMat.create(depthImageSize.y, depthImageSize.x, CV_16UC1);
#else
    throw std::invalid_argument("Cannot compress depth images to PNG format. Rebuild enabling WITH_OPENCV.");
#endif
  }

  // Handle RGB compression type.
  if (rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_JPG || rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    // Three channel, will use cvtColor to fill it.
    m_impl->uncompressedRgbMat.create(rgbImageSize.y, rgbImageSize.x, CV_8UC3);
#else
    throw std::invalid_argument("Cannot compress colour images to PNG or JPG format. Rebuild enabling WITH_OPENCV.");
#endif
  }

  // Preallocate data that is always used.
  m_impl->uncompressedDepthImage = mbf.make_image<short>(depthImageSize);
  m_impl->uncompressedRgbImage = mbf.make_image<Vector4u>(rgbImageSize);
}

void RGBDFrameCompressor::compress_rgbd_frame(const RGBDFrameMessage &uncompressedFrame,
                                              CompressedRGBDFrameHeaderMessage &compressedHeader,
                                              CompressedRGBDFrameMessage &compressedFrame)
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

  // Now prepare the compressed header.
  compressedHeader.set_depth_image_size(static_cast<uint32_t>(m_impl->compressedDepthBytes.size()));
  compressedHeader.set_rgb_image_size(static_cast<uint32_t>(m_impl->compressedRgbBytes.size()));

  // Finally, prepare the compressed message.
  compressedFrame.set_compressed_image_sizes(compressedHeader);
  compressedFrame.set_depth_image_data(m_impl->compressedDepthBytes);
  compressedFrame.set_rgb_image_data(m_impl->compressedRgbBytes);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RGBDFrameCompressor::uncompress_rgbd_frame(const CompressedRGBDFrameMessage &compressedFrame,
                                                RGBDFrameMessage &uncompressedFrame)
{
  // Copy metadata.
  uncompressedFrame.set_frame_index(compressedFrame.extract_frame_index());
  uncompressedFrame.set_pose(compressedFrame.extract_pose());

  // Extract the compressed byte vectors.
  compressedFrame.extract_depth_image_data(m_impl->compressedDepthBytes);
  compressedFrame.extract_rgb_image_data(m_impl->compressedRgbBytes);

  // Uncompress the images.
  uncompress_depth_image();
  uncompress_rgb_image();

  // Store the images in the uncompressed message.
  uncompressedFrame.set_depth_image(m_impl->uncompressedDepthImage);
  uncompressedFrame.set_rgb_image(m_impl->uncompressedRgbImage);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void RGBDFrameCompressor::compress_depth_image()
{
  if(m_impl->depthCompressionType == DepthCompressionType::DEPTH_COMPRESSION_PNG)
  {
#ifdef WITH_OPENCV
    cv::Mat depthWrapper(m_impl->uncompressedDepthImage->noDims.y,
                         m_impl->uncompressedDepthImage->noDims.x, CV_16SC1,
                         m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU));

    // Copy the image onto OpenCV's matrix.
    // Note the conversion to CV_16U, necessary to properly encode the image to PNG.
    depthWrapper.convertTo(m_impl->uncompressedDepthMat, CV_16U);

    // Now compress the image.
    cv::imencode(".png", m_impl->uncompressedDepthMat, m_impl->compressedDepthBytes);
#endif
  }
  else
  {
    // No compression, just copy the bytes.
    m_impl->compressedDepthBytes.resize(m_impl->uncompressedDepthImage->dataSize * sizeof(short));
    memcpy(m_impl->compressedDepthBytes.data(), m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedDepthBytes.size());
  }
}

void RGBDFrameCompressor::compress_rgb_image()
{
  if(m_impl->rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_NONE)
  {
    // No compression, just copy the bytes.
    m_impl->compressedRgbBytes.resize(m_impl->uncompressedRgbImage->dataSize * sizeof(Vector4u));
    memcpy(m_impl->compressedRgbBytes.data(), m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedRgbBytes.size());
  }
  else
  {
#ifdef WITH_OPENCV
    cv::Mat rgbWrapper(m_impl->uncompressedRgbImage->noDims.y,
                       m_impl->uncompressedRgbImage->noDims.x, CV_8UC4,
                       m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU));

    // Reorder colours and drop the alpha channel.
    cv::cvtColor(rgbWrapper, m_impl->uncompressedRgbMat, CV_RGBA2BGR);

    // Now compress the images to the appropriate format.
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
    // Decode the image (using the current Mat as a placeholder, to avoid reallocations).
    m_impl->uncompressedDepthMat = cv::imdecode(m_impl->compressedDepthBytes, cv::IMREAD_ANYDEPTH, &m_impl->uncompressedDepthMat);

    cv::Mat depthWrapper(m_impl->uncompressedDepthImage->noDims.y,
                         m_impl->uncompressedDepthImage->noDims.x, CV_16SC1,
                         m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU));

    // Copy the image to ITM's image.
    // Note the conversion of CV_16U (returned by cv::imdecode) to CV_16S.
    m_impl->uncompressedDepthMat.convertTo(depthWrapper, CV_16S);
#endif
  }
  else
  {
    // Check that the size of the output image matches  the received data.
    if(m_impl->uncompressedDepthImage->dataSize * sizeof(short) != m_impl->compressedDepthBytes.size())
    {
      throw std::runtime_error("Depth image size in the compressed message does not match the uncompressed depth image size.");
    }

    // Just copy the bytes.
    memcpy(m_impl->uncompressedDepthImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedDepthBytes.data(), m_impl->compressedDepthBytes.size());
  }
}

void RGBDFrameCompressor::uncompress_rgb_image()
{
  if(m_impl->rgbCompressionType == RGBCompressionType::RGB_COMPRESSION_NONE)
  {
    // Check that the size of the output image matches the received data.
    if(m_impl->uncompressedRgbImage->dataSize * sizeof(Vector4u) != m_impl->compressedRgbBytes.size())
    {
      throw std::runtime_error("RGB image size in the compressed message does not match the uncompressed RGB image size.");
    }

    // Just copy the bytes.
    memcpy(m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU), m_impl->compressedRgbBytes.data(), m_impl->compressedRgbBytes.size());
  }
  else
  {
#ifdef WITH_OPENCV
    // Decode the image (using the current Mat as a placeholder, to avoid reallocations).
    m_impl->uncompressedRgbMat = cv::imdecode(m_impl->compressedRgbBytes, cv::IMREAD_COLOR, &m_impl->uncompressedRgbMat);

    cv::Mat rgbWrapper(m_impl->uncompressedRgbImage->noDims.y,
                       m_impl->uncompressedRgbImage->noDims.x, CV_8UC4,
                       m_impl->uncompressedRgbImage->GetData(MEMORYDEVICE_CPU));

    // Copy the image to ITM's image, reordering the bytes and readding the alpha channel.
    cv::cvtColor(m_impl->uncompressedRgbMat, rgbWrapper, CV_BGR2RGBA);
#endif
  }
}

}
