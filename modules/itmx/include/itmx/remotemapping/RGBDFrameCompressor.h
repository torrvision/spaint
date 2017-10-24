/**
 * itmx: RGBDFrameCompressor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RGBDFRAMECOMPRESSOR
#define H_ITMX_RGBDFRAMECOMPRESSOR

#include "CompressedRGBDFrameMessage.h"
#include "CompressedRGBDFrameHeaderMessage.h"
#include "RGBDCalibrationMessage.h"
#include "RGBDFrameMessage.h"

namespace itmx {

/**
 * \brief An instance of this class allows the compression and decompression of RGB-D frame messages.
 *        Multiple compression types can be enabled with the DepthCompressionType and RGBCompressionType enums.
 */
class RGBDFrameCompressor
{
  //#################### PRIVATE NESTED TYPES ####################
private:
  /** Forward declare a nested structure holding private implementation data. */
  struct Impl;

  //#################### CONSTRUCTOR ####################
public:
  /**
   * \brief Constructs an instance of a RGBDFrameCompressor.
   *
   * \param rgbImageSize          The size of the RGB images to be compressed.
   * \param depthImageSize        The size of the Depth images to be compressed.
   * \param depthCompressionType  The type of compression to apply to the depth images.
   * \param rgbCompressionType    The type of compression to apply to the colour images.
   *
   * \throws std::invalid_argument  If the specified compression types cannot be used (e.g. when building without OpenCV).
   */
  RGBDFrameCompressor(const Vector2i& rgbImageSize,
                      const Vector2i& depthImageSize,
                      DepthCompressionType depthCompressionType = DEPTH_COMPRESSION_NONE,
                      RGBCompressionType rgbCompressionType = RGB_COMPRESSION_NONE);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Compress an RGB-D frame message.
   *
   * \param uncompressedFrame  The message to compress.
   * \param compressedHeader   Will contain the header of the compressed frame message.
   * \param compressedFrame    Will contain the compressed RGB-D frame message.
   */
  void compress_rgbd_frame(const RGBDFrameMessage& uncompressedFrame,
                           CompressedRGBDFrameHeaderMessage& compressedHeader,
                           CompressedRGBDFrameMessage& compressedFrame);

  /**
   * \brief Uncompress an RGB-D frame message.
   *
   * \param compressedFrame    The compressed frame message.
   * \param uncompressedFrame  Will contain the uncompressed message.
   */
  void uncompress_rgbd_frame(const CompressedRGBDFrameMessage& compressedFrame, RGBDFrameMessage& uncompressedFrame);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Compress a depth image.
   */
  void compress_depth_image();

  /**
   * \brief Compress a colour image.
   */
  void compress_rgb_image();

  /**
   * \brief Uncompress a depth image.
   */
  void uncompress_depth_image();

  /**
   * \brief Uncompress a colour image.
   */
  void uncompress_rgb_image();

  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** Pointer to the implementation details. */
  boost::shared_ptr<Impl> m_impl;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RGBDFrameCompressor> RGBDFrameCompressor_Ptr;
typedef boost::shared_ptr<const RGBDFrameCompressor> RGBDFrameCompressor_CPtr;

}

#endif
