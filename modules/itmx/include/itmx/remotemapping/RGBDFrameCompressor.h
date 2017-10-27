/**
 * itmx: RGBDFrameCompressor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RGBDFRAMECOMPRESSOR
#define H_ITMX_RGBDFRAMECOMPRESSOR

#include "CompressedRGBDFrameHeaderMessage.h"
#include "CompressedRGBDFrameMessage.h"
#include "DepthCompressionType.h"
#include "RGBCompressionType.h"
#include "RGBDFrameMessage.h"

namespace itmx {

/**
 * \brief An instance of this class can be used to compress or decompress RGB-D frame messages.
 */
class RGBDFrameCompressor
{
  //#################### NESTED TYPES ####################
private:
  /** Forward declare a nested structure holding private implementation data. */
  struct Impl;

  //#################### PRIVATE VARIABLES ####################
private:
  /** A pointer to the implementation details. */
  boost::shared_ptr<Impl> m_impl;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an RGB-D frame compressor.
   *
   * \param rgbImageSize          The size of the RGB images to be compressed.
   * \param depthImageSize        The size of the depth images to be compressed.
   * \param rgbCompressionType    The type of compression to apply to the RGB images.
   * \param depthCompressionType  The type of compression to apply to the depth images.
   *
   * \throws std::invalid_argument  If the specified compression types cannot be used (e.g. when building without OpenCV).
   */
  RGBDFrameCompressor(const Vector2i& rgbImageSize, const Vector2i& depthImageSize,
                      RGBCompressionType rgbCompressionType = RGB_COMPRESSION_NONE,
                      DepthCompressionType depthCompressionType = DEPTH_COMPRESSION_NONE);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Compresses an RGB-D frame message.
   *
   * \param uncompressedFrame  The message to compress.
   * \param compressedHeader   Will contain the header data for the compressed RGB-D frame.
   * \param compressedFrame    Will contain the compressed RGB-D frame data.
   */
  void compress_rgbd_frame(const RGBDFrameMessage& uncompressedFrame, CompressedRGBDFrameHeaderMessage& compressedHeader, CompressedRGBDFrameMessage& compressedFrame);

  /**
   * \brief Uncompresses an RGB-D frame message.
   *
   * \param compressedFrame    The compressed frame message.
   * \param uncompressedFrame  Will contain the uncompressed message.
   */
  void uncompress_rgbd_frame(const CompressedRGBDFrameMessage& compressedFrame, RGBDFrameMessage& uncompressedFrame);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Compresses the uncompressed depth image on which we are currently working.
   */
  void compress_depth_image();

  /**
   * \brief Compresses the uncompressed RGB image on which we are currently working.
   */
  void compress_rgb_image();

  /**
   * \brief Uncompresses the compressed depth image on which we are currently working.
   */
  void uncompress_depth_image();

  /**
   * \brief Uncompresses the compressed RGB image on which we are currently working.
   */
  void uncompress_rgb_image();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RGBDFrameCompressor> RGBDFrameCompressor_Ptr;
typedef boost::shared_ptr<const RGBDFrameCompressor> RGBDFrameCompressor_CPtr;

}

#endif
