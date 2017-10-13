/**
 * itmx: RGBDFrameCompressor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RGBDFRAMECOMPRESSOR
#define H_ITMX_RGBDFRAMECOMPRESSOR

#include <opencv2/core.hpp>

#include "RGBDFrameMessage.h"
#include "CompressedRGBDFrameMessage.h"
#include "CompressedRGBDFrameHeaderMessage.h"

namespace itmx {

class RGBDFrameCompressor
{
public:
  RGBDFrameCompressor(const Vector2i& rgbImageSize, const Vector2i& depthImageSize);

  void compress_rgbd_frame(const RGBDFrameMessage& uncompressedFrame, CompressedRGBDFrameHeaderMessage& compressedHeader, CompressedRGBDFrameMessage& compressedFrame);

  void uncompress_rgbd_frame(const CompressedRGBDFrameMessage& compressedFrame, RGBDFrameMessage& uncompressedFrame);

private:

  std::vector<uint8_t> m_compressedDepthBytes;
  std::vector<uint8_t> m_compressedRgbBytes;

  ITMShortImage_Ptr m_uncompressedDepthImage;
  ITMUChar4Image_Ptr m_uncompressedRgbImage;

  cv::Mat m_uncompressedDepthMat;
  cv::Mat m_uncompressedRgbMat;
};

typedef boost::shared_ptr<RGBDFrameCompressor> RGBDFrameCompressor_Ptr;
typedef boost::shared_ptr<const RGBDFrameCompressor> RGBDFrameCompressor_CPtr;

}

#endif
