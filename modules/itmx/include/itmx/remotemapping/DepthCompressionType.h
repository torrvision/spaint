/**
 * itmx: DepthCompressionType.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_DEPTHCOMPRESSIONTYPE
#define H_ITMX_DEPTHCOMPRESSIONTYPE

namespace itmx {

/**
 * \brief The values of this enumeration can be used to specify a compression mode for depth images.
 */
enum DepthCompressionType
{
  /** The depth images will not be compressed. */
  DEPTH_COMPRESSION_NONE,

  /** The depth images will be compressed using lossless PNG compression (requires OpenCV). */
  DEPTH_COMPRESSION_PNG,
};

}

#endif
