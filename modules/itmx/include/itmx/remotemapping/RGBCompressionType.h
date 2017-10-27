/**
 * itmx: RGBCompressionType.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RGBCOMPRESSIONTYPE
#define H_ITMX_RGBCOMPRESSIONTYPE

namespace itmx {

/**
 * \brief The values of this enumeration can be used to specify a compression mode for RGB images.
 */
enum RGBCompressionType
{
  /** The RGB images will be compressed using lossy JPG compression (requires OpenCV). */
  RGB_COMPRESSION_JPG,

  /** The RGB images will not be compressed. */
  RGB_COMPRESSION_NONE,

  /** The RGB images will be compressed using lossless PNG compression (requires OpenCV). */
  RGB_COMPRESSION_PNG,
};

}

#endif
