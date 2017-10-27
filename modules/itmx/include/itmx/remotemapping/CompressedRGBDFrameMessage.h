/**
 * itmx: CompressedRGBDFrameMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_COMPRESSEDRGBDFRAMEMESSAGE
#define H_ITMX_COMPRESSEDRGBDFRAMEMESSAGE

#include "BaseRGBDFrameMessage.h"
#include "CompressedRGBDFrameHeaderMessage.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing a single frame of compressed RGB-D data (frame index + pose + RGB-D).
 */
class CompressedRGBDFrameMessage : public BaseRGBDFrameMessage
{
  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a compressed RGB-D frame message.
   *
   * \param headerMsg The header message corresponding to this message, which specifies the size of the compressed depth and RGB segments.
   */
  CompressedRGBDFrameMessage(const CompressedRGBDFrameHeaderMessage& headerMsg);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the compressed depth image data from the message and writes it into the specified destination vector.
   *
   * \param depthImageData  The vector into which to write the message's compressed depth image data. It will be resized as necessary.
   */
  void extract_depth_image_data(std::vector<uint8_t>& depthImageData) const;

  /**
   * \brief Extracts the compressed RGB image data from the message and writes it into the specified destination vector.
   *
   * \param rgbImageData  The vector into which to write the message's compressed RGB image data. It will be resized as necessary.
   */
  void extract_rgb_image_data(std::vector<uint8_t>& rgbImageData) const;

  /**
   * \brief Sets the segment sizes for the depth and RGB images according to the compressed message header. Resizes the raw data storage accordingly.
   *
   * \param headerMsg The header message corresponding to this message, which specifies the size of the compressed depth and RGB segments.
   */
  void set_compressed_image_sizes(const CompressedRGBDFrameHeaderMessage& headerMsg);

  /**
   * \brief Copies a compressed depth image into the appropriate byte segment in the message.
   *
   * \param depthImageData  The compressed depth image data.
   */
  void set_depth_image_data(const std::vector<uint8_t>& depthImageData);

  /**
   * \brief Copies a compressed RGB image into the appropriate byte segment in the message.
   *
   * \param rgbImage  The compressed RGB image data.
   */
  void set_rgb_image_data(const std::vector<uint8_t>& rgbImageData);
};

}

#endif
