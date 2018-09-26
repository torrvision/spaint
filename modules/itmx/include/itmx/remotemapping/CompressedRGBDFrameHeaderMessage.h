/**
 * itmx: CompressedRGBDFrameHeaderMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_COMPRESSEDRGBDFRAMEHEADERMESSAGE
#define H_ITMX_COMPRESSEDRGBDFRAMEHEADERMESSAGE

#include <boost/cstdint.hpp>

#include <ORUtils/Math.h>

#include "MappingMessage.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing the sizes (in bytes) and dimensions of the compressed images for a single RGB-D frame.
 */
class CompressedRGBDFrameHeaderMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the size in bytes of the compressed depth image. */
  Segment m_depthImageByteSizeSegment;

  /** The byte segment within the message data that corresponds to the dimensions of the compressed depth image. */
  Segment m_depthImageSizeSegment;

  /** The byte segment within the message data that corresponds to the size in bytes of the compressed RGB image. */
  Segment m_rgbImageByteSizeSegment;

  /** The byte segment within the message data that corresponds to the dimensions of the compressed RGB image. */
  Segment m_rgbImageSizeSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a compressed RGB-D frame header message.
   */
  CompressedRGBDFrameHeaderMessage();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the size (in bytes) of the compressed depth image from the message.
   *
   * \return The size (in bytes) of the compressed depth image.
   */
  uint32_t extract_depth_image_byte_size() const;

  /**
   * \brief Extracts the dimensions of the compressed depth image from the message.
   *
   * \return  The dimensions of the compressed depth image.
   */
  Vector2i extract_depth_image_size() const;

  /**
   * \brief Extracts the size (in bytes) of the compressed RGB image from the message.
   *
   * \return The size (in bytes) of the compressed RGB image.
   */
  uint32_t extract_rgb_image_byte_size() const;

  /**
   * \brief Extracts the dimensions of the compressed RGB image from the message.
   *
   * \return  The dimensions of the compressed RGB image.
   */
  Vector2i extract_rgb_image_size() const;

  /**
   * \brief Sets the size (in bytes) of the compressed depth image.
   *
   * \param depthImageByteSize  The size (in bytes) of the compressed depth image.
   */
  void set_depth_image_byte_size(uint32_t depthImageByteSize);

  /**
   * \brief Sets the dimensions of the compressed depth image.
   *
   * \param depthImageSize  The dimensions of the compressed depth image.
   */
  void set_depth_image_size(const Vector2i& depthImageSize);

  /**
   * \brief Sets the size (in bytes) of the compressed RGB image.
   *
   * \param rgbImageByteSize  The size (in bytes) of the compressed RGB image.
   */
  void set_rgb_image_byte_size(uint32_t rgbImageByteSize);

  /**
   * \brief Sets the dimensions of the compressed RGB image.
   *
   * \param rgbImageSize  The dimensions of the compressed RGB image.
   */
  void set_rgb_image_size(const Vector2i& rgbImageSize);
};

}

#endif
