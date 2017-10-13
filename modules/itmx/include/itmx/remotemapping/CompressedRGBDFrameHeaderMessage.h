/**
 * itmx: CompressedRGBDFrameHeaderMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_COMPRESSEDRGBDFRAMEHEADERMESSAGE
#define H_ITMX_COMPRESSEDRGBDFRAMEHEADERMESSAGE

#include <ORUtils/SE3Pose.h>

#include "MappingMessage.h"
#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing the header for a single frame of compressed RGB-D data (size of the colour and depth chunks).
 */
class CompressedRGBDFrameHeaderMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the size in bytes of the depth image. */
  Segment m_depthSizeSegment;

  /** The byte segment within the message data that corresponds to the size in bytes of the colour image. */
  Segment m_rgbSizeSegment;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs the header for a compressed RGB-D frame message.
   */
  CompressedRGBDFrameHeaderMessage();

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes a CompressedRGBDFrameHeaderMessage.
   */
  static boost::shared_ptr<CompressedRGBDFrameHeaderMessage> make();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the size (in bytes) of the compressed depth image from the message.
   *
   * \return The size (in bytes) of the compressed depth image.
   */
  uint32_t extract_depth_image_size() const;

  /**
   * \brief Extracts the size (in bytes) of the compressed colour image from the message.
   *
   * \return The size (in bytes) of the compressed colour image.
   */
  uint32_t extract_rgb_image_size() const;

  /**
   * \brief Sets the size in bytes of the compressed depth image.
   *
   * \param depthImageSize The size in bytes of the compressed depth image.
   */
  void set_depth_image_size(uint32_t depthImageSize);

  /**
   * \brief Sets the size in bytes of the compressed colour image.
   *
   * \param rgbImageSize The size in bytes of the compressed colour image.
   */
  void set_rgb_image_size(uint32_t rgbImageSize);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<CompressedRGBDFrameHeaderMessage> CompressedRGBDFrameHeaderMessage_Ptr;
typedef boost::shared_ptr<const CompressedRGBDFrameHeaderMessage> CompressedRGBDFrameHeaderMessage_CPtr;

}

#endif
