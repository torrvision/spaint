/**
 * itmx: RGBDFrameMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RGBDFRAMEMESSAGE
#define H_ITMX_RGBDFRAMEMESSAGE

#include "BaseRGBDFrameMessage.h"
#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing a single frame of RGB-D data (frame index + pose + RGB-D).
 */
class RGBDFrameMessage : public BaseRGBDFrameMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The size of the frame's depth image. */
  Vector2i m_depthImageSize;

  /** The size of the frame's RGB image. */
  Vector2i m_rgbImageSize;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an RGB-D frame message.
   *
   * \param rgbImageSize    The size of the frame's RGB image.
   * \param depthImageSize  The size of the frame's depth image.
   */
  RGBDFrameMessage(const Vector2i& rgbImageSize, const Vector2i& depthImageSize);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Makes an RGB-D frame message.
   *
   * \param rgbImageSize    The size of the frame's RGB image.
   * \param depthImageSize  The size of the frame's depth image.
   */
  static boost::shared_ptr<RGBDFrameMessage> make(const Vector2i& rgbImageSize, const Vector2i& depthImageSize);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Extracts the depth image from the message and writes it into the specified destination image.
   *
   * \param depthImage  The destination image into which to write the message's depth image.
   */
  void extract_depth_image(ORShortImage *depthImage) const;

  /**
   * \brief Extracts the RGB image from the message and writes it into the specified destination image.
   *
   * \param rgbImage  The destination image into which to write the message's RGB image.
   */
  void extract_rgb_image(ORUChar4Image *rgbImage) const;

  /**
   * \brief Copies a depth image into the appropriate byte segment in the message.
   *
   * \param depthImage  The depth image.
   */
  void set_depth_image(const ORShortImage_CPtr& depthImage);

  /**
   * \brief Copies an RGB image into the appropriate byte segment in the message.
   *
   * \param rgbImage  The RGB image.
   */
  void set_rgb_image(const ORUChar4Image_CPtr& rgbImage);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RGBDFrameMessage> RGBDFrameMessage_Ptr;
typedef boost::shared_ptr<const RGBDFrameMessage> RGBDFrameMessage_CPtr;

}

#endif
