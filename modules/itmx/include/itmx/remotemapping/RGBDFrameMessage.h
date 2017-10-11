/**
 * itmx: RGBDFrameMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_RGBDFRAMEMESSAGE
#define H_ITMX_RGBDFRAMEMESSAGE

#include <ORUtils/SE3Pose.h>

#include "MappingMessage.h"
#include "../base/ITMImagePtrTypes.h"

namespace itmx {

/**
 * \brief An instance of this class represents a message containing a single frame of RGB-D data (frame index + pose + RGB-D).
 */
class RGBDFrameMessage : public MappingMessage
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The byte segment within the message data that corresponds to the depth image. */
  Segment m_depthImageSegment;

  /** The byte segment within the message data that corresponds to the frame index. */
  Segment m_frameIndexSegment;

  /** The byte segment within the message data that corresponds to the pose. */
  Segment m_poseSegment;

  /** The byte segment within the message data that corresponds to the RGB image. */
  Segment m_rgbImageSegment;

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
  void extract_depth_image(ITMShortImage *depthImage) const;

  /**
   * \brief Extracts the frame index from the message.
   *
   * \return  The message's frame index.
   */
  int extract_frame_index() const;

  /**
   * \brief Extracts the pose from the message.
   *
   * \return  The message's pose.
   */
  ORUtils::SE3Pose extract_pose() const;

  /**
   * \brief Extracts the RGB image from the message and writes it into the specified destination image.
   *
   * \param rgbImage  The destination image into which to write the message's RGB image.
   */
  void extract_rgb_image(ITMUChar4Image *rgbImage) const;

  /**
   * \brief Copies a depth image into the appropriate byte segment in the message.
   *
   * \param depthImage  The depth image.
   */
  void set_depth_image(const ITMShortImage_CPtr& depthImage);

  /**
   * \brief Copies a frame index into the appropriate byte segment in the message.
   *
   * \param frameIndex  The frame index.
   */
  void set_frame_index(int frameIndex);

  /**
   * \brief Copies a pose into the appropriate byte segment in the message.
   *
   * \param pose  The pose.
   */
  void set_pose(const ORUtils::SE3Pose& pose);

  /**
   * \brief Copies an RGB image into the appropriate byte segment in the message.
   *
   * \param rgbImage  The RGB image.
   */
  void set_rgb_image(const ITMUChar4Image_CPtr& rgbImage);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RGBDFrameMessage> RGBDFrameMessage_Ptr;
typedef boost::shared_ptr<const RGBDFrameMessage> RGBDFrameMessage_CPtr;

}

#endif
