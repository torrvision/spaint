/**
 * itmx: BaseRGBDFrameMessage.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_BASERGBDFRAMEMESSAGE
#define H_ITMX_BASERGBDFRAMEMESSAGE

#include <ORUtils/SE3Pose.h>

#include "MappingMessage.h"

namespace itmx {

/**
 * \brief An instance of a class deriving from this one represents a message containing a single frame of RGB-D data (frame index + pose + RGB-D).
 */
class BaseRGBDFrameMessage : public MappingMessage
{
  //#################### PROTECTED VARIABLES ####################
protected:
  /** The byte segment within the message data that corresponds to the depth image. */
  Segment m_depthImageSegment;

  /** The byte segment within the message data that corresponds to the frame index. */
  Segment m_frameIndexSegment;

  /** The byte segment within the message data that corresponds to the pose. */
  Segment m_poseSegment;

  /** The byte segment within the message data that corresponds to the RGB image. */
  Segment m_rgbImageSegment;

  //#################### CONSTRUCTORS ####################
protected:
  // Deliberately protected to prevent direct instantiation of this class.
  BaseRGBDFrameMessage();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
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
};

}

#endif
