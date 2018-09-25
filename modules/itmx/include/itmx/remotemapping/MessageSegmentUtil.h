/**
 * itmx: MessageSegmentUtil.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_ITMX_MESSAGESEGMENTUTIL
#define H_ITMX_MESSAGESEGMENTUTIL

#include <ORUtils/SE3Pose.h>

#include <tvgutil/net/Message.h>

namespace itmx {

/**
 * \brief This struct provides functions that can be used to read and write message segments.
 */
struct MessageSegmentUtil
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief TODO
   */
  static size_t bytes_for_pose();

  /**
   * \brief TODO
   */
  static ORUtils::SE3Pose extract_pose(const std::vector<char>& data, const tvgutil::Message::Segment& poseSegment);

  /**
   * \brief TODO
   */
  static void set_pose(const ORUtils::SE3Pose& pose, std::vector<char>& data, const tvgutil::Message::Segment& poseSegment);
};

}

#endif
