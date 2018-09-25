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
  //#################### TYPEDEFS ####################

  typedef tvgutil::Message::Segment Segment;

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief TODO
   */
  static size_t bytes_for_pose();

  /**
   * \brief TODO
   */
  template <typename T>
  static T extract_simple(const std::vector<char>& data, const Segment& segment)
  {
    return *reinterpret_cast<const T*>(&data[segment.first]);
  }

  /**
   * \brief TODO
   */
  static ORUtils::SE3Pose extract_pose(const std::vector<char>& data, const Segment& segment);

  /**
   * \brief TODO
   */
  template <typename T>
  static void set_simple(const T& t, std::vector<char>& data, const Segment& segment)
  {
    memcpy(&data[segment.first], reinterpret_cast<const char*>(&t), segment.second);
  }

  /**
   * \brief TODO
   */
  static void set_pose(const ORUtils::SE3Pose& pose, std::vector<char>& data, const Segment& segment);
};

}

#endif
