/**
 * itmx: BaseRGBDFrameMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/BaseRGBDFrameMessage.h"

#include <ORUtils/Math.h>

#include "remotemapping/MessageSegmentUtil.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

BaseRGBDFrameMessage::BaseRGBDFrameMessage() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int BaseRGBDFrameMessage::extract_frame_index() const
{
  return *reinterpret_cast<const int*>(&m_data[m_frameIndexSegment.first]);
}

ORUtils::SE3Pose BaseRGBDFrameMessage::extract_pose() const
{
  return MessageSegmentUtil::extract_pose(m_data, m_poseSegment);
}

void BaseRGBDFrameMessage::set_frame_index(int frameIndex)
{
  memcpy(&m_data[m_frameIndexSegment.first], reinterpret_cast<const char*>(&frameIndex), m_frameIndexSegment.second);
}

void BaseRGBDFrameMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  MessageSegmentUtil::set_pose(pose, m_data, m_poseSegment);
}

}
