/**
 * itmx: BaseRGBDFrameMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/BaseRGBDFrameMessage.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

BaseRGBDFrameMessage::BaseRGBDFrameMessage() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int BaseRGBDFrameMessage::extract_frame_index() const
{
  return read_simple<int>(m_frameIndexSegment);
}

ORUtils::SE3Pose BaseRGBDFrameMessage::extract_pose() const
{
  return read_pose(m_poseSegment);
}

void BaseRGBDFrameMessage::set_frame_index(int frameIndex)
{
  write_simple(frameIndex, m_frameIndexSegment);
}

void BaseRGBDFrameMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  write_pose(pose, m_poseSegment);
}

}
