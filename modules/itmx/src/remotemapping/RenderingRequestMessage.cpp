/**
 * itmx: RenderingRequestMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "remotemapping/RenderingRequestMessage.h"

#include "remotemapping/MessageSegmentUtil.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

RenderingRequestMessage::RenderingRequestMessage()
{
  m_poseSegment = std::make_pair(0, MessageSegmentUtil::bytes_for_pose());
  m_visualisationTypeSegment = std::make_pair(m_poseSegment.first + m_poseSegment.second, sizeof(int));
  m_data.resize(m_visualisationTypeSegment.first + m_visualisationTypeSegment.second);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ORUtils::SE3Pose RenderingRequestMessage::extract_pose() const
{
  return MessageSegmentUtil::extract_pose(m_data, m_poseSegment);
}

int RenderingRequestMessage::extract_visualisation_type() const
{
  return MessageSegmentUtil::extract_int(m_data, m_visualisationTypeSegment);
}

void RenderingRequestMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  MessageSegmentUtil::set_pose(pose, m_data, m_poseSegment);
}

void RenderingRequestMessage::set_visualisation_type(int visualisationType)
{
  MessageSegmentUtil::set_int(visualisationType, m_data, m_visualisationTypeSegment);
}

}
