/**
 * itmx: RenderingRequestMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "remotemapping/RenderingRequestMessage.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

RenderingRequestMessage::RenderingRequestMessage()
{
  m_imageSizeSegment = std::make_pair(0, sizeof(Vector2i));
  m_poseSegment = std::make_pair(m_imageSizeSegment.first + m_imageSizeSegment.second, bytes_for_pose());
  m_visualisationTypeSegment = std::make_pair(m_poseSegment.first + m_poseSegment.second, sizeof(int));
  m_data.resize(m_visualisationTypeSegment.first + m_visualisationTypeSegment.second);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

Vector2i RenderingRequestMessage::extract_image_size() const
{
  return read_simple<Vector2i>(m_imageSizeSegment);
}

ORUtils::SE3Pose RenderingRequestMessage::extract_pose() const
{
  return read_pose(m_poseSegment);
}

int RenderingRequestMessage::extract_visualisation_type() const
{
  return read_simple<int>(m_visualisationTypeSegment);
}

void RenderingRequestMessage::set_image_size(const Vector2i& imgSize)
{
  write_simple(imgSize, m_imageSizeSegment);
}

void RenderingRequestMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  write_pose(pose, m_poseSegment);
}

void RenderingRequestMessage::set_visualisation_type(int visualisationType)
{
  write_simple(visualisationType, m_visualisationTypeSegment);
}

}
