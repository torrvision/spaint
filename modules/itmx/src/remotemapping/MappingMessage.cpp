/**
 * itmx: MappingMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/MappingMessage.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingMessage::MappingMessage(const Vector2i& rgbImageSize, const Vector2i& depthImageSize)
{
  const int se3ParamCount = 6;
  m_frameIndexSegment = std::make_pair(0, sizeof(int));
  m_poseSegment = std::make_pair(m_frameIndexSegment.second, se3ParamCount * sizeof(float));
  m_rgbImageSegment = std::make_pair(m_poseSegment.first + m_poseSegment.second, rgbImageSize.width * rgbImageSize.height * sizeof(Vector4u));
  m_depthImageSegment = std::make_pair(m_rgbImageSegment.first + m_rgbImageSegment.second, depthImageSize.width * depthImageSize.height * sizeof(short));
  m_data.resize(m_depthImageSegment.first + m_depthImageSegment.second);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

char *MappingMessage::get_data_ptr()
{
  return &m_data[0];
}

const char *MappingMessage::get_data_ptr() const
{
  return &m_data[0];
}

size_t MappingMessage::get_size() const
{
  return m_data.size();
}

void MappingMessage::extract_depth_image(ITMShortImage *depthImage) const
{
  if(depthImage->dataSize != m_depthImageSegment.second)
  {
    throw std::runtime_error("Error: The target image has a different size to that of the depth image in the message");
  }

  memcpy(reinterpret_cast<char*>(depthImage->GetData(MEMORYDEVICE_CPU)), &m_data[m_depthImageSegment.first], m_depthImageSegment.second);
}

int MappingMessage::extract_frame_index() const
{
  return *reinterpret_cast<const int*>(&m_data[m_frameIndexSegment.first]);
}

ORUtils::SE3Pose MappingMessage::extract_pose() const
{
  ORUtils::SE3Pose pose;
  pose.SetFrom(reinterpret_cast<const float*>(&m_data[m_poseSegment.first]));
  return pose;
}

void MappingMessage::extract_rgb_image(ITMUChar4Image *rgbImage) const
{
  if(rgbImage->dataSize != m_rgbImageSegment.second)
  {
    throw std::runtime_error("Error: The target image has a different size to that of the RGB image in the message");
  }

  memcpy(reinterpret_cast<char*>(rgbImage->GetData(MEMORYDEVICE_CPU)), &m_data[m_rgbImageSegment.first], m_rgbImageSegment.second);
}

void MappingMessage::set_depth_image(const ITMShortImage_CPtr& depthImage)
{
  memcpy(&m_data[m_depthImageSegment.first], reinterpret_cast<const char*>(depthImage->GetData(MEMORYDEVICE_CPU)), m_depthImageSegment.second);
}

void MappingMessage::set_frame_index(int frameIndex)
{
  memcpy(&m_data[m_frameIndexSegment.first], reinterpret_cast<const char*>(&frameIndex), m_frameIndexSegment.second);
}

void MappingMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  memcpy(&m_data[m_poseSegment.first], reinterpret_cast<const char*>(pose.GetParams()), m_poseSegment.second);
}

void MappingMessage::set_rgb_image(const ITMUChar4Image_CPtr& rgbImage)
{
  memcpy(&m_data[m_rgbImageSegment.first], reinterpret_cast<const char*>(rgbImage->GetData(MEMORYDEVICE_CPU)), m_rgbImageSegment.second);
}

}
