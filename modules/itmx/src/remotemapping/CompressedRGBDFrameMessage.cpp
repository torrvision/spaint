/**
 * itmx: CompressedRGBDFrameMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/CompressedRGBDFrameMessage.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

CompressedRGBDFrameMessage::CompressedRGBDFrameMessage(const CompressedRGBDFrameHeaderMessage& messageHeader)
{
  // Frame index and pose have fixed size.
  m_frameIndexSegment = std::make_pair(0, sizeof(int));
  m_poseSegment = std::make_pair(m_frameIndexSegment.second, sizeof(Matrix4f) + 6 * sizeof(float));

  // Depth and RGB segments depend on the message header.
  set_compressed_image_sizes(messageHeader);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

CompressedRGBDFrameMessage_Ptr CompressedRGBDFrameMessage::make(const CompressedRGBDFrameHeaderMessage& messageHeader)
{
  return CompressedRGBDFrameMessage_Ptr(new CompressedRGBDFrameMessage(messageHeader));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CompressedRGBDFrameMessage::extract_depth_image_data(std::vector<uint8_t> &depthImageData) const
{
  depthImageData.resize(m_depthImageSegment.second);
  printf("Resized depth to %d\n", depthImageData.size());
  memcpy(reinterpret_cast<char *>(depthImageData.data()), &m_data[m_depthImageSegment.first], m_depthImageSegment.second);
}

int CompressedRGBDFrameMessage::extract_frame_index() const
{
  return *reinterpret_cast<const int*>(&m_data[m_frameIndexSegment.first]);
}

ORUtils::SE3Pose CompressedRGBDFrameMessage::extract_pose() const
{
  ORUtils::SE3Pose pose;

  Matrix4f M = *reinterpret_cast<const Matrix4f*>(&m_data[m_poseSegment.first]);

  float params[6];
  memcpy(params, &m_data[m_poseSegment.first + sizeof(Matrix4f)], 6 * sizeof(float));

  pose.SetBoth(M, params);
  return pose;
}

void CompressedRGBDFrameMessage::extract_rgb_image_data(std::vector<uint8_t> &rgbImageData) const
{
  rgbImageData.resize(m_rgbImageSegment.second);
  printf("Resized rgb to %d\n", rgbImageData.size());
  memcpy(reinterpret_cast<char *>(rgbImageData.data()), &m_data[m_rgbImageSegment.first], m_rgbImageSegment.second);
}

void CompressedRGBDFrameMessage::set_depth_image_data(const std::vector<uint8_t> &depthImageData)
{
  if(depthImageData.size() != m_depthImageSegment.second)
  {
    throw std::runtime_error("Error: The compressed source depth image has a different size to that of the depth segment in the message.");
  }

  memcpy(&m_data[m_depthImageSegment.first], reinterpret_cast<const char *>(depthImageData.data()), m_depthImageSegment.second);
}

void CompressedRGBDFrameMessage::set_frame_index(int frameIndex)
{
  memcpy(&m_data[m_frameIndexSegment.first], reinterpret_cast<const char*>(&frameIndex), m_frameIndexSegment.second);
}

void CompressedRGBDFrameMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  memcpy(&m_data[m_poseSegment.first], reinterpret_cast<const char*>(&pose.GetM()), sizeof(Matrix4f));
  memcpy(&m_data[m_poseSegment.first + sizeof(Matrix4f)], reinterpret_cast<const char*>(pose.GetParams()), 6 * sizeof(float));
}

void CompressedRGBDFrameMessage::set_compressed_image_sizes(const CompressedRGBDFrameHeaderMessage &messageHeader)
{
  m_depthImageSegment = std::make_pair(m_poseSegment.first + m_poseSegment.second, messageHeader.extract_depth_image_size());
  m_rgbImageSegment = std::make_pair(m_depthImageSegment.first + m_depthImageSegment.second, messageHeader.extract_rgb_image_size());

  m_data.resize(m_rgbImageSegment.first + m_rgbImageSegment.second);
}

void CompressedRGBDFrameMessage::set_rgb_image_data(const std::vector<uint8_t> &rgbImageData)
{
  if(rgbImageData.size() != m_rgbImageSegment.second)
  {
    throw std::runtime_error("Error: The compressed source RGB image has a different size to that of the depth segment in the message.");
  }

  memcpy(&m_data[m_rgbImageSegment.first], reinterpret_cast<const char *>(rgbImageData.data()), m_rgbImageSegment.second);
}

}
