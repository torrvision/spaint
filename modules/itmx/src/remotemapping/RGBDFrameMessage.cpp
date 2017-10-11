/**
 * itmx: RGBDFrameMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/RGBDFrameMessage.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

RGBDFrameMessage::RGBDFrameMessage(const Vector2i& rgbImageSize, const Vector2i& depthImageSize)
{
  m_frameIndexSegment = std::make_pair(0, sizeof(int));
  m_poseSegment = std::make_pair(m_frameIndexSegment.second, sizeof(Matrix4f) + 6 * sizeof(float));
  m_rgbImageSegment = std::make_pair(m_poseSegment.first + m_poseSegment.second, rgbImageSize.width * rgbImageSize.height * sizeof(Vector4u));
  m_depthImageSegment = std::make_pair(m_rgbImageSegment.first + m_rgbImageSegment.second, depthImageSize.width * depthImageSize.height * sizeof(short));
  m_data.resize(m_depthImageSegment.first + m_depthImageSegment.second);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

RGBDFrameMessage_Ptr RGBDFrameMessage::make(const Vector2i& rgbImageSize, const Vector2i& depthImageSize)
{
  return RGBDFrameMessage_Ptr(new RGBDFrameMessage(rgbImageSize, depthImageSize));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void RGBDFrameMessage::extract_depth_image(ITMShortImage *depthImage) const
{
  if(depthImage->dataSize * sizeof(short) != m_depthImageSegment.second)
  {
    throw std::runtime_error("Error: The target image has a different size to that of the depth image in the message");
  }

  memcpy(reinterpret_cast<char*>(depthImage->GetData(MEMORYDEVICE_CPU)), &m_data[m_depthImageSegment.first], m_depthImageSegment.second);
}

int RGBDFrameMessage::extract_frame_index() const
{
  return *reinterpret_cast<const int*>(&m_data[m_frameIndexSegment.first]);
}

ORUtils::SE3Pose RGBDFrameMessage::extract_pose() const
{
  ORUtils::SE3Pose pose;

  Matrix4f M = *reinterpret_cast<const Matrix4f*>(&m_data[m_poseSegment.first]);

  float params[6];
  memcpy(params, &m_data[m_poseSegment.first + sizeof(Matrix4f)], 6 * sizeof(float));

  pose.SetBoth(M, params);
  return pose;
}

void RGBDFrameMessage::extract_rgb_image(ITMUChar4Image *rgbImage) const
{
  if(rgbImage->dataSize * sizeof(Vector4u) != m_rgbImageSegment.second)
  {
    throw std::runtime_error("Error: The target image has a different size to that of the RGB image in the message");
  }

  memcpy(reinterpret_cast<char*>(rgbImage->GetData(MEMORYDEVICE_CPU)), &m_data[m_rgbImageSegment.first], m_rgbImageSegment.second);
}

void RGBDFrameMessage::set_depth_image(const ITMShortImage_CPtr& depthImage)
{
  memcpy(&m_data[m_depthImageSegment.first], reinterpret_cast<const char*>(depthImage->GetData(MEMORYDEVICE_CPU)), m_depthImageSegment.second);
}

void RGBDFrameMessage::set_frame_index(int frameIndex)
{
  memcpy(&m_data[m_frameIndexSegment.first], reinterpret_cast<const char*>(&frameIndex), m_frameIndexSegment.second);
}

void RGBDFrameMessage::set_pose(const ORUtils::SE3Pose& pose)
{
  memcpy(&m_data[m_poseSegment.first], reinterpret_cast<const char*>(&pose.GetM()), sizeof(Matrix4f));
  memcpy(&m_data[m_poseSegment.first + sizeof(Matrix4f)], reinterpret_cast<const char*>(pose.GetParams()), 6 * sizeof(float));
}

void RGBDFrameMessage::set_rgb_image(const ITMUChar4Image_CPtr& rgbImage)
{
  memcpy(&m_data[m_rgbImageSegment.first], reinterpret_cast<const char*>(rgbImage->GetData(MEMORYDEVICE_CPU)), m_rgbImageSegment.second);
}

}
