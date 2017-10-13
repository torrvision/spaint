/**
 * itmx: CompressedRGBDFrameHeaderMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/CompressedRGBDFrameHeaderMessage.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

CompressedRGBDFrameHeaderMessage::CompressedRGBDFrameHeaderMessage()
{
  m_depthSizeSegment = std::make_pair(0, sizeof(uint32_t));
  m_rgbSizeSegment = std::make_pair(m_depthSizeSegment.second, sizeof(uint32_t));
  m_data.resize(m_rgbSizeSegment.first + m_rgbSizeSegment.second);
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

CompressedRGBDFrameHeaderMessage_Ptr CompressedRGBDFrameHeaderMessage::make()
{
  return CompressedRGBDFrameHeaderMessage_Ptr(new CompressedRGBDFrameHeaderMessage());
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

uint32_t CompressedRGBDFrameHeaderMessage::extract_depth_image_size() const
{
  return *reinterpret_cast<const uint32_t *>(&m_data[m_depthSizeSegment.first]);
}

uint32_t CompressedRGBDFrameHeaderMessage::extract_rgb_image_size() const
{
  return *reinterpret_cast<const uint32_t *>(&m_data[m_rgbSizeSegment.first]);
}

void CompressedRGBDFrameHeaderMessage::set_depth_image_size(uint32_t depthImageSize)
{
  memcpy(&m_data[m_depthSizeSegment.first], reinterpret_cast<const char *>(&depthImageSize), m_depthSizeSegment.second);
}

void CompressedRGBDFrameHeaderMessage::set_rgb_image_size(uint32_t rgbImageSize)
{
  memcpy(&m_data[m_rgbSizeSegment.first], reinterpret_cast<const char *>(&rgbImageSize), m_rgbSizeSegment.second);
}

}
