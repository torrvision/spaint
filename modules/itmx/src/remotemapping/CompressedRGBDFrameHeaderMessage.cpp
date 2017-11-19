/**
 * itmx: CompressedRGBDFrameHeaderMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/CompressedRGBDFrameHeaderMessage.h"

#include <cstring>

namespace itmx {

//#################### CONSTRUCTORS ####################

CompressedRGBDFrameHeaderMessage::CompressedRGBDFrameHeaderMessage()
{
  m_depthImageSizeSegment = std::make_pair(0, sizeof(uint32_t));
  m_rgbImageSizeSegment = std::make_pair(m_depthImageSizeSegment.second, sizeof(uint32_t));
  m_data.resize(m_rgbImageSizeSegment.first + m_rgbImageSizeSegment.second);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

uint32_t CompressedRGBDFrameHeaderMessage::extract_depth_image_size() const
{
  return *reinterpret_cast<const uint32_t*>(&m_data[m_depthImageSizeSegment.first]);
}

uint32_t CompressedRGBDFrameHeaderMessage::extract_rgb_image_size() const
{
  return *reinterpret_cast<const uint32_t*>(&m_data[m_rgbImageSizeSegment.first]);
}

void CompressedRGBDFrameHeaderMessage::set_depth_image_size(uint32_t depthImageSize)
{
  memcpy(&m_data[m_depthImageSizeSegment.first], reinterpret_cast<const char*>(&depthImageSize), m_depthImageSizeSegment.second);
}

void CompressedRGBDFrameHeaderMessage::set_rgb_image_size(uint32_t rgbImageSize)
{
  memcpy(&m_data[m_rgbImageSizeSegment.first], reinterpret_cast<const char*>(&rgbImageSize), m_rgbImageSizeSegment.second);
}

}
