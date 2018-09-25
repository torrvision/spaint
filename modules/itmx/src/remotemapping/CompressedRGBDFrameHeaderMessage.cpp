/**
 * itmx: CompressedRGBDFrameHeaderMessage.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/CompressedRGBDFrameHeaderMessage.h"

#include <cstring>

#include "remotemapping/MessageSegmentUtil.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

CompressedRGBDFrameHeaderMessage::CompressedRGBDFrameHeaderMessage()
{
  m_depthImageByteSizeSegment = std::make_pair(0, sizeof(uint32_t));
  m_depthImageSizeSegment = std::make_pair(m_depthImageByteSizeSegment.second, sizeof(Vector2i));
  m_rgbImageByteSizeSegment = std::make_pair(m_depthImageSizeSegment.first + m_depthImageSizeSegment.second, sizeof(uint32_t));
  m_rgbImageSizeSegment = std::make_pair(m_rgbImageByteSizeSegment.first + m_rgbImageByteSizeSegment.second, sizeof(Vector2i));
  m_data.resize(m_rgbImageSizeSegment.first + m_rgbImageSizeSegment.second);
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

uint32_t CompressedRGBDFrameHeaderMessage::extract_depth_image_byte_size() const
{
  return MessageSegmentUtil::extract_simple<uint32_t>(m_data, m_depthImageByteSizeSegment);
}

Vector2i CompressedRGBDFrameHeaderMessage::extract_depth_image_size() const
{
  return MessageSegmentUtil::extract_simple<Vector2i>(m_data, m_depthImageSizeSegment);
}

uint32_t CompressedRGBDFrameHeaderMessage::extract_rgb_image_byte_size() const
{
  return MessageSegmentUtil::extract_simple<uint32_t>(m_data, m_rgbImageByteSizeSegment);
}

Vector2i CompressedRGBDFrameHeaderMessage::extract_rgb_image_size() const
{
  return MessageSegmentUtil::extract_simple<Vector2i>(m_data, m_rgbImageSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_depth_image_byte_size(uint32_t depthImageByteSize)
{
  MessageSegmentUtil::set_simple(depthImageByteSize, m_data, m_depthImageByteSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_depth_image_size(const Vector2i& depthImageSize)
{
  MessageSegmentUtil::set_simple(depthImageSize, m_data, m_depthImageSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_rgb_image_byte_size(uint32_t rgbImageByteSize)
{
  MessageSegmentUtil::set_simple(rgbImageByteSize, m_data, m_rgbImageByteSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_rgb_image_size(const Vector2i& rgbImageSize)
{
  MessageSegmentUtil::set_simple(rgbImageSize, m_data, m_rgbImageSizeSegment);
}

}
