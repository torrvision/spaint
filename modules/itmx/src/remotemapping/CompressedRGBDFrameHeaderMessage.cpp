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
  m_depthImageByteSizeSegment = std::make_pair(0, sizeof(uint32_t));
  m_depthImageSizeSegment = std::make_pair(end_of(m_depthImageByteSizeSegment), sizeof(Vector2i));
  m_rgbImageByteSizeSegment = std::make_pair(end_of(m_depthImageSizeSegment), sizeof(uint32_t));
  m_rgbImageSizeSegment = std::make_pair(end_of(m_rgbImageByteSizeSegment), sizeof(Vector2i));
  m_data.resize(end_of(m_rgbImageSizeSegment));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

uint32_t CompressedRGBDFrameHeaderMessage::extract_depth_image_byte_size() const
{
  return read_simple<uint32_t>(m_depthImageByteSizeSegment);
}

Vector2i CompressedRGBDFrameHeaderMessage::extract_depth_image_size() const
{
  return read_simple<Vector2i>(m_depthImageSizeSegment);
}

uint32_t CompressedRGBDFrameHeaderMessage::extract_rgb_image_byte_size() const
{
  return read_simple<uint32_t>(m_rgbImageByteSizeSegment);
}

Vector2i CompressedRGBDFrameHeaderMessage::extract_rgb_image_size() const
{
  return read_simple<Vector2i>(m_rgbImageSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_depth_image_byte_size(uint32_t depthImageByteSize)
{
  write_simple(depthImageByteSize, m_depthImageByteSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_depth_image_size(const Vector2i& depthImageSize)
{
  write_simple(depthImageSize, m_depthImageSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_rgb_image_byte_size(uint32_t rgbImageByteSize)
{
  write_simple(rgbImageByteSize, m_rgbImageByteSizeSegment);
}

void CompressedRGBDFrameHeaderMessage::set_rgb_image_size(const Vector2i& rgbImageSize)
{
  write_simple(rgbImageSize, m_rgbImageSizeSegment);
}

}
