/**
 * itmx: MappingClientData.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "remotemapping/MappingClientData.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingClientData::MappingClientData()
: m_frameMessageQueue(new RGBDFrameMessageQueue(tvgutil::pooled_queue::PES_DISCARD)),
  m_imagesDirty(false),
  m_poseDirty(false)
{
  m_frameMessage.reset(new CompressedRGBDFrameMessage(m_headerMessage));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& MappingClientData::get_depth_image_size() const
{
  return m_calib.intrinsics_d.imgSize;
}

const Vector2i& MappingClientData::get_rgb_image_size() const
{
  return m_calib.intrinsics_rgb.imgSize;
}

}
