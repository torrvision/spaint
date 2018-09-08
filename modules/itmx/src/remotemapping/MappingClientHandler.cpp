/**
 * itmx: MappingClientHandler.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "remotemapping/MappingClientHandler.h"

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingClientHandler::MappingClientHandler()
: m_frameMessageQueue(new RGBDFrameMessageQueue(tvgutil::pooled_queue::PES_DISCARD)),
  m_imagesDirty(false),
  m_poseDirty(false)
{
  m_frameMessage.reset(new CompressedRGBDFrameMessage(m_headerMessage));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const Vector2i& MappingClientHandler::get_depth_image_size() const
{
  return m_calib.intrinsics_d.imgSize;
}

const Vector2i& MappingClientHandler::get_rgb_image_size() const
{
  return m_calib.intrinsics_rgb.imgSize;
}

void MappingClientHandler::handle_main(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
{
  // TODO
}

void MappingClientHandler::handle_post(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
{
  // TODO
}

void MappingClientHandler::handle_pre(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
{
  // TODO
}

}
