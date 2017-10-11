/**
 * itmx: MappingClient.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/MappingClient.h"
using namespace tvgutil;

#include <stdexcept>

#include <tvgutil/boost/WrappedAsio.h>
using boost::asio::ip::tcp;

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingClient::MappingClient(const std::string& host, const std::string& port)
: m_frameMessageQueue(pooled_queue::PES_DISCARD), m_stream(host, port)
{
  if(!m_stream) throw std::runtime_error("Error: Could not connect to server");
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

MappingClient::RGBDFrameMessageQueue::PushHandler_Ptr MappingClient::begin_push_frame_message()
{
  return m_frameMessageQueue.begin_push();
}

void MappingClient::send_calibration_message(const RGBDCalibrationMessage& msg)
{
  m_stream.write(msg.get_data_ptr(), msg.get_size());

  const int capacity = 1;
  m_frameMessageQueue.initialise(capacity, boost::bind(&RGBDFrameMessage::make, msg.extract_rgb_image_size(), msg.extract_depth_image_size()));

  boost::thread messageSender(&MappingClient::run_message_sender, this);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void MappingClient::run_message_sender()
{
  bool connectionOk = true;
  while(connectionOk)
  {
    RGBDFrameMessage_Ptr msg = m_frameMessageQueue.peek();
    connectionOk = connectionOk && m_stream.write(msg->get_data_ptr(), msg->get_size());
    m_frameMessageQueue.pop();
  }
}

}
