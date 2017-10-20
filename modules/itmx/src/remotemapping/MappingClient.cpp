/**
 * itmx: MappingClient.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/MappingClient.h"
using namespace tvgutil;

#include <stdexcept>

#include <tvgutil/boost/WrappedAsio.h>
using boost::asio::ip::tcp;

#include "remotemapping/AckMessage.h"

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

  // Wait for an Ack (synchronously).
  AckMessage ackMsg;
  m_stream.read(ackMsg.get_data_ptr(), ackMsg.get_size());

  const int capacity = 1;
  m_frameMessageQueue.initialise(capacity, boost::bind(&RGBDFrameMessage::make, msg.extract_rgb_image_size(), msg.extract_depth_image_size()));

  // Setup RGB-D compression.
  m_frameCompressor.reset(new RGBDFrameCompressor(msg.extract_rgb_image_size(), msg.extract_depth_image_size(),
                                                  msg.extract_depth_compression_type(), msg.extract_rgb_compression_type()));

  boost::thread messageSender(&MappingClient::run_message_sender, this);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void MappingClient::run_message_sender()
{
  // Allocate compressed messages.
  CompressedRGBDFrameHeaderMessage_Ptr compressedRGBDMessageHeader(new CompressedRGBDFrameHeaderMessage);
  CompressedRGBDFrameMessage_Ptr compressedRGBDMessage(new CompressedRGBDFrameMessage(*compressedRGBDMessageHeader));

  // Allocate AckMessage.
  AckMessage_Ptr ackMessage(new AckMessage);

  bool connectionOk = true;
  while(connectionOk)
  {
    RGBDFrameMessage_Ptr msg = m_frameMessageQueue.peek();

    // Compress the frame.
    m_frameCompressor->compress_rgbd_frame(*msg, *compressedRGBDMessageHeader, *compressedRGBDMessage);

    // Send first the header, then the compressed frame. Chained with && to early out in case one of the send fails.
    // Finally, wait for an Ack.
    connectionOk = connectionOk
        && m_stream.write(compressedRGBDMessageHeader->get_data_ptr(), compressedRGBDMessageHeader->get_size())
        && m_stream.write(compressedRGBDMessage->get_data_ptr(), compressedRGBDMessage->get_size())
        && m_stream.read(ackMessage->get_data_ptr(), ackMessage->get_size());

    // TODO: check Ack status if necessary.

    m_frameMessageQueue.pop();
  }
}

}
