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

MappingClient::MappingClient(const std::string& host, const std::string& port, pooled_queue::PoolEmptyStrategy poolEmptyStrategy)
: m_frameMessageQueue(poolEmptyStrategy), m_stream(host, port)
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
  bool connectionOk = true;

  // Send the message to the server.
  connectionOk = connectionOk && m_stream.write(msg.get_data_ptr(), msg.get_size());

  // Wait for an acknowledgement (note that this is blocking, unless the connection fails).
  AckMessage ackMsg;
  connectionOk = connectionOk && m_stream.read(ackMsg.get_data_ptr(), ackMsg.get_size());

  // Throw if the message was not successfully sent and acknowledged.
  if(!connectionOk) throw std::runtime_error("Error: Failed to send calibration message");

  // Initialise the frame message queue.
  const int capacity = 1;
  const ITMLib::ITMRGBDCalib calib = msg.extract_calib();
  const Vector2i rgbImageSize = calib.intrinsics_rgb.imgSize;
  const Vector2i depthImageSize = calib.intrinsics_d.imgSize;
  m_frameMessageQueue.initialise(capacity, boost::bind(&RGBDFrameMessage::make, rgbImageSize, depthImageSize));

  // Set up the RGB-D frame compressor.
  m_frameCompressor.reset(new RGBDFrameCompressor(rgbImageSize, depthImageSize, msg.extract_rgb_compression_type(), msg.extract_depth_compression_type()));

  // Start the message sender thread.
  boost::thread messageSender(&MappingClient::run_message_sender, this);
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void MappingClient::run_message_sender()
{
  AckMessage ackMsg;
  CompressedRGBDFrameHeaderMessage headerMsg;
  CompressedRGBDFrameMessage frameMsg(headerMsg);

  bool connectionOk = true;

  while(connectionOk)
  {
    // Read the first frame message from the queue (this will block until a message is available).
    RGBDFrameMessage_Ptr msg = m_frameMessageQueue.peek();

    // Compress the frame. The compressed frame is split into two messages - a header message,
    // which tells the server how large a frame to expect, and a separate message containing
    // the actual frame data.
    m_frameCompressor->compress_rgbd_frame(*msg, headerMsg, frameMsg);

    // First send the header message, then send the frame message, then wait for an acknowledgement
    // from the server. We chain all of these with && so as to early out in case of failure.
    connectionOk = connectionOk
      && m_stream.write(headerMsg.get_data_ptr(), headerMsg.get_size())
      && m_stream.write(frameMsg.get_data_ptr(), frameMsg.get_size())
      && m_stream.read(ackMsg.get_data_ptr(), ackMsg.get_size());

    // Remove the frame message that we have just sent from the queue.
    m_frameMessageQueue.pop();
  }
}

}
