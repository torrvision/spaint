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
#include "remotemapping/InteractionTypeMessage.h"

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
  m_frameMessageQueue.initialise(capacity, boost::bind(&RGBDFrameMessage::make, msg.extract_rgb_image_size(), msg.extract_depth_image_size()));

  // Set up the RGB-D frame compressor.
  m_frameCompressor.reset(new RGBDFrameCompressor(
    msg.extract_rgb_image_size(), msg.extract_depth_image_size(),
    msg.extract_rgb_compression_type(), msg.extract_depth_compression_type()
  ));

  // Start the message sender thread.
  boost::thread messageSender(&MappingClient::run_message_sender, this);
}

void MappingClient::update_rendering(const ORUtils::SE3Pose& clientPose)
{
  AckMessage ackMsg;
  InteractionTypeMessage interactionTypeMsg(IT_UPDATERENDERING);
  SimpleMessage<ORUtils::SE3Pose> clientPoseMsg(clientPose);

  boost::lock_guard<boost::mutex> lock(m_interactionMutex);

  // First send the interaction type message, then send the client pose message,
  // then wait for an acknowledgement from the server. We chain all of these
  // with && so as to early out in case of failure.
  m_stream.write(interactionTypeMsg.get_data_ptr(), interactionTypeMsg.get_size()) &&
  m_stream.write(clientPoseMsg.get_data_ptr(), clientPoseMsg.get_size()) &&
  m_stream.read(ackMsg.get_data_ptr(), ackMsg.get_size());
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void MappingClient::run_message_sender()
{
  AckMessage ackMsg;
  CompressedRGBDFrameHeaderMessage headerMsg;
  CompressedRGBDFrameMessage frameMsg(headerMsg);
  InteractionTypeMessage interactionTypeMsg(IT_SENDFRAME);

  bool connectionOk = true;

  while(connectionOk)
  {
    // Read the first frame message from the queue (this will block until a message is available).
    RGBDFrameMessage_Ptr msg = m_frameMessageQueue.peek();

    // Compress the frame. The compressed frame is split into two messages - a header message,
    // which tells the server how large a frame to expect, and a separate message containing
    // the actual frame data.
    m_frameCompressor->compress_rgbd_frame(*msg, headerMsg, frameMsg);

    {
      boost::lock_guard<boost::mutex> lock(m_interactionMutex);

      // First send the interaction type message, then send the frame header message, then send
      // the frame message itself, then wait for an acknowledgement from the server. We chain
      // all of these with && so as to early out in case of failure.
      connectionOk = connectionOk
        && m_stream.write(interactionTypeMsg.get_data_ptr(), interactionTypeMsg.get_size())
        && m_stream.write(headerMsg.get_data_ptr(), headerMsg.get_size())
        && m_stream.write(frameMsg.get_data_ptr(), frameMsg.get_size())
        && m_stream.read(ackMsg.get_data_ptr(), ackMsg.get_size());
    }

    // Remove the frame message that we have just sent from the queue.
    m_frameMessageQueue.pop();
  }
}

}
