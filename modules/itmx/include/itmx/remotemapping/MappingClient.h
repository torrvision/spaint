/**
 * itmx: MappingClient.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGCLIENT
#define H_ITMX_MAPPINGCLIENT

#include <tvgutil/boost/WrappedAsio.h>
#include <tvgutil/containers/PooledQueue.h>

#include "RGBDCalibrationMessage.h"
#include "RGBDFrameCompressor.h"
#include "RGBDFrameMessage.h"

namespace itmx {

/**
 * \brief An instance of this class represents a client that can be used to communicate with a remote mapping server.
 */
class MappingClient
{
  //#################### TYPEDEFS ####################
public:
  typedef tvgutil::PooledQueue<RGBDFrameMessage_Ptr> RGBDFrameMessageQueue;

  //#################### PRIVATE VARIABLES ####################
private:
  /** A frame compressor, used to compress frame messages to reduce the network bandwidth they consume. */
  RGBDFrameCompressor_Ptr m_frameCompressor;

  /** A queue containing the RGB-D frame messages to be sent to the server. */
  RGBDFrameMessageQueue m_frameMessageQueue;

  /** The TCP stream used as a wrapper around the connection to the server. */
  boost::asio::ip::tcp::iostream m_stream;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mapping client.
   *
   * \param host              The mapping host to which to connect.
   * \param port              The port on the mapping host to which to connect.
   * \param poolEmptyStrategy A strategy specifying what should happen when a push is attempted while the frame message queue's pool is empty.
   */
  explicit MappingClient(const std::string& host = "localhost", const std::string& port = "7851", tvgutil::pooled_queue::PoolEmptyStrategy poolEmptyStrategy = tvgutil::pooled_queue::PES_DISCARD);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Starts the push of a frame message so that it can be sent to the server.
   */
  RGBDFrameMessageQueue::PushHandler_Ptr begin_push_frame_message();

  /**
   * \brief Sends a calibration message to the server.
   *
   * \param msg The message to send.
   */
  void send_calibration_message(const RGBDCalibrationMessage& msg);

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Sends frame messages from the message queue across to the server.
   */
  void run_message_sender();
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MappingClient> MappingClient_Ptr;
typedef boost::shared_ptr<const MappingClient> MappingClient_CPtr;

}

#endif
