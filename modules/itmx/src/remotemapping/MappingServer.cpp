/**
 * itmx: MappingServer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/MappingServer.h"
using boost::asio::ip::tcp;
using namespace ITMLib;
using namespace tvgutil;

#include <iostream>

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#include "remotemapping/AckMessage.h"
#include "remotemapping/CompressedRGBDFrameHeaderMessage.h"
#include "remotemapping/CompressedRGBDFrameMessage.h"
#include "remotemapping/RGBDCalibrationMessage.h"
#include "remotemapping/RGBDFrameCompressor.h"

#define DEBUGGING 0

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingServer::MappingServer(Mode mode, int port)
: Server(mode, port)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ITMRGBDCalib MappingServer::get_calib(int clientID) const
{
  // FIXME: What to do when the client no longer exists needs more thought.
  Client_Ptr client = get_client(clientID);
  return client ? client->m_calib : ITMRGBDCalib();
}

Vector2i MappingServer::get_depth_image_size(int clientID) const
{
  // FIXME: What to do when the client no longer exists needs more thought.
  Client_Ptr client = get_client(clientID);
  return client ? client->get_depth_image_size() : Vector2i();
}

void MappingServer::get_images(int clientID, ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  // Look up the client whose images we want to get. If it is no longer active, early out.
  Client_Ptr client = get_client(clientID);
  if(!client) return;

  // If the images of the first message on the queue have already been read, it's time to
  // move on to the next frame, so pop the message from the queue and reset the flags.
  if(client->m_imagesDirty)
  {
    client->m_frameMessageQueue->pop();
    client->m_imagesDirty = client->m_poseDirty = false;
  }

  // Extract the images from the first message on the queue. This will block until the queue
  // has a message from which to extract images.
#if DEBUGGING
  std::cout << "Peeking for message" << std::endl;
#endif
  RGBDFrameMessage_Ptr msg = client->m_frameMessageQueue->peek();
#if DEBUGGING
  std::cout << "Extracting images for frame " << msg->extract_frame_index() << std::endl;
#endif
  msg->extract_rgb_image(rgb);
  msg->extract_depth_image(rawDepth);

  // Record the fact that we've now read the images from the first message on the queue.
  client->m_imagesDirty = true;
}

void MappingServer::get_pose(int clientID, ORUtils::SE3Pose& pose)
{
  // Look up the client whose pose we want to get. If it is no longer active, early out.
  Client_Ptr client = get_client(clientID);
  if(!client) return;

  // If the pose of the first message on the queue has already been read, it's time to
  // move on to the next frame, so pop the message from the queue and reset the flags.
  if(client->m_poseDirty)
  {
    client->m_frameMessageQueue->pop();
    client->m_imagesDirty = client->m_poseDirty = false;
  }

  // Extract the pose from the first message on the queue. This will block until the queue
  // has a message from which to extract the pose.
  RGBDFrameMessage_Ptr msg = client->m_frameMessageQueue->peek();
#if DEBUGGING
  std::cout << "Extracting pose for frame " << msg->extract_frame_index() << std::endl;
#endif
  pose = msg->extract_pose();

  // Record the fact that we've now read the pose from the first message on the queue.
  client->m_poseDirty = true;
}

Vector2i MappingServer::get_rgb_image_size(int clientID) const
{
  // FIXME: What to do when the client no longer exists needs more thought.
  Client_Ptr client = get_client(clientID);
  return client ? client->get_rgb_image_size() : Vector2i();
}

bool MappingServer::has_images_now(int clientID) const
{
  // Look up the client. If it is no longer active, early out.
  Client_Ptr client = get_client(clientID);
  if(!client) return false;

  // Calculate the effective queue size of the client (this excludes any message that we have already started reading).
  size_t effectiveQueueSize = client->m_frameMessageQueue->size();
  if(client->m_imagesDirty || client->m_poseDirty) --effectiveQueueSize;

  // Return whether or not the effective queue size is non-zero (i.e. there are new messages we haven't looked at).
  return effectiveQueueSize > 0;
}

bool MappingServer::has_more_images(int clientID) const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  // Return whether or not the client is still active.
  return m_finishedClients.find(clientID) == m_finishedClients.end();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void MappingServer::run_client_hook(int clientID, const Client_Ptr& client, const boost::shared_ptr<tcp::socket>& sock)
{
  // Read and record frame messages from the client until either (a) the connection drops, or (b) the server itself is terminating.
  CompressedRGBDFrameHeaderMessage headerMsg;
  CompressedRGBDFrameMessage frameMsg(headerMsg);
  while(connectionOk && !m_shouldTerminate)
  {
#if DEBUGGING
    std::cout << "Message queue size (" << clientID << "): " << client->m_frameMessageQueue->size() << std::endl;
#endif

    RGBDFrameMessageQueue::PushHandler_Ptr pushHandler = client->m_frameMessageQueue->begin_push();
    boost::optional<RGBDFrameMessage_Ptr&> elt = pushHandler->get();
    RGBDFrameMessage& msg = elt ? **elt : *dummyFrameMsg;

    // First, try to read a frame header message.
    if((connectionOk = read_message(sock, headerMsg)))
    {
      // If that succeeds, set up the frame message accordingly.
      frameMsg.set_compressed_image_sizes(headerMsg);

      // Now, read the frame message itself.
      if((connectionOk = read_message(sock, frameMsg)))
      {
        // If that succeeds, uncompress the images and send an acknowledgement to the client.
        frameCompressor->uncompress_rgbd_frame(frameMsg, msg);
        connectionOk = write_message(sock, AckMessage());

#if DEBUGGING
        std::cout << "Got message: " << msg.extract_frame_index() << std::endl;

      #ifdef WITH_OPENCV
        static ORUChar4Image_Ptr rgbImage(new ORUChar4Image(client->m_rgbImageSize, true, false));
        msg.extract_rgb_image(rgbImage.get());
        cv::Mat3b cvRGB = OpenCVUtil::make_rgb_image(rgbImage->GetData(MEMORYDEVICE_CPU), rgbImage->noDims.x, rgbImage->noDims.y);
        cv::imshow("RGB", cvRGB);
        cv::waitKey(1);
      #endif
#endif
      }
    }
  }

  // Destroy the frame compressor prior to stopping the client (this cleanly deallocates CUDA memory and avoids a crash on exit).
  frameCompressor.reset();
}

void MappingServer::setup_client_hook(int clientID, const Client_Ptr& client, const boost::shared_ptr<tcp::socket>& sock)
{
  // Read a calibration message from the client to get its camera's image sizes and calibration parameters.
  RGBDCalibrationMessage calibMsg;
  bool connectionOk = read_message(sock, calibMsg);
#if DEBUGGING
  std::cout << "Received calibration message from client: " << clientID << std::endl;
#endif

  // If the calibration message was successfully read:
  RGBDFrameCompressor_Ptr frameCompressor;
  RGBDFrameMessage_Ptr dummyFrameMsg;
  if(connectionOk)
  {
    // Save the calibration parameters.
    client->m_calib = calibMsg.extract_calib();

    // Initialise the frame message queue.
    const size_t capacity = 5;
    const Vector2i& rgbImageSize = client->get_rgb_image_size();
    const Vector2i& depthImageSize = client->get_depth_image_size();
    client->m_frameMessageQueue->initialise(capacity, boost::bind(&RGBDFrameMessage::make, rgbImageSize, depthImageSize));

    // Set up the frame compressor.
    frameCompressor.reset(new RGBDFrameCompressor(rgbImageSize, depthImageSize, calibMsg.extract_rgb_compression_type(), calibMsg.extract_depth_compression_type()));

    // Construct a dummy frame message to consume messages that cannot be pushed onto the queue.
    dummyFrameMsg.reset(new RGBDFrameMessage(rgbImageSize, depthImageSize));

    // Signal to the client that the server is ready.
    connectionOk = write_message(sock, AckMessage());
  }
}

}
