/**
 * itmx: MappingServer.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "remotemapping/MappingServer.h"
using boost::asio::ip::tcp;
using namespace ITMLib;
using namespace tvgutil;

#include <iostream>

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
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  return clientHandler ? clientHandler->get_calib() : ITMRGBDCalib();
}

Vector2i MappingServer::get_depth_image_size(int clientID) const
{
  // FIXME: What to do when the client no longer exists needs more thought.
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  return clientHandler ? clientHandler->get_depth_image_size() : Vector2i();
}

void MappingServer::get_images(int clientID, ORUChar4Image *rgb, ORShortImage *rawDepth)
{
  // Look up the handler for the client whose images we want to get. If the client is no longer active, early out.
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  if(!clientHandler) return;

  // If the images of the first message on the queue have already been read, it's time to
  // move on to the next frame, so pop the message from the queue and reset the flags.
  if(clientHandler->images_dirty())
  {
    clientHandler->get_frame_message_queue()->pop();
    clientHandler->set_images_dirty(false);
    clientHandler->set_pose_dirty(false);
  }

  // Extract the images from the first message on the queue. This will block until the queue
  // has a message from which to extract images.
#if DEBUGGING
  std::cout << "Peeking for message" << std::endl;
#endif
  RGBDFrameMessage_Ptr msg = clientHandler->get_frame_message_queue()->peek();
#if DEBUGGING
  std::cout << "Extracting images for frame " << msg->extract_frame_index() << std::endl;
#endif
  msg->extract_rgb_image(rgb);
  msg->extract_depth_image(rawDepth);

  // Record the fact that we've now read the images from the first message on the queue.
  clientHandler->set_images_dirty(true);
}

void MappingServer::get_pose(int clientID, ORUtils::SE3Pose& pose)
{
  // Look up the handler for the client whose pose we want to get. If the client is no longer active, early out.
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  if(!clientHandler) return;

  // If the pose of the first message on the queue has already been read, it's time to
  // move on to the next frame, so pop the message from the queue and reset the flags.
  if(clientHandler->pose_dirty())
  {
    clientHandler->get_frame_message_queue()->pop();
    clientHandler->set_images_dirty(false);
    clientHandler->set_pose_dirty(false);
  }

  // Extract the pose from the first message on the queue. This will block until the queue
  // has a message from which to extract the pose.
  RGBDFrameMessage_Ptr msg = clientHandler->get_frame_message_queue()->peek();
#if DEBUGGING
  std::cout << "Extracting pose for frame " << msg->extract_frame_index() << std::endl;
#endif
  pose = msg->extract_pose();

  // Record the fact that we've now read the pose from the first message on the queue.
  clientHandler->set_pose_dirty(true);
}

ExclusiveHandle_Ptr<ORUChar4Image_Ptr>::Type MappingServer::get_rendered_image(int clientID) const
{
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  return clientHandler ? clientHandler->get_rendered_image() : ExclusiveHandle_Ptr<ORUChar4Image_Ptr>::Type();
}

boost::optional<RenderingRequestMessage> MappingServer::get_rendering_request(int clientID) const
{
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  if(clientHandler)
  {
    ExclusiveHandle_Ptr<boost::optional<RenderingRequestMessage> >::Type requestHandle = clientHandler->get_rendering_request();
    return requestHandle->get();
  }
  else return boost::none;
}

Vector2i MappingServer::get_rgb_image_size(int clientID) const
{
  // FIXME: What to do when the client no longer exists needs more thought.
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  return clientHandler ? clientHandler->get_rgb_image_size() : Vector2i();
}

bool MappingServer::has_images_now(int clientID) const
{
  // Look up the handler for the client. If the client is no longer active, early out.
  ClientHandler_Ptr clientHandler = get_client_handler(clientID);
  if(!clientHandler) return false;

  // Calculate the effective queue size of the client (this excludes any message that we have already started reading).
  size_t effectiveQueueSize = clientHandler->get_frame_message_queue()->size();
  if(clientHandler->images_dirty() || clientHandler->pose_dirty()) --effectiveQueueSize;

  // Return whether or not the effective queue size is non-zero (i.e. there are new messages we haven't looked at).
  return effectiveQueueSize > 0;
}

bool MappingServer::has_more_images(int clientID) const
{
  return !has_finished(clientID);
}

}
