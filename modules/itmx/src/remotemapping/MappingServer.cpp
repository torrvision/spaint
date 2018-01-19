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
#include "remotemapping/InteractionTypeMessage.h"
#include "remotemapping/RGBDCalibrationMessage.h"
#include "remotemapping/RGBDFrameCompressor.h"

#define DEBUGGING 0

namespace itmx {

//#################### CONSTRUCTORS ####################

MappingServer::MappingServer(Mode mode, int port)
: m_mode(mode), m_nextClientID(0), m_port(port), m_shouldTerminate(false), m_worker(new boost::asio::io_service::work(m_ioService))
{}

//#################### DESTRUCTOR ####################

MappingServer::~MappingServer()
{
  terminate();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<int> MappingServer::get_active_clients() const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  // Note: If necessary, we can optimise this by caching the list of active clients and updating it when new clients
  //       are accepted or existing clients terminate. However, since the number of clients is generally fairly low,
  //       the benefits of doing so are quite limited in practice.
  std::vector<int> activeClients;
  activeClients.reserve(m_clients.size());
  for(std::map<int,Client_Ptr>::const_iterator it = m_clients.begin(), iend = m_clients.end(); it != iend; ++it)
  {
    if(m_finishedClients.find(it->first) == m_finishedClients.end())
    {
      activeClients.push_back(it->first);
    }
  }

  return activeClients;
}

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
  return client ? client->m_depthImageSize : Vector2i();
}

void MappingServer::get_images(int clientID, ITMUChar4Image *rgb, ITMShortImage *rawDepth)
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

const boost::optional<ORUtils::SE3Pose>& MappingServer::get_rendering_pose(int clientID) const
{
  Client_Ptr client = get_client(clientID);
  if(client)
  {
    boost::lock_guard<boost::mutex> lock(client->m_renderingPoseMutex);
    return client->m_renderingPose;
  }
  else return boost::none;
}

Vector2i MappingServer::get_rgb_image_size(int clientID) const
{
  // FIXME: What to do when the client no longer exists needs more thought.
  Client_Ptr client = get_client(clientID);
  return client ? client->m_rgbImageSize : Vector2i();
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

void MappingServer::start()
{
  m_serverThread.reset(new boost::thread(boost::bind(&MappingServer::run_server, this)));
}

void MappingServer::terminate()
{
  m_shouldTerminate = true;

  if(m_serverThread) m_serverThread->join();

  if(m_cleanerThread)
  {
    // Make sure that the cleaner thread can terminate when there are no clients remaining to wake it up.
    m_uncleanClients.insert(-1);
    m_clientsHaveFinished.notify_one();

    m_cleanerThread->join();
  }

  // Note: It's essential that we destroy the acceptor before the I/O service, or there will be a crash.
  m_acceptor.reset();
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void MappingServer::accept_client()
{
  // FIXME: It would be better to have accept_client_handler call accept_client after accepted a connection.
  //        This would allow us to get rid of the sleep loop.
  boost::shared_ptr<tcp::socket> sock(new tcp::socket(m_ioService));
  m_acceptor->async_accept(*sock, boost::bind(&MappingServer::accept_client_handler, this, sock, _1));
  while(!m_shouldTerminate && m_ioService.poll() == 0)
  {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
  }
}

void MappingServer::accept_client_handler(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, const boost::system::error_code& err)
{
  // If an error occurred, early out.
  if(err) return;

  // If the server is running in single client mode and a second client tries to connect, early out.
  if(m_mode == MSM_SINGLE_CLIENT && m_nextClientID != 0)
  {
    std::cout << "Warning: Rejecting client connection (server is in single client mode)" << std::endl;
    sock->close();
    return;
  }

  // If a client successfully connects, start a thread for it and add an entry to the clients map.
  std::cout << "Accepted client connection" << std::endl;
  boost::lock_guard<boost::mutex> lock(m_mutex);
  Client_Ptr client(new Client);
  boost::shared_ptr<boost::thread> clientThread(new boost::thread(boost::bind(&MappingServer::handle_client, this, m_nextClientID, client, sock)));
  client->m_thread = clientThread;
  ++m_nextClientID;
}

MappingServer::Client_Ptr MappingServer::get_client(int clientID) const
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // Wait until the client is either active or has terminated.
  std::map<int,Client_Ptr>::const_iterator it;
  while((it = m_clients.find(clientID)) == m_clients.end() && m_finishedClients.find(clientID) == m_finishedClients.end())
  {
    m_clientReady.wait(lock);
  }

  return it != m_clients.end() ? it->second : Client_Ptr();
}

void MappingServer::handle_client(int clientID, const Client_Ptr& client, const boost::shared_ptr<tcp::socket>& sock)
{
  std::cout << "Starting client: " << clientID << '\n';

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
    // Save the image sizes and calibration parameters.
    client->m_rgbImageSize = calibMsg.extract_rgb_image_size();
    client->m_depthImageSize = calibMsg.extract_depth_image_size();
    client->m_calib = calibMsg.extract_calib();

    // Initialise the frame message queue.
    const size_t capacity = 5;
    client->m_frameMessageQueue->initialise(capacity, boost::bind(&RGBDFrameMessage::make, client->m_rgbImageSize, client->m_depthImageSize));

    // Set up the frame compressor.
    frameCompressor.reset(new RGBDFrameCompressor(
      client->m_rgbImageSize, client->m_depthImageSize,
      calibMsg.extract_rgb_compression_type(),
      calibMsg.extract_depth_compression_type()
    ));

    // Construct a dummy frame message to consume messages that cannot be pushed onto the queue.
    dummyFrameMsg.reset(new RGBDFrameMessage(client->m_rgbImageSize, client->m_depthImageSize));

    // Signal to the client that the server is ready.
    connectionOk = write_message(sock, AckMessage());
  }

  // Add the client to the map of active clients.
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_clients.insert(std::make_pair(clientID, client));
  }

  // Signal to other threads that we're ready to start reading frame messages from the client.
#if DEBUGGING
  std::cout << "Client ready: " << clientID << '\n';
#endif
  m_clientReady.notify_one();

  // Read and process messages from the client until either (a) the connection drops, or (b) the server itself is terminating.
  CompressedRGBDFrameHeaderMessage headerMsg;
  CompressedRGBDFrameMessage frameMsg(headerMsg);
  InteractionTypeMessage interactionTypeMsg;
  SimpleMessage<ORUtils::SE3Pose> renderingPoseMsg;

  while(connectionOk && !m_shouldTerminate)
  {
    // First, try to read an interaction type message.
    if((connectionOk = read_message(sock, interactionTypeMsg)))
    {
      // If that succeeds, determine the type of interaction the client wants to have with the server and proceed accordingly.
      switch(interactionTypeMsg.extract_value())
      {
        case IT_GETRENDERING:
        {
#if DEBUGGING
          std::cout << "Receiving get rendering request from client" << std::endl;
#endif

          // TODO
          break;
        }
        case IT_SENDFRAME:
        {
#if DEBUGGING
          std::cout << "Receiving frame from client" << std::endl;
#endif

          // Try to read a frame header message.
          if((connectionOk = read_message(sock, headerMsg)))
          {
            // If that succeeds, set up the frame message accordingly.
            frameMsg.set_compressed_image_sizes(headerMsg);

            // Now, read the frame message itself.
            if((connectionOk = read_message(sock, frameMsg)))
            {
              // If that succeeds, uncompress the images, store them on the frame message queue and send an acknowledgement to the client.
#if DEBUGGING
              std::cout << "Message queue size (" << clientID << "): " << client->m_frameMessageQueue->size() << std::endl;
#endif

              RGBDFrameMessageQueue::PushHandler_Ptr pushHandler = client->m_frameMessageQueue->begin_push();
              boost::optional<RGBDFrameMessage_Ptr&> elt = pushHandler->get();
              RGBDFrameMessage& msg = elt ? **elt : *dummyFrameMsg;
              frameCompressor->uncompress_rgbd_frame(frameMsg, msg);

              connectionOk = write_message(sock, AckMessage());

#if DEBUGGING
              std::cout << "Got message: " << msg.extract_frame_index() << std::endl;

            #ifdef WITH_OPENCV
              static ITMUChar4Image_Ptr rgbImage(new ITMUChar4Image(client->m_rgbImageSize, true, false));
              msg.extract_rgb_image(rgbImage.get());
              cv::Mat3b cvRGB = OpenCVUtil::make_rgb_image(rgbImage->GetData(MEMORYDEVICE_CPU), rgbImage->noDims.x, rgbImage->noDims.y);
              cv::imshow("RGB", cvRGB);
              cv::waitKey(1);
            #endif
#endif
            }
          }

          break;
        }
        case IT_UPDATERENDERING:
        {
#if DEBUGGING
          std::cout << "Receiving update rendering request from client" << std::endl;
#endif

          // Try to read a rendering pose message.
          if((connectionOk = read_message(sock, renderingPoseMsg)))
          {
            // If that succeeds, store the pose so that it can be picked up by the renderer and send an acknowledgement to the client.
            boost::lock_guard<boost::mutex> lock(client->m_renderingPoseMutex);
            client->m_renderingPose = renderingPoseMsg.extract_value();
            connectionOk = write_message(sock, AckMessage());
          }

          break;
        }
        default:
        {
          break;
        }
      }
    }
  }

  // Destroy the frame compressor prior to stopping the client (this cleanly deallocates CUDA memory and avoids a crash on exit).
  frameCompressor.reset();

  // Once we've finished reading messages, add the client to the finished clients set so that it can be cleaned up.
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    std::cout << "Stopping client: " << clientID << '\n';
    m_finishedClients.insert(clientID);
    m_uncleanClients.insert(clientID);
    m_clientsHaveFinished.notify_one();
  }
}

void MappingServer::read_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
{
  // Store any error message so that it can be examined by read_message.
  ret = err;
}

void MappingServer::run_cleaner()
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  bool canTerminate = m_shouldTerminate && m_clients.empty();
  while(!canTerminate)
  {
    // Wait for clients to finish.
    while(m_uncleanClients.empty()) m_clientsHaveFinished.wait(lock);

    // Clean up any clients that have finished.
    for(std::set<int>::const_iterator it = m_uncleanClients.begin(), iend = m_uncleanClients.end(); it != iend; ++it)
    {
      if(m_clients.find(*it) != m_clients.end())
      {
        std::cout << "Cleaning up client: " << *it << std::endl;
        m_clients.erase(*it);
      }
    }

    m_uncleanClients.clear();

    // Update the flag.
    canTerminate = m_shouldTerminate && m_clients.empty();
  }

#if DEBUGGING
  std::cout << "Cleaner thread terminating" << std::endl;
#endif
}

void MappingServer::run_server()
{
  // Spawn a thread to keep the map of clients clean by removing any clients that have terminated.
  m_cleanerThread.reset(new boost::thread(&MappingServer::run_cleaner, this));

  // Set up the TCP acceptor and listen for connections.
  tcp::endpoint endpoint(tcp::v4(), m_port);
  m_acceptor.reset(new tcp::acceptor(m_ioService, endpoint));

  std::cout << "Listening for connections...\n";

  while(!m_shouldTerminate)
  {
    accept_client();
  }

#if DEBUGGING
  std::cout << "Server thread terminating" << std::endl;
#endif
}

void MappingServer::write_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
{
  // Store any error message so that it can be examined by write_message.
  ret = err;
}

}
