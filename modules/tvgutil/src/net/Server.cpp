/**
 * tvgutil: Server.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "net/Server.h"
using boost::asio::ip::tcp;

#include <iostream>

#ifdef WITH_OPENCV
#include "ocv/OpenCVUtil.h"
#endif

#define DEBUGGING 0

namespace tvgutil {

//#################### CONSTRUCTORS ####################

Server::Server(Mode mode, int port)
: m_mode(mode), m_nextClientID(0), m_port(port), m_shouldTerminate(false), m_worker(new boost::asio::io_service::work(m_ioService))
{}

//#################### DESTRUCTOR ####################

Server::~Server()
{
  terminate();
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::vector<int> Server::get_active_clients() const
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  // Note: If necessary, we can optimise this by caching the list of active clients and updating it when new clients
  //       are accepted or existing clients terminate. However, since the number of clients is generally fairly low,
  //       the benefits of doing so are quite limited in practice.
  std::vector<int> activeClients;
  activeClients.reserve(m_clients.size());
  for (std::map<int, Client_Ptr>::const_iterator it = m_clients.begin(), iend = m_clients.end(); it != iend; ++it)
  {
    if (m_finishedClients.find(it->first) == m_finishedClients.end())
    {
      activeClients.push_back(it->first);
    }
  }

  return activeClients;
}

void Server::start()
{
  m_serverThread.reset(new boost::thread(boost::bind(&Server::run_server, this)));
}

void Server::terminate()
{
  m_shouldTerminate = true;

  if (m_serverThread) m_serverThread->join();

  if (m_cleanerThread)
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

void Server::accept_client()
{
  // FIXME: It would be better to have accept_client_handler call accept_client after accepting a connection.
  //        This would allow us to get rid of the sleep loop.
  boost::shared_ptr<tcp::socket> sock(new tcp::socket(m_ioService));
  m_acceptor->async_accept(*sock, boost::bind(&Server::accept_client_handler, this, sock, _1));
  while (!m_shouldTerminate && m_ioService.poll() == 0)
  {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
  }
}

void Server::accept_client_handler(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, const boost::system::error_code& err)
{
  // If an error occurred, early out.
  if (err) return;

  // If the server is running in single client mode and a second client tries to connect, early out.
  if (m_mode == SM_SINGLE_CLIENT && m_nextClientID != 0)
  {
    std::cout << "Warning: Rejecting client connection (server is in single client mode)" << std::endl;
    sock->close();
    return;
  }

  // If a client successfully connects, start a thread for it and add an entry to the clients map.
  std::cout << "Accepted client connection" << std::endl;
  boost::lock_guard<boost::mutex> lock(m_mutex);
  Client_Ptr client(new Client);
  boost::shared_ptr<boost::thread> clientThread(new boost::thread(boost::bind(&Server::handle_client, this, m_nextClientID, client, sock)));
  client->m_thread = clientThread;
  ++m_nextClientID;
}

Server::Client_Ptr Server::get_client(int clientID) const
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  // Wait until the client is either active or has terminated.
  std::map<int, Client_Ptr>::const_iterator it;
  while ((it = m_clients.find(clientID)) == m_clients.end() && m_finishedClients.find(clientID) == m_finishedClients.end())
  {
    m_clientReady.wait(lock);
  }

  return it != m_clients.end() ? it->second : Client_Ptr();
}

void Server::handle_client(int clientID, const Client_Ptr& client, const boost::shared_ptr<tcp::socket>& sock)
{
  std::cout << "Starting client: " << clientID << '\n';

#if 0
  // Read a calibration message from the client to get its camera's image sizes and calibration parameters.
  RGBDCalibrationMessage calibMsg;
  bool connectionOk = read_message(sock, calibMsg);
#if DEBUGGING
  std::cout << "Received calibration message from client: " << clientID << std::endl;
#endif

  // If the calibration message was successfully read:
  RGBDFrameCompressor_Ptr frameCompressor;
  RGBDFrameMessage_Ptr dummyFrameMsg;
  if (connectionOk)
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

  // Read and record frame messages from the client until either (a) the connection drops, or (b) the server itself is terminating.
  CompressedRGBDFrameHeaderMessage headerMsg;
  CompressedRGBDFrameMessage frameMsg(headerMsg);
  while (connectionOk && !m_shouldTerminate)
  {
#if DEBUGGING
    std::cout << "Message queue size (" << clientID << "): " << client->m_frameMessageQueue->size() << std::endl;
#endif

    RGBDFrameMessageQueue::PushHandler_Ptr pushHandler = client->m_frameMessageQueue->begin_push();
    boost::optional<RGBDFrameMessage_Ptr&> elt = pushHandler->get();
    RGBDFrameMessage& msg = elt ? **elt : *dummyFrameMsg;

    // First, try to read a frame header message.
    if ((connectionOk = read_message(sock, headerMsg)))
    {
      // If that succeeds, set up the frame message accordingly.
      frameMsg.set_compressed_image_sizes(headerMsg);

      // Now, read the frame message itself.
      if ((connectionOk = read_message(sock, frameMsg)))
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
#endif

  // Once we've finished reading messages, add the client to the finished clients set so that it can be cleaned up.
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    std::cout << "Stopping client: " << clientID << '\n';
    m_finishedClients.insert(clientID);
    m_uncleanClients.insert(clientID);
    m_clientsHaveFinished.notify_one();
  }
}

void Server::read_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
{
  // Store any error message so that it can be examined by read_message.
  ret = err;
}

void Server::run_cleaner()
{
  boost::unique_lock<boost::mutex> lock(m_mutex);

  bool canTerminate = m_shouldTerminate && m_clients.empty();
  while (!canTerminate)
  {
    // Wait for clients to finish.
    while (m_uncleanClients.empty()) m_clientsHaveFinished.wait(lock);

    // Clean up any clients that have finished.
    for (std::set<int>::const_iterator it = m_uncleanClients.begin(), iend = m_uncleanClients.end(); it != iend; ++it)
    {
      if (m_clients.find(*it) != m_clients.end())
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

void Server::run_server()
{
  // Spawn a thread to keep the map of clients clean by removing any clients that have terminated.
  m_cleanerThread.reset(new boost::thread(&Server::run_cleaner, this));

  // Set up the TCP acceptor and listen for connections.
  tcp::endpoint endpoint(tcp::v4(), m_port);
  m_acceptor.reset(new tcp::acceptor(m_ioService, endpoint));

  std::cout << "Listening for connections...\n";

  while (!m_shouldTerminate)
  {
    accept_client();
  }

#if DEBUGGING
  std::cout << "Server thread terminating" << std::endl;
#endif
}

void Server::write_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
{
  // Store any error message so that it can be examined by write_message.
  ret = err;
}

}
