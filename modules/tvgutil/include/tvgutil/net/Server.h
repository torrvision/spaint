/**
 * tvgutil: Server.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_TVGUTIL_SERVER
#define H_TVGUTIL_SERVER

#include <map>
#include <set>
#include <vector>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include "../boost/WrappedAsio.h"

//#define DEBUGGING 1

namespace tvgutil {

namespace Server_NS {

using boost::asio::ip::tcp;

/**
 * \brief An instance of a class deriving from this one represents a server that can be used to communicate with one or more clients.
 */
template <typename ClientHandlerType>
class Server
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<ClientHandlerType> ClientHandler_Ptr;

  //#################### ENUMERATIONS ####################
public:
  /** The values of this enumeration can be used to specify the mode in which the server should run. */
  enum Mode
  {
    /** The server will accept multiple clients. */
    SM_MULTI_CLIENT,

    /** The server will only accept a single client. */
    SM_SINGLE_CLIENT
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The server's TCP acceptor. */
  boost::shared_ptr<boost::asio::ip::tcp::acceptor> m_acceptor;

  /** A thread that keeps the map of clients clean by removing any clients that have terminated. */
  boost::shared_ptr<boost::thread> m_cleanerThread;

  /** The handlers for the currently active clients. */
  std::map<int, ClientHandler_Ptr> m_clientHandlers;

  /** A condition variable used to wait until a client thread is ready to start reading frame messages. */
  mutable boost::condition_variable m_clientReady;

  /** A condition variable used to wait for finished client threads that need to be cleaned up. */
  boost::condition_variable m_clientsHaveFinished;

  /** The set of clients that have finished. */
  std::set<int> m_finishedClients;

  /** The server's I/O service. */
  boost::asio::io_service m_ioService;

  /** The mode in which the server should run. */
  Mode m_mode;

  /** The mutex used to control client creation/deletion. */
  mutable boost::mutex m_mutex;

  /** The ID to give the next client to connect. */
  int m_nextClientID;

  /** The port on which the server should listen for connections. */
  int m_port;

  /** The server thread. */
  boost::shared_ptr<boost::thread> m_serverThread;

  /** Whether or not the server should terminate. */
  boost::shared_ptr<boost::atomic<bool> > m_shouldTerminate;

  /** The set of clients that have finished but have not yet been removed from the clients map. */
  std::set<int> m_uncleanClients;

  /** A worker variable used to keep the I/O service running until we want it to stop. */
  boost::shared_ptr<boost::asio::io_service::work> m_worker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a server.
   *
   * \param mode  The mode in which the server should run.
   * \param port  The port on which the server should listen for connections.
   */
  explicit Server(Mode mode = SM_MULTI_CLIENT, int port = 7851)
  : m_mode(mode),
    m_nextClientID(0),
    m_port(port),
    m_shouldTerminate(new boost::atomic<bool>(false)),
    m_worker(new boost::asio::io_service::work(m_ioService))
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the server.
   */
  virtual ~Server()
  {
    terminate();
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the IDs of the clients that are currently active.
   *
   * \return  The IDs of the clients that are currently active.
   */
  std::vector<int> get_active_clients() const
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);

    // Note: If necessary, we can optimise this by caching the list of active clients and updating it when new clients
    //       are accepted or existing clients terminate. However, since the number of clients is generally fairly low,
    //       the benefits of doing so are quite limited in practice.
    std::vector<int> activeClients;
    activeClients.reserve(m_clientHandlers.size());
    for(typename std::map<int, ClientHandler_Ptr>::const_iterator it = m_clientHandlers.begin(), iend = m_clientHandlers.end(); it != iend; ++it)
    {
      if(m_finishedClients.find(it->first) == m_finishedClients.end())
      {
        activeClients.push_back(it->first);
      }
    }

    return activeClients;
  }

  /**
   * \brief Gets whether or not the specified client is currently active.
   *
   * \param clientID  The ID of the client to check.
   * \return          true, if the client is currently active, or false otherwise.
   */
  bool is_active(int clientID) const
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);

    // Return whether or not the client is still active.
    return m_finishedClients.find(clientID) == m_finishedClients.end();
  }

  /**
   * \brief Starts the server.
   */
  void start()
  {
    m_serverThread.reset(new boost::thread(boost::bind(&Server::run_server, this)));
  }

  /**
   * \brief Gracefully terminates the server.
   */
  void terminate()
  {
    *m_shouldTerminate = true;

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

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Attempts to get the handler of the active client with the specified ID.
   *
   * If the client has not yet started, this will block.
   *
   * \param clientID  The ID of the client whose handler we want to get.
   * \return          The client handler, if the client exists and is active, or null if it has already finished.
   */
  ClientHandler_Ptr get_client_handler(int clientID) const
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);

    // Wait until the client is either active or has terminated.
    typename std::map<int, ClientHandler_Ptr>::const_iterator it;
    while((it = m_clientHandlers.find(clientID)) == m_clientHandlers.end() && m_finishedClients.find(clientID) == m_finishedClients.end())
    {
      m_clientReady.wait(lock);
    }

    return it != m_clientHandlers.end() ? it->second : ClientHandler_Ptr();
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Accepts a client.
   *
   * This will block until either a client connects or the server terminates.
   */
  void accept_client()
  {
    // FIXME: It would be better to have accept_client_handler call accept_client after accepting a connection.
    //        This would allow us to get rid of the sleep loop.
    boost::shared_ptr<tcp::socket> sock(new tcp::socket(m_ioService));
    m_acceptor->async_accept(*sock, boost::bind(&Server::accept_client_handler, this, sock, _1));
    while(!*m_shouldTerminate && m_ioService.poll() == 0)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
    }
  }

  /**
   * \brief The handler called when a client connects (technically, when an asynchronous accept finishes).
   *
   * \param sock  The socket via which to communicate with the client.
   * \param err   The error code associated with the accept.
   */
  void accept_client_handler(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, const boost::system::error_code& err)
  {
    // If an error occurred, early out.
    if(err) return;

    // If the server is running in single client mode and a second client tries to connect, early out.
    if(m_mode == SM_SINGLE_CLIENT && m_nextClientID != 0)
    {
      std::cout << "Warning: Rejecting client connection (server is in single client mode)" << std::endl;
      sock->close();
      return;
    }

    // If a client successfully connects, start a thread for it.
    std::cout << "Accepted client connection" << std::endl;
    boost::lock_guard<boost::mutex> lock(m_mutex);
    ClientHandler_Ptr clientHandler(new ClientHandlerType(m_nextClientID, sock, m_shouldTerminate));
    boost::shared_ptr<boost::thread> clientThread(new boost::thread(boost::bind(&Server::handle_client, this, clientHandler)));
    clientHandler->m_thread = clientThread;
    ++m_nextClientID;
  }

  /**
   * \brief Handles messages from a client.
   *
   * \param client  The client.
   */
  void handle_client(const ClientHandler_Ptr& clientHandler)
  {
    const int clientID = clientHandler->get_client_id();

    std::cout << "Starting client: " << clientID << '\n';

    // Run the pre-loop code for the client.
    clientHandler->run_pre();

    // Add the client handler to the map of handlers for active clients.
    {
      boost::lock_guard<boost::mutex> lock(m_mutex);
      m_clientHandlers.insert(std::make_pair(clientID, clientHandler));
    }

    // Signal to other threads that we're ready to start running the main loop for the client.
#if DEBUGGING
    std::cout << "Client ready: " << clientID << '\n';
#endif
    m_clientReady.notify_one();

    // Run the main loop for the client. Loop until either (a) the connection drops, or (b) the server itself is terminating.
    while(clientHandler->m_connectionOk && !*m_shouldTerminate)
    {
      clientHandler->run_iter();
    }

    // Run the post-loop code for the client.
    clientHandler->run_post();

    // Once the client's finished, add it to the finished clients set so that it can be cleaned up.
    {
      boost::lock_guard<boost::mutex> lock(m_mutex);
      std::cout << "Stopping client: " << clientID << '\n';
      m_finishedClients.insert(clientID);
      m_uncleanClients.insert(clientID);
      m_clientsHaveFinished.notify_one();
    }
  }

  /**
   * \brief Keeps the map of clients clean by removing any clients that have terminated.
   */
  void run_cleaner()
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);

    bool canTerminate = *m_shouldTerminate && m_clientHandlers.empty();
    while(!canTerminate)
    {
      // Wait for clients to finish.
      while(m_uncleanClients.empty()) m_clientsHaveFinished.wait(lock);

      // Clean up any clients that have finished.
      for(std::set<int>::const_iterator it = m_uncleanClients.begin(), iend = m_uncleanClients.end(); it != iend; ++it)
      {
        if(m_clientHandlers.find(*it) != m_clientHandlers.end())
        {
          std::cout << "Cleaning up client: " << *it << std::endl;
          m_clientHandlers.erase(*it);
        }
      }

      m_uncleanClients.clear();

      // Update the flag.
      canTerminate = *m_shouldTerminate && m_clientHandlers.empty();
    }

#if DEBUGGING
    std::cout << "Cleaner thread terminating" << std::endl;
#endif
  }

  /**
   * \brief Runs the server.
   */
  void run_server()
  {
    // Spawn a thread to keep the map of clients clean by removing any clients that have terminated.
    m_cleanerThread.reset(new boost::thread(&Server::run_cleaner, this));

    // Set up the TCP acceptor and listen for connections.
    tcp::endpoint endpoint(tcp::v4(), m_port);
    m_acceptor.reset(new tcp::acceptor(m_ioService, endpoint));

    std::cout << "Listening for connections...\n";

    while(!*m_shouldTerminate)
    {
      accept_client();
    }

#if DEBUGGING
    std::cout << "Server thread terminating" << std::endl;
#endif
  }
};

}

using Server_NS::Server;

}

#endif
