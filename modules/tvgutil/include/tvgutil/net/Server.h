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
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include "../boost/WrappedAsio.h"

namespace tvgutil {

/**
 * \brief An instance of a class deriving from this one represents a server that can be used to communicate with one or more clients.
 */
class Server
{
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

  //#################### NESTED TYPES ####################
private:
  /**
   * \brief An instance of this struct contains all of the information associated with an individual client.
   */
  struct Client
  {
    //~~~~~~~~~~~~~~~~~~~~ PUBLIC VARIABLES ~~~~~~~~~~~~~~~~~~~~

    /** The thread that manages communication with the client. */
    boost::shared_ptr<boost::thread> m_thread;
  };

  typedef boost::shared_ptr<Client> Client_Ptr;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The server's TCP acceptor. */
  boost::shared_ptr<boost::asio::ip::tcp::acceptor> m_acceptor;

  /** A thread that keeps the map of clients clean by removing any clients that have terminated. */
  boost::shared_ptr<boost::thread> m_cleanerThread;

  /** A condition variable used to wait until a client thread is ready to start reading frame messages. */
  // STILL NEEDED?
  mutable boost::condition_variable m_clientReady;

  /** A condition variable used to wait for finished client threads that need to be cleaned up. */
  boost::condition_variable m_clientsHaveFinished;

  /** The currently active clients. */
  std::map<int, Client_Ptr> m_clients;

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
  boost::atomic<bool> m_shouldTerminate;

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
  explicit Server(Mode mode = SM_MULTI_CLIENT, int port = 7851);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the server.
   */
  virtual ~Server();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the IDs of the clients that are currently active.
   *
   * \return  The IDs of the clients that are currently active.
   */
  std::vector<int> get_active_clients() const;

  /**
   * \brief Starts the server.
   */
  void start();

  /**
   * \brief Gracefully terminates the server.
   */
  void terminate();

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Accepts a client.
   *
   * This will block until either a client connects or the server terminates.
   */
  void accept_client();

  /**
   * \brief The handler called when a client connects (technically, when an asynchronous accept finishes).
   *
   * \param sock  The socket via which to communicate with the client.
   * \param err   The error code associated with the accept.
   */
  void accept_client_handler(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, const boost::system::error_code& err);

  /**
   * \brief Attempts to get the active client with the specified ID.
   *
   * If the client has not yet started, this will block.
   *
   * \param clientID  The ID of the client to get.
   * \return          The client, if it exists and is active, or null if it has already finished.
   */
  Client_Ptr get_client(int clientID) const;

  /**
   * \brief Handles messages from a client.
   *
   * \param clientID  The ID of the client.
   * \param client    The client itself.
   * \param socket    The TCP socket associated with the client.
   */
  void handle_client(int clientID, const Client_Ptr& client, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock);

  /**
   * \brief Attempts to read a message of type T from the specified socket.
   *
   * This will block until either the read succeeds, an error occurs or the server terminates.
   *
   * \param sock  The socket from which to attempt to read the message.
   * \param msg   The T into which to write the message, if reading succeeded.
   * \return      true, if reading succeeded, or false otherwise.
   */
  template <typename T>
  bool read_message(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, T& msg)
  {
    boost::optional<boost::system::error_code> err;
    boost::asio::async_read(*sock, boost::asio::buffer(msg.get_data_ptr(), msg.get_size()), boost::bind(&MappingServer::read_message_handler, this, _1, boost::ref(err)));
    while (!err && !m_shouldTerminate) boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    return m_shouldTerminate ? false : !*err;
  }

  /**
   * \brief The handler called when an asynchronous read of a message finishes.
   *
   * \param err The error code associated with the read.
   * \param ret A location into which to write the error code so that read_message can access it.
   */
  void read_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret);

  /**
   * \brief Keeps the map of clients clean by removing any clients that have terminated.
   */
  void run_cleaner();

  /**
   * \brief Runs the server.
   */
  void run_server();

  /**
   * \brief Attempts to write a message of type T on the specified socket.
   *
   * This will block until either the write succeeds, an error occurs or the server terminates.
   *
   * \param sock  The socket to which to attempt to write the message.
   * \param msg   The T to write.
   * \return      true, if writing succeeded, or false otherwise.
   */
  template <typename T>
  bool write_message(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock, const T& msg)
  {
    boost::optional<boost::system::error_code> err;
    boost::asio::async_write(*sock, boost::asio::buffer(msg.get_data_ptr(), msg.get_size()), boost::bind(&MappingServer::write_message_handler, this, _1, boost::ref(err)));
    while (!err && !m_shouldTerminate) boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    return m_shouldTerminate ? false : !*err;
  }

  /**
   * \brief The handler called when an asynchronous write of a message finishes.
   *
   * \param err The error code associated with the write.
   * \param ret A location into which to write the error code so that write_message can access it.
   */
  void write_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<Server> Server_Ptr;
typedef boost::shared_ptr<const Server> Server_CPtr;

}

#endif
