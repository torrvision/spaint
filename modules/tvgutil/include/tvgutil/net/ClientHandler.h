/**
 * tvgutil: ClientHandler.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_TVGUTIL_CLIENTHANDLER
#define H_TVGUTIL_CLIENTHANDLER

#include <boost/thread.hpp>

#include "../boost/WrappedAsio.h"

namespace tvgutil {

/**
 * \brief An instance of this class can be used to manage the connection to a client.
 */
class ClientHandler
{
  //#################### PUBLIC VARIABLES ####################
public:
  /** Whether or not the connection is still ok (effectively tracks whether or not the most recent read/write succeeded). */
  bool m_connectionOk;

  /** The thread that manages communication with the client. */
  boost::shared_ptr<boost::thread> m_thread;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a client handler.
   */
  ClientHandler();

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the client handler.
   */
  virtual ~ClientHandler();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  virtual void handle_main(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock);

  /**
   * \brief TODO
   */
  virtual void handle_post(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock);

  /**
   * \brief TODO
   */
  virtual void handle_pre(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock);
};

}

#endif
