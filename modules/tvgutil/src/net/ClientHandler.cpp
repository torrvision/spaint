/**
 * tvgutil: ClientHandler.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "net/ClientHandler.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

ClientHandler::ClientHandler(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock,
                             const boost::shared_ptr<const boost::atomic<bool> >& shouldTerminate)
: m_clientID(clientID),
  m_connectionOk(true),
  m_shouldTerminate(shouldTerminate),
  m_sock(sock)
{}

//#################### DESTRUCTOR ####################

ClientHandler::~ClientHandler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int ClientHandler::get_client_id() const
{
  return m_clientID;
}

void ClientHandler::run_iter()
{
  // No-op by default
}

void ClientHandler::run_post()
{
  // No-op by default
}

void ClientHandler::run_pre()
{
  // No-op by default
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void ClientHandler::read_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
{
  // Store any error message so that it can be examined by read_message.
  ret = err;
}

void ClientHandler::write_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
{
  // Store any error message so that it can be examined by write_message.
  ret = err;
}

}
