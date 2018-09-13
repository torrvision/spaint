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

void ClientHandler::handle_main()
{
  // No-op
}

void ClientHandler::handle_post()
{
  // No-op
}

void ClientHandler::handle_pre()
{
  // No-op
}

}
