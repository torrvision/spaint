/**
 * tvgutil: ClientHandler.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "net/ClientHandler.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

ClientHandler::ClientHandler(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
: m_connectionOk(true), m_sock(sock)
{}

//#################### DESTRUCTOR ####################

ClientHandler::~ClientHandler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ClientHandler::handle_main(int clientID)
{
  // No-op
}

void ClientHandler::handle_post(int clientID)
{
  // No-op
}

void ClientHandler::handle_pre(int clientID)
{
  // No-op
}

}
