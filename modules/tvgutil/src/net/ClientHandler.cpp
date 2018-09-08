/**
 * tvgutil: ClientHandler.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#include "net/ClientHandler.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

ClientHandler::ClientHandler()
: m_connectionOk(true)
{}

//#################### DESTRUCTOR ####################

ClientHandler::~ClientHandler() {}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ClientHandler::handle_main(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
{
  // No-op
}

void ClientHandler::handle_post(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
{
  // No-op
}

void ClientHandler::handle_pre(int clientID, const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock)
{
  // No-op
}

}
