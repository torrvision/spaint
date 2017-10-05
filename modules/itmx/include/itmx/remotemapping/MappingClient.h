/**
 * itmx: MappingClient.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_ITMX_MAPPINGCLIENT
#define H_ITMX_MAPPINGCLIENT

#include <boost/shared_ptr.hpp>

#include <tvgutil/boost/WrappedAsio.h>

#include "MappingMessage.h"

namespace itmx {

/**
 * \brief An instance of this class represents a client that can be used to communicate with a remote mapping server.
 */
class MappingClient
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The TCP stream used as a wrapper around the connection to the server. */
  boost::asio::ip::tcp::iostream m_stream;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a mapping client.
   *
   * \param host  The mapping host to which to connect.
   * \param port  The port on the mapping host to which to connect.
   */
  explicit MappingClient(const std::string& host = "localhost", const std::string& port = "7851");

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Sends a mapping message to the server.
   *
   * \param msg The message to send.
   */
  void send_message(const MappingMessage& msg);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<MappingClient> MappingClient_Ptr;
typedef boost::shared_ptr<const MappingClient> MappingClient_CPtr;

}

#endif
