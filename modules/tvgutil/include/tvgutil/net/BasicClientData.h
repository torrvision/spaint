/**
 * tvgutil: BasicClientData.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_TVGUTIL_BASICCLIENTDATA
#define H_TVGUTIL_BASICCLIENTDATA

#include <boost/thread.hpp>

namespace tvgutil {

/**
 * \brief An instance of this struct can be used to hold the data for a basic client.
 */
struct BasicClientData
{
  //#################### PUBLIC VARIABLES ####################

  /** Whether or not the connection is still ok (effectively tracks whether or not the most recent read/write succeeded). */
  bool m_connectionOk;

  /** The thread that manages communication with the client. */
  boost::shared_ptr<boost::thread> m_thread;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs a basic client data instance.
   */
  BasicClientData();
};

}

#endif
