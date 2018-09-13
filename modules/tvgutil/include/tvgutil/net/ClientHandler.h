/**
 * tvgutil: ClientHandler.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2018. All rights reserved.
 */

#ifndef H_TVGUTIL_CLIENTHANDLER
#define H_TVGUTIL_CLIENTHANDLER

#include <boost/optional.hpp>
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

  /** Whether or not the server should terminate. */
  boost::shared_ptr<const boost::atomic<bool> > m_shouldTerminate;

  /** The socket used to communicate with the client. */
  const boost::shared_ptr<boost::asio::ip::tcp::socket> m_sock;

  /** The thread that manages communication with the client. */
  boost::shared_ptr<boost::thread> m_thread;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a client handler.
   *
   * \param sock              The socket used to communicate with the client.
   * \param shouldTerminate   Whether or not the server should terminate.
   */
  explicit ClientHandler(const boost::shared_ptr<boost::asio::ip::tcp::socket>& sock,
                         const boost::shared_ptr<const boost::atomic<bool> >& shouldTerminate);

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
  virtual void handle_main(int clientID);

  /**
   * \brief TODO
   */
  virtual void handle_post(int clientID);

  /**
   * \brief TODO
   */
  virtual void handle_pre(int clientID);

  //#################### PROTECTED MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Attempts to read a message of type T from the socket used to communicate with the client.
   *
   * This will block until either the read succeeds, an error occurs or the server terminates.
   *
   * \param msg   The T into which to write the message, if reading succeeded.
   * \return      true, if reading succeeded, or false otherwise.
   */
  template <typename T>
  bool read_message(T& msg)
  {
    boost::optional<boost::system::error_code> err;
    boost::asio::async_read(*m_sock, boost::asio::buffer(msg.get_data_ptr(), msg.get_size()), boost::bind(&ClientHandler::read_message_handler, this, _1, boost::ref(err)));
    while(!err && !*m_shouldTerminate) boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    return *m_shouldTerminate ? false : !*err;
  }

  /**
   * \brief Attempts to write a message of type T on the socket used to communicate with the client.
   *
   * This will block until either the write succeeds, an error occurs or the server terminates.
   *
   * \param msg   The T to write.
   * \return      true, if writing succeeded, or false otherwise.
   */
  template <typename T>
  bool write_message(const T& msg)
  {
    boost::optional<boost::system::error_code> err;
    boost::asio::async_write(*m_sock, boost::asio::buffer(msg.get_data_ptr(), msg.get_size()), boost::bind(&ClientHandler::write_message_handler, this, _1, boost::ref(err)));
    while(!err && !*m_shouldTerminate) boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    return *m_shouldTerminate ? false : !*err;
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief The handler called when an asynchronous read of a message finishes.
   *
   * \param err The error code associated with the read.
   * \param ret A location into which to write the error code so that read_message can access it.
   */
  void read_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
  {
    // Store any error message so that it can be examined by read_message.
    ret = err;
  }

  /**
   * \brief The handler called when an asynchronous write of a message finishes.
   *
   * \param err The error code associated with the write.
   * \param ret A location into which to write the error code so that write_message can access it.
   */
  void write_message_handler(const boost::system::error_code& err, boost::optional<boost::system::error_code>& ret)
  {
    // Store any error message so that it can be examined by write_message.
    ret = err;
  }
};

}

#endif
