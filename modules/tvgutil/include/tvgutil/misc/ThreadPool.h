/**
 * tvgutil: ThreadPool.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_TVGUTIL_THREADPOOL
#define H_TVGUTIL_THREADPOOL

#include <string>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace tvgutil {

/**
 * \brief This class represents a pool of threads that can be used to asynchronously execute
 *        arbitrary functions.
 */
class ThreadPool
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief  Returns a static instance of the ThreadPool constructed with default parameters.
   *         Can be used when there is no need to determine the lifecycle of the thread pool.
   *
   * \return A static instance of the ThreadPool constructed with default parameters.
   */
  static ThreadPool& instance();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
  /**
   * \brief Starts a function on a separate thread.
   *
   * \param task  The task that will executed on a thread part of the pool.
   */
  template<typename F>
  void start_asynch(F task)
  {
    m_scheduler.post(task);
  }

  //#################### CONSTRUCTOR ####################
  /**
   * \brief Constructs a ThreadPool.
   *
   * \param num_threads The number of threads part of the pool.
   */
  explicit ThreadPool(size_t num_threads = 20);

  //#################### DESTRUCTOR ####################
  /**
   * \brief Destructs a ThreadPool. All threads in the pool are joined.
   *
   * \note  Can be blocking.
   */
  ~ThreadPool();

private:
  // Disable copy and assignment operations.
  ThreadPool(const ThreadPool &);
  const ThreadPool& operator=(const ThreadPool &);


  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  // io_service used to schedule work for the threads.
  boost::asio::io_service m_scheduler;

  // Contains all the threads part of the pool.
  boost::thread_group m_threadpool;

  // Worker variable used to keep the scheduler running.
  boost::shared_ptr<boost::asio::io_service::work> m_worker;
};

}

#endif
