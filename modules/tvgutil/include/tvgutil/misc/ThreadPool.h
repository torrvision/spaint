/**
 * tvgutil: ThreadPool.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_TVGUTIL_THREADPOOL
#define H_TVGUTIL_THREADPOOL

#include <string>

#ifdef _MSC_VER
  // Suppress some VC++ warnings that are produced by boost/asio.hpp.
  #pragma warning(disable:4267 4996)
#endif

#include <boost/asio.hpp>

#ifdef _MSC_VER
  // Re-enable the VC++ warnings for the rest of the code.
  #pragma warning(default:4267 4996)
#endif

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace tvgutil {

/**
 * \brief An instance of this class represents a pool of threads that can be used to asynchronously execute arbitrary tasks.
 */
class ThreadPool
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** An I/O service used to schedule work for the threads. */
  boost::asio::io_service m_scheduler;

  /** The threads in the pool. */
  boost::thread_group m_threads;

  /** A worker variable used to keep the scheduler running until we want it to stop. */
  boost::shared_ptr<boost::asio::io_service::work> m_worker;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a thread pool.
   *
   * \param numThreads  The number of threads that should be in the pool.
   */
  explicit ThreadPool(size_t numThreads = 20);

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the thread pool.
   *
   * \note  Since all threads in the pool are joined, this can block.
   */
  ~ThreadPool();

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  ThreadPool(const ThreadPool&);
  ThreadPool& operator=(const ThreadPool&);

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets a global instance of the thread pool that has been constructed with default parameters.
   *
   * This can be used when there is no need to control the lifecycle of the thread pool.
   *
   * \return The default global instance of the thread pool.
   */
  static ThreadPool& instance();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Posts a task to be executed by the thread pool.
   *
   * \param task  The task to execute.
   */
  template <typename Task>
  void post_task(Task task)
  {
    m_scheduler.post(task);
  }
};

}

#endif
