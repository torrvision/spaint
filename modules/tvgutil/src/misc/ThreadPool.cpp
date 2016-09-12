/**
 * tvgutil: ThreadPool.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "misc/ThreadPool.h"

#include <boost/bind.hpp>

namespace tvgutil {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
ThreadPool& ThreadPool::instance()
{
  // Use a default constructed instance of the ThreadPool.
  static ThreadPool instance;

  return instance;
}

//#################### CONSTRUCTOR ####################
ThreadPool::ThreadPool(size_t num_threads) :
    m_worker(new boost::asio::io_service::work(m_scheduler))
{
  // Create num_threads threads all running the run() function of the io_service.
  for (size_t i = 0; i < num_threads; ++i)
  {
    m_threadpool.create_thread(boost::bind(&boost::asio::io_service::run, &m_scheduler));
  }
}

//#################### DESTRUCTOR ####################
ThreadPool::~ThreadPool()
{
  // Stop executing the io_service::run() function but allow running and queued tasks to finish cleanly.
  m_worker.reset();
  // Wait for all threads to terminate.
  m_threadpool.join_all();
  //Cleanly stop the io_service
  m_scheduler.stop();
}

}
