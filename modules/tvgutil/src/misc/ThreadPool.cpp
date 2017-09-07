/**
 * tvgutil: ThreadPool.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "misc/ThreadPool.h"

#include <boost/bind.hpp>

namespace tvgutil {

//#################### CONSTRUCTORS ####################

ThreadPool::ThreadPool(size_t numThreads)
: m_worker(new boost::asio::io_service::work(m_scheduler))
{
  for(size_t i = 0; i < numThreads; ++i)
  {
    m_threads.create_thread(boost::bind(&boost::asio::io_service::run, &m_scheduler));
  }
}

//#################### DESTRUCTOR ####################

ThreadPool::~ThreadPool()
{
  // Stop executing the scheduler's run function, but allow running and queued tasks to finish cleanly.
  m_worker.reset();

  // Wait for all threads to terminate.
  m_threads.join_all();

  // Cleanly stop the scheduler.
  m_scheduler.stop();
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ThreadPool& ThreadPool::instance()
{
  static ThreadPool s_instance;
  return s_instance;
}

}
