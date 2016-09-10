/**
 * spaint: ThreadPool.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "util/ThreadPool.h"

#include <boost/bind.hpp>

namespace spaint {
ThreadPool& ThreadPool::instance()
{
  static ThreadPool instance;
  return instance;
}

ThreadPool::~ThreadPool()
{
  m_scheduler.stop();
  m_threadpool.join_all();
}

ThreadPool::ThreadPool() :
    m_worker(m_scheduler)
{
  static const int POOL_SIZE = 20;
  for (int i = 0; i < POOL_SIZE; ++i)
  {
    m_threadpool.create_thread(boost::bind(&boost::asio::io_service::run, &m_scheduler));
  }
}
}
