/**
 * tvgutil: ThreadPool.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "misc/ThreadPool.h"

#include <boost/bind.hpp>

namespace tvgutil {

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

ThreadPool::ThreadPool(size_t num_threads) :
    m_worker(m_scheduler)
{
  for (size_t i = 0; i < num_threads; ++i)
  {
    m_threadpool.create_thread(boost::bind(&boost::asio::io_service::run, &m_scheduler));
  }
}

}
