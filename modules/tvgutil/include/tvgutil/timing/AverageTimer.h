/**
 * tvgutil: AverageTimer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_AVERAGETIMER
#define H_TVGUTIL_AVERAGETIMER

#include <ostream>
#include <stdexcept>
#include <string>

#include <boost/chrono/chrono_io.hpp>

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#endif

namespace tvgutil {

/**
 * \brief An instance of an instantiation of this class template represents a timer that can be
 *        used to time an event over multiple runs and compute the average time taken per run.
 */
template <typename Scale>
class AverageTimer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of times the event being timed has been executed. */
  size_t m_count;

  /** The time taken by the event on its most recent run. */
  Scale m_lastDuration;

  /** The name of the timer. */
  std::string m_name;

  /** The starting time for the event's most recent run. */
  boost::chrono::high_resolution_clock::time_point m_t0;

  /** The time taken by the event over all runs up to this point. */
  Scale m_totalDuration;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a timer with the specified name.
   *
   * \param name  The name of the timer.
   */
  explicit AverageTimer(const std::string& name)
  : m_count(0), m_name(name), m_totalDuration(0)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Computes the average time taken by the event on each run.
   *
   * \return  The average time taken by the event on each run, or zero if the event has never run.
   */
  Scale average_duration() const
  {
    return m_count != 0 ? Scale(m_totalDuration / m_count) : Scale(0);
  }

  /**
   * \brief Gets the number of times the event being timed has been executed.
   *
   * return The number of times the event being timed has been executed.
   */
  size_t count() const
  {
    return m_count;
  }

  /**
   * \brief Gets the time taken by the event on its most recent run.
   *
   * \return  The time taken by the event on its most recent run.
   */
  const Scale& last_duration() const
  {
    return m_lastDuration;
  }

  /**
   * \brief Gets the time taken by the event over all runs up to this point.
   *
   * \return  The time taken by the event over all runs up to this point.
   */
  const Scale& total_duration() const
  {
    return m_totalDuration;
  }

  /**
   * \brief Gets the name of the timer.
   *
   * \return  The name of the timer.
   */
  const std::string& name() const
  {
    return m_name;
  }

  /**
   * \brief Starts the timer (this should be called prior before each run of the event).
   */
  void start()
  {
    m_t0 = boost::chrono::high_resolution_clock::now();
  }

  /**
   * \brief Stops the timer (this should be called after each run of the event).
   */
  void stop()
  {
    boost::chrono::high_resolution_clock::time_point t1 = boost::chrono::high_resolution_clock::now();
    m_lastDuration = boost::chrono::duration_cast<Scale>(t1 - m_t0);
    m_totalDuration += m_lastDuration;
    ++m_count;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs the specified timer to a stream.
 *
 * \param os  The stream.
 * \param rhs The timer.
 * \return    The stream.
 */
template <typename Scale>
std::ostream& operator<<(std::ostream& os, const AverageTimer<Scale>& rhs)
{
  os << rhs.name() << ": " << rhs.last_duration() << ' ' << rhs.average_duration();
  return os;
}

//#################### MACROS ####################

#define AVG_TIME(target, scale, tag) \
  static tvgutil::AverageTimer<boost::chrono::scale> tag(#tag); \
  tag.start(); \
  target; \
  tag.stop()

#ifdef WITH_CUDA
#define CUDA_AVG_TIME(target, scale, tag) \
  static tvgutil::AverageTimer<boost::chrono::scale> tag(#tag); \
  tag.start(); \
  target; \
  cudaDeviceSynchronize(); \
  tag.stop()
#endif

}

#endif
