/**
 * tvgutil: Timer.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_TIMER
#define H_TVGUTIL_TIMER

#include <ostream>
#include <string>

#include <boost/chrono/chrono_io.hpp>

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#endif

namespace tvgutil {

/**
 * \brief An instance of an instantiation of this class template represents a timer that can be used to time an event.
 */
template <typename Scale>
class Timer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The time taken by the event (set when stop is called). */
  Scale m_duration;

  /** The name of the timer. */
  std::string m_name;

  /** The starting time for the event. */
  boost::chrono::high_resolution_clock::time_point m_t0;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a timer with the specified name, and starts it.
   *
   * \param name  The name of the timer.
   */
  explicit Timer(const std::string& name)
  : m_name(name), m_t0(boost::chrono::high_resolution_clock::now())
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the time taken by the event.
   *
   * \return  The time taken by the event.
   */
  const Scale& duration() const
  {
    return m_duration;
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
   * \brief Stops the timer.
   */
  void stop()
  {
    boost::chrono::high_resolution_clock::time_point t1 = boost::chrono::high_resolution_clock::now();
    m_duration = boost::chrono::duration_cast<Scale>(t1 - m_t0);
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
std::ostream& operator<<(std::ostream& os, const Timer<Scale>& rhs)
{
  os << rhs.name() << ": " << rhs.duration();
  return os;
}

//#################### MACROS ####################

#define TIME(target, scale, tag) \
  tvgutil::Timer<boost::chrono::scale> tag(#tag); \
  target; \
  tag.stop()

#ifdef WITH_CUDA
#define CUDA_TIME(target, scale, tag) \
  tvgutil::Timer<boost::chrono::scale> tag(#tag); \
  target; \
  cudaDeviceSynchronize(); \
  tag.stop()
#endif

}

#endif
