/**
 * tvgutil: Timing.h
 */

#ifndef H_TVGUTIL_TIMING
#define H_TVGUTIL_TIMING

#include <ostream>
#include <stdexcept>
#include <string>

#include <boost/chrono/chrono_io.hpp>

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#endif

namespace tvgutil {

/**
 * \brief TODO
 */
template <typename T>
class Timer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** TODO */
  T m_duration;

  /** The name of the timer. */
  std::string m_name;

  /** TODO */
  boost::chrono::high_resolution_clock::time_point m_t0;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  explicit Timer(const std::string& name)
  : m_name(name), m_t0(boost::chrono::high_resolution_clock::now())
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  const T& duration() const
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
    m_duration = boost::chrono::duration_cast<T>(t1 - m_t0);
  }
};

/**
 * \brief TODO
 */
template <typename T>
class AverageTimer
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of times the event being timed has been executed. */
  size_t m_count;

  /** TODO */
  T m_lastDuration;

  /** The name of the timer. */
  std::string m_name;

  /** TODO */
  boost::chrono::high_resolution_clock::time_point m_t0;

  /** TODO */
  T m_totalDuration;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief TODO
   */
  explicit AverageTimer(const std::string& name)
  : m_count(0), m_name(name)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  T average_duration() const
  {
    if(m_count == 0) throw std::runtime_error("Error: Cannot calculate the average duration of an event until it has been executed at least once");
    return m_totalDuration / m_count;
  }

  /**
   * \brief TODO
   */
  size_t count() const
  {
    return m_count;
  }

  /**
   * \brief TODO
   */
  const T& last_duration() const
  {
    return m_lastDuration;
  }

  /**
   * \brief TODO
   */
  const T& total_duration() const
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
   * \brief TODO
   */
  void start()
  {
    m_t0 = boost::chrono::high_resolution_clock::now();
  }

  /**
   * \brief TODO
   */
  void stop()
  {
    boost::chrono::high_resolution_clock::time_point t1 = boost::chrono::high_resolution_clock::now();
    m_lastDuration = boost::chrono::duration_cast<T>(t1 - m_t0);
    m_totalDuration += m_lastDuration;
    ++m_count;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief TODO
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Timer<T>& rhs)
{
  os << rhs.name() << ": " << rhs.duration();
  return os;
}

//#################### MACROS ####################

#define TIME(target, scale, tag) \
  tvgutil::Timer<boost::chrono::scale> tag(#tag); \
  target; \
  tag.stop();

#define AVG_TIME(target, scale, tag) \
  static tvgutil::AverageTimer<boost::chrono::scale> tag(#tag); \
  tag.start(); \
  target; \
  tag.stop();

#ifdef WITH_CUDA

#define CUDA_TIME(target, scale, tag) \
  tvgutil::Timer<boost::chrono::scale> tag(#tag); \
  target; \
  cudaDeviceSynchronize(); \
  tag.stop();

#define CUDA_AVG_TIME(target, scale, tag) \
  static tvgutil::AverageTimer<boost::chrono::scale> tag(#tag); \
  tag.start(); \
  target; \
  cudaDeviceSynchronize(); \
  tag.stop();

#endif

}

#endif
