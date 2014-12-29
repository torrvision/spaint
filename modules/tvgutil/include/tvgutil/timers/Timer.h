/**
 * tvgutil: Timer.h
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

#ifdef WITH_CUDA
#define CUDA_TIME(target, scale, tag) \
  tvgutil::Timer<boost::chrono::scale> tag(#tag); \
  target; \
  cudaDeviceSynchronize(); \
  tag.stop();
#endif

}

#endif
