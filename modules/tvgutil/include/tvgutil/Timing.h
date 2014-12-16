/**
 * tvgutil: Timing.h
 */

#include <boost/chrono/chrono_io.hpp>

#include <cuda_runtime.h>

#define TIME(target, scale) \
  boost::chrono::high_resolution_clock::time_point t0 = boost::chrono::high_resolution_clock::now(); \
  target; \
  boost::chrono::high_resolution_clock::time_point t1 = boost::chrono::high_resolution_clock::now(); \
  boost::chrono::scale dur = boost::chrono::duration_cast<boost::chrono::scale>(t1 - t0);

#define CUDA_TIME(target, scale) \
  boost::chrono::high_resolution_clock::time_point t0 = boost::chrono::high_resolution_clock::now(); \
  target; \
  cudaDeviceSynchronize(); \
  boost::chrono::high_resolution_clock::time_point t1 = boost::chrono::high_resolution_clock::now(); \
  boost::chrono::scale dur = boost::chrono::duration_cast<boost::chrono::scale>(t1 - t0);

#define CUDA_AVG_TIME(target, scale) \
  boost::chrono::high_resolution_clock::time_point t0 = boost::chrono::high_resolution_clock::now(); \
  target; \
  cudaDeviceSynchronize(); \
  boost::chrono::high_resolution_clock::time_point t1 = boost::chrono::high_resolution_clock::now(); \
  boost::chrono::scale thisDur = boost::chrono::duration_cast<boost::chrono::scale>(t1 - t0); \
  static boost::chrono::scale totalDur(0); \
  static int durCount = 0; \
  totalDur += thisDur; \
  ++durCount; \
  boost::chrono::scale dur = totalDur / durCount;

#define TIMEX(target, scale, tag) \
  Timing<boost::chrono::scale> tag(#tag); \
  target; \
  tag.stop();

template <typename T>
class Timing
{
private:
  T m_duration;
  std::string m_name;
  boost::chrono::high_resolution_clock::time_point m_t0, m_t1;

public:
  Timing(std::string name)
  : m_name(name)
  {
    m_t0 = boost::chrono::high_resolution_clock::now();
  }

  T duration() const
  {
    return m_duration;
  }

  void print()
  {
    std::cout << m_name << ": " << m_duration << "\n";
  }

  void stop()
  {
    m_t1 = boost::chrono::high_resolution_clock::now();
    m_duration = boost::chrono::duration_cast<T>(m_t1 - m_t0);
  }
};


