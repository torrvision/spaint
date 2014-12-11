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
