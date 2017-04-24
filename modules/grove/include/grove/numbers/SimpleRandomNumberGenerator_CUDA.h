/**
 * grove: SimpleRandomNumberGenerator_CUDA.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_GROVE_SIMPLERANDOMNUMBERGENERATORCUDA
#define H_GROVE_SIMPLERANDOMNUMBERGENERATORCUDA

#include <boost/shared_ptr.hpp>
#include <curand_kernel.h>
#include <math_constants.h>

#include "ORUtils/MemoryBlock.h"
#include "ORUtils/PlatformIndependence.h"

namespace grove {

/**
 * \brief An instance of this class represents a CUDA random number generator.
 *        Note: similar to the RandomNumberGenerator class but simpler and lightweight,
 *        meant to be used as templated RNG in shared code.
 */
class SimpleRandomNumberGenerator_CUDA
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The random state. */
  curandState_t m_state;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random number generator that can be invoked from CUDA kernels.
   */
  SimpleRandomNumberGenerator_CUDA()
  {
  }

#ifdef __CUDACC__ // Needed to hide CUDART_MIN_DENORM_F and __float2int_rz
  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  __device__
  inline void reset(unsigned int seed, unsigned int sequenceId)
  {
    curand_init(seed, sequenceId, 0, &m_state);
  }

  /**
   * \brief Generates a random number from a 1D Gaussian distribution with the specified parameters.
   *
   * \param mean  The mean of the Gaussian distribution.
   * \param sigma The standard deviation of the Gaussian distribution.
   * \return      The generated float.
   */
  __device__
  inline float generate_from_gaussian(float mean, float sigma)
  {
    return curand_normal(&m_state) * sigma + mean;
  }

  /**
   * \brief Generates a random integer from a uniform distribution over the specified (open) range.
   *
   * For example, generate_int_from_uniform(3,5) returns an integer in the range [3,5[.
   *
   * \param lower The lower bound of the range.
   * \param upper The upper bound of the range.
   * \return      The generated integer.
   */
  __device__
  inline int generate_int_from_uniform(int lower, int upper)
  {
    // curand_uniform generates a number in ]0,1]
    const float generated = curand_uniform(&m_state);
    // __float2int_ru rounds the generated values in ]lower, upper], subtracting 1 gives the intended range
    const int result = __float2int_ru(generated * (upper - lower)) + lower - 1;
    return result;
  }

  /**
   * \brief Generates a random real number from a uniform distribution over the specified (closed) range.
   *
   * \param lower The lower bound of the range.
   * \param upper The upper bound of the range.
   * \return      The generated real number.
   */
  __device__
  inline float generate_real_from_uniform(float lower, float upper)
  {
    return curand_uniform(&m_state) * (upper - lower) + lower;
  }
#endif
};

typedef SimpleRandomNumberGenerator_CUDA CUDARNG;
typedef ORUtils::MemoryBlock<CUDARNG> CUDARNGMemoryBlock;
typedef boost::shared_ptr<CUDARNGMemoryBlock> CUDARNGMemoryBlock_Ptr;
typedef boost::shared_ptr<const CUDARNGMemoryBlock> CUDARNGMemoryBlock_CPtr;

}

#endif
