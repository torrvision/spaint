/**
 * grove: CUDARNG.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_GROVE_CUDARNG
#define H_GROVE_CUDARNG

#include <boost/shared_ptr.hpp>

#include <curand_kernel.h>
#include <math_constants.h>

#include "ORUtils/MemoryBlock.h"

namespace grove {

/**
 * \brief An instance of this class can be used to generate random numbers using CUDA.
 *
 * This is a lightweight class for use in shared code.
 */
class CUDARNG
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The random state. */
  curandState_t m_state;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
#ifdef __CUDACC__ // needed to hide __float2int_ru
  /**
   * \brief Generates a random number from a 1D Gaussian distribution with the specified parameters.
   *
   * \param mean  The mean of the Gaussian distribution.
   * \param sigma The standard deviation of the Gaussian distribution.
   * \return      The generated number.
   */
  __device__
  inline float generate_from_gaussian(float mean, float sigma)
  {
    return curand_normal(&m_state) * sigma + mean;
  }

  /**
   * \brief Generates a random integer from a uniform distribution over the specified (closed) range.
   *
   * For example, generate_int_from_uniform(3,5) returns an integer in the range [3,5].
   *
   * \param lower The lower bound of the range.
   * \param upper The upper bound of the range.
   * \return      The generated integer.
   */
  __device__
  inline int generate_int_from_uniform(int lower, int upper)
  {
    // The curand_uniform function generates a number in ]0,1].
    const float generated = curand_uniform(&m_state);

    // The __float2int_ru function rounds the generated values into [1, upper+1-lower],
    // and then adding lower-1 gives the intended range.
    const int result = __float2int_ru(generated * (upper + 1 - lower)) + lower - 1;

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

  /**
   * \brief Reinitialises curand with a new seed and sequence number.
   *
   * \param seed        The seed with which to reinitialise curand.
   * \param sequenceID  The sequence number with which to reinitialise curand.
   */
  __device__
  inline void reset(unsigned int seed, unsigned int sequenceID)
  {
    curand_init(seed, sequenceID, 0, &m_state);
  }
#endif
};

//#################### TYPEDEFS ####################

typedef ORUtils::MemoryBlock<CUDARNG> CUDARNGMemoryBlock;
typedef boost::shared_ptr<CUDARNGMemoryBlock> CUDARNGMemoryBlock_Ptr;
typedef boost::shared_ptr<const CUDARNGMemoryBlock> CUDARNGMemoryBlock_CPtr;

}

#endif
