/**
 * grove: CPURNG.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#ifndef H_GROVE_CPURNG
#define H_GROVE_CPURNG

#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>

#include "ORUtils/MemoryBlock.h"

namespace grove {

/**
 * \brief An instance of this class represents a random number generator.
 *        Note: similar to the RandomNumberGenerator class but simpler and lightweight,
 *        meant to be used as templated RNG in shared code.
 */
class CPURNG
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The generation engine. */
  boost::mt19937 m_gen;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random number generator whose generation engine is seeded with the specified value.
   *
   * \param seed  The seed with which to initialise the generation engine.
   */
  explicit CPURNG(unsigned int seed = 42) :
      m_gen(seed)
  {
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  inline void reset(unsigned int seed)
  {
    m_gen.seed(seed);
  }

  /**
   * \brief Generates a random number from a 1D Gaussian distribution with the specified parameters.
   *
   * \param mean  The mean of the Gaussian distribution.
   * \param sigma The standard deviation of the Gaussian distribution.
   * \return      The generated float.
   */
  template<typename T = float>
  inline T generate_from_gaussian(T mean, T sigma)
  {
    boost::random::normal_distribution<T> dist(mean, sigma);
    return dist(m_gen);
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
  inline int generate_int_from_uniform(int lower, int upper)
  {
    // Note: The Mersenne Twister generation engine can only generate random numbers >= 0.
    boost::random::uniform_int_distribution<> dist(0, upper - lower - 1);
    return dist(m_gen) + lower;
  }

  /**
   * \brief Generates a random real number from a uniform distribution over the specified (closed) range.
   *
   * \param lower The lower bound of the range.
   * \param upper The upper bound of the range.
   * \return      The generated real number.
   */
  template<typename T = float>
  inline T generate_real_from_uniform(T lower, T upper)
  {
    boost::random::uniform_real_distribution<T> dist(lower, upper);
    return dist(m_gen);
  }
};

typedef ORUtils::MemoryBlock<CPURNG> CPURNGMemoryBlock;
typedef boost::shared_ptr<CPURNGMemoryBlock> CPURNGMemoryBlock_Ptr;
typedef boost::shared_ptr<const CPURNGMemoryBlock> CPURNGMemoryBlock_CPtr;
}

#endif
