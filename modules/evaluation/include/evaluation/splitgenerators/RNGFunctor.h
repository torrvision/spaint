/**
 * evaluation: RNGFunctor.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_EVALUATION_RNGFUNCTOR
#define H_EVALUATION_RNGFUNCTOR

#include <tvgutil/numbers/RandomNumberGenerator.h>

namespace evaluation {

/**
 * \brief An instance of this struct can be used to forward calls from std::random_shuffle to a random number generator.
 *
 * Note that these are deliberately intended to be used locally and then discarded. In particular, they must not survive
 * longer than the random number generator to which they refer.
 */
struct RNGFunctor
{
  //#################### PUBLIC VARIABLES ####################

  /** The random number generator to which to forward calls. */
  tvgutil::RandomNumberGenerator& m_rng;

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs an instance of the functor.
   *
   * \param rng The random number generator to which to forward calls.
   */
  explicit RNGFunctor(tvgutil::RandomNumberGenerator& rng);

  //#################### PUBLIC OPERATORS ####################

  /**
   * \brief Generates a random number between 0 and n - 1.
   *
   * \param n The upper bound of the range in which to generate a random number.
   * \return  The generated random number.
   */
  int operator()(int n) const;
};

}

#endif
