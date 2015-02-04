/**
 * evaluation: RNGFunctor.h
 */

#ifndef H_EVALUATION_RNGFUNCTOR
#define H_EVALUATION_RNGFUNCTOR

#include <tvgutil/RandomNumberGenerator.h>

namespace evaluation {

/**
 * \brief An instance of this struct can be used to forward calls from std::random_shuffle to a random number generator.
 */
struct RNGFunctor
{
  tvgutil::RandomNumberGenerator& m_rng;

  //#################### CONSTRUCTOR ####################

  RNGFunctor(tvgutil::RandomNumberGenerator& rng);

  int operator()(int n) const;
};

}

#endif

