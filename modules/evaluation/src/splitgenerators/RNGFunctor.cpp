/**
 * evaluation: RNGFunctor.cpp
 */

#include "splitgenerators/RNGFunctor.h"

namespace evaluation {

//#################### CONSTRUCTOR ####################

RNGFunctor::RNGFunctor(tvgutil::RandomNumberGenerator& rng)
: m_rng(rng)
{}

int RNGFunctor::operator()(int n) const
{
  return m_rng.generate_int_from_uniform(0, n - 1);
}

}

