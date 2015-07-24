/**
 * evaluation: RNGFunctor.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "splitgenerators/RNGFunctor.h"

namespace evaluation {

//#################### CONSTRUCTORS ####################

RNGFunctor::RNGFunctor(tvgutil::RandomNumberGenerator& rng)
: m_rng(rng)
{}

//#################### PUBLIC OPERATORS ####################

int RNGFunctor::operator()(int n) const
{
  return m_rng.generate_int_from_uniform(0, n - 1);
}

}
