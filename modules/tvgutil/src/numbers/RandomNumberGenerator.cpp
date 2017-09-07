/**
 * tvgutil: RandomNumberGenerator.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "numbers/RandomNumberGenerator.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

RandomNumberGenerator::RandomNumberGenerator(unsigned int seed)
: m_gen(new boost::mt19937(seed)), m_seed(seed)
{}

RandomNumberGenerator::RandomNumberGenerator()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int RandomNumberGenerator::generate_int_from_uniform(int lower, int upper)
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  // Note: The Mersenne Twister generation engine can only generate random numbers >= 0.
  boost::random::uniform_int_distribution<> dist(0, upper - lower);
  return dist(*m_gen) + lower;
}

}
