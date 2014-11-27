/**
 * tvgutil: RandomNumberGenerator.cpp
 */

#include "RandomNumberGenerator.h"

namespace tvgutil {

//#################### CONSTRUCTORS ####################

RandomNumberGenerator::RandomNumberGenerator(unsigned int seed)
: m_gen(seed)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

int RandomNumberGenerator::generate_int_from_uniform(int lower, int upper)
{
  boost::lock_guard<boost::mutex> lock(m_mutex);

  // Note: The Mersenne Twister generation engine can only generate random numbers >= 0.
  boost::random::uniform_int_distribution<> dist(0, upper - lower);
  return dist(m_gen) + lower;
}

}
