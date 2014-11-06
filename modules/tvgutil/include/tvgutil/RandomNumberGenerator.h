/**
 * tvgutil: RandomNumberGenerator.h
 */

#ifndef H_TVGUTIL_RANDOMNUMBERGENERATOR
#define H_TVGUTIL_RANDOMNUMBERGENERATOR

#include <boost/random.hpp>
#include <boost/thread.hpp>

namespace tvgutil {

/**
 * \brief An instance of this class represents a thread-safe random number generator.
 */
class RandomNumberGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The generation engine. */
  boost::mt19937 m_gen;

  /** The mutex used to synchronise access to the random number generator. */
  boost::mutex m_mutex;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a random number generator that is seeded with the specified value.
   *
   * \param seed  The seed with which to initialise the generation engine.
   */
  explicit RandomNumberGenerator(unsigned int seed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates an integer in the specified range.
   *
   * \param lower The lower bound of the range.
   * \param upper The upper bound of the range.
   * \return      The generated integer.
   */
  int generate_int_in_range(int lower, int upper);
};

}

#endif
