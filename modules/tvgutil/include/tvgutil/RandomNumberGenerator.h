/**
 * tvgutil: RandomNumberGenerator.h
 */

#ifndef H_TVGUTIL_RANDOMNUMBERGENERATOR
#define H_TVGUTIL_RANDOMNUMBERGENERATOR

#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>
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
   * \brief Constructs a random number generator whose generation engine is seeded with the specified value.
   *
   * \param seed  The seed with which to initialise the generation engine.
   */
  explicit RandomNumberGenerator(unsigned int seed);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a random number from a 1D Gaussian distribution with the specified parameters.
   *
   * \param mean  The mean of the Gaussian distribution.
   * \param sigma The standard deviation of the Gaussian distribution.
   * \return      The generated float.
   */
  template <typename T = float>
  T generate_from_gaussian(T mean, T sigma)
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    boost::random::normal_distribution<T> dist(mean, sigma);
    return dist(m_gen);
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
  int generate_int_from_uniform(int lower, int upper);

  /**
   * \brief Generates a random real number from a uniform distribution over the specified (closed) range.
   *
   * \param lower The lower bound of the range.
   * \param upper The upper bound of the range.
   * \return      The generated real number.
   */
  template <typename T = float>
  T generate_real_from_uniform(T lower, T upper)
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    boost::random::uniform_real_distribution<T> dist(lower, upper);
    return dist(m_gen);
  }
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RandomNumberGenerator> RandomNumberGenerator_Ptr;

}

#endif
