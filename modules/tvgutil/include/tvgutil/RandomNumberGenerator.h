/**
 * tvgutil: RandomNumberGenerator.h
 */

#ifndef H_TVGUTIL_RANDOMNUMBERGENERATOR
#define H_TVGUTIL_RANDOMNUMBERGENERATOR

#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "Serialization.h"

namespace tvgutil {
class RandomNumberGenerator;
}

namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(Archive& ar, const tvgutil::RandomNumberGenerator *rng, const unsigned int file_version);

template<class Archive>
inline void load_construct_data(Archive& ar, tvgutil::RandomNumberGenerator *rng, const unsigned int file_version);
}}


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

  /** The seed of the random number generator. */
  unsigned int m_seed;

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

  //#################### SERIALIZATION #################### 
public:
  friend class boost::serialization::access;
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    // Intentionally left empty.
  }

  template<class Archive>
  friend void boost::serialization::save_construct_data(Archive& ar, const tvgutil::RandomNumberGenerator *rng, const unsigned int file_version);

  template<class Archive>
  friend void boost::serialization::load_construct_data(Archive& ar, tvgutil::RandomNumberGenerator *rng, const unsigned int file_version);
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RandomNumberGenerator> RandomNumberGenerator_Ptr;

}

namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(Archive& ar, const tvgutil::RandomNumberGenerator *rng, const unsigned int file_version)
{
  std::cout << "Saving RandomNumberGenerator\n";
  // Save the data required to construct instance.
  ar << rng->m_seed;
}

template<class Archive>
inline void load_construct_data(Archive& ar, tvgutil::RandomNumberGenerator *rng, const unsigned int file_version)
{
  std::cout << "Loading RandomNumberGenerator\n";
  // Retrieve data from archive required to construct new instance.
  unsigned int seed;
  ar >> seed;

  // Invoke inplace constructor to initialise instance of class.
  ::new(rng)tvgutil::RandomNumberGenerator(seed);
}
}}

#endif
