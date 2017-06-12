/**
 * tvgutil: RandomNumberGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_RANDOMNUMBERGENERATOR
#define H_TVGUTIL_RANDOMNUMBERGENERATOR

#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <boost/serialization/split_member.hpp>

namespace tvgutil {

/**
 * \brief An instance of this class represents a thread-safe random number generator.
 */
class RandomNumberGenerator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The generation engine. */
  boost::shared_ptr<boost::mt19937> m_gen;

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

private:
  /**
   * \brief Constructs a random number generator.
   *
   * Note: This constructor is needed for serialization and should not be used otherwise.
   */
  RandomNumberGenerator();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Generates a random number from a 1D Gaussian distribution with the specified parameters.
   *
   * \param mean  The mean of the Gaussian distribution.
   * \param sigma The standard deviation of the Gaussian distribution.
   * \return      The generated number.
   */
  template <typename T = float>
  T generate_from_gaussian(T mean, T sigma)
  {
    boost::lock_guard<boost::mutex> lock(m_mutex);
    boost::random::normal_distribution<T> dist(mean, sigma);
    return dist(*m_gen);
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
    return dist(*m_gen);
  }

  //#################### SERIALIZATION #################### 
public:
  /**
   * \brief Loads the random number generator from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void load(Archive& ar, const unsigned int version)
  {
    ar & m_seed;
    m_gen.reset(new boost::mt19937(m_seed));
  }

  /**
   * \brief Saves the random number generator to an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const
  {
    ar & m_seed;
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()

  friend class boost::serialization::access;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<RandomNumberGenerator> RandomNumberGenerator_Ptr;
typedef boost::shared_ptr<const RandomNumberGenerator> RandomNumberGenerator_CPtr;

}

#endif
