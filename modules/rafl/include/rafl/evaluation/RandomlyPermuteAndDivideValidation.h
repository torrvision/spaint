/**
 * rafl: RandomlyPermuteAndDivideValidation.h
 */

#ifndef H_RAFL_RANDOMLYPERMUTEANDDIVIDEVALIDATION
#define H_RAFL_RANDOMLYPERMUTEANDDIVIDEVALIDATION

#include <cassert>
#include <algorithm>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <tvgutil/RandomNumberGenerator.h>
#include <tvgutil/LimitedContainer.h>

#include "../examples/Example.h"

namespace rafl {

/**
 * \brief An instance of this class finds the quantitative performance of running an Algorithm on a set of Examples via the following procedure:
 * Repeat the following procedure n times:
 * First randomly permute the example indices,
 * then divide them into two sets where the first set has a percentage of the examples,
 * and the second set the remaining examples.
 */
template <typename Algorithm, typename QuantitativePerformance, typename Label>
class RandomlyPermuteAndDivideValidation
{
  //#################### PRIVATE TYPEDEFS ####################
private:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;
  typedef boost::shared_ptr<Algorithm> Algorithm_Ptr;
  typedef std::vector<size_t> Indices;
  typedef std::pair<Indices,Indices> Split;
  typedef std::vector<Split> Splits;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The number of folds. */
  size_t m_num_folds;

  /** The ratio [number of examples in the training set]/[number of examples in the validation set]. */
  float m_ratio;

  /** A random number generator. */
  tvgutil::RandomNumberGenerator m_rng;

  /** Splits of the data. */
  Splits m_splits;

  //#################### CONSTRUCTOR ####################
public:
  /**
   * \brief Creates a general cross-validation object.
   *
   * \param num_folds  The number of folds with which to split the dataset of examples.
   * \param seed       The seed of the random number generator.
   */
  RandomlyPermuteAndDivideValidation(float ratio, size_t num_folds, unsigned int seed)
  : m_num_folds(num_folds), m_ratio(ratio), m_rng(seed)
  {
    assert(m_num_folds > 1);
    assert((m_ratio > 0.0f) && (m_ratio < 1.0f));
    if((m_ratio < 0.1f)||(m_ratio > 0.9f)) std::cout << "Warning: the ratio value being used: " << m_ratio << " seems unwise.\n";
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Runs the randomly-permute-and-divide-validation with a particular Algorithm and set of examples.
   *
   * \param algorithm  A general object whose output is of type QuantitativePerformance.
   * \param examples   The vector of examples making up the data on which to evaluate the Algorithm.
   *
   * \return The average quantitative performance of the algorithm over the folds.
   */
  QuantitativePerformance run(Algorithm_Ptr algorithm, const std::vector<Example_CPtr>& examples)
  {
    initialise_splits(examples.size());

    std::vector<QuantitativePerformance> performance;
    for(size_t i = 0; i < m_num_folds; ++i)
    {
      performance.push_back(algorithm->cross_validation_offline_output(examples, m_splits[i]));
    }

    return QuantitativePerformance::average(performance);
  }

  /**
   * \brief Access the number of folds.
   *
   * \return The number of folds.
   */
  size_t num_folds() const
  {
    return m_num_folds;
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialise the splits of the examples by repeatedly randomly permuting the example indices and splitting them according to a predefined ratio.
   *
   * \param size  The number of examples.
   */
  void initialise_splits(size_t size)
  {
    assert(size > 2);

    std::vector<unsigned int> exampleIndices(size);
    for(size_t i = 0; i < size; ++i)
    {
      exampleIndices[i] = i;
    }

#if 1
    std::cout << "exampleIncides: \n" << tvgutil::make_limited_container(exampleIndices, 20) << "\n";
#endif

    size_t firstSetSize = m_ratio*size;
    for(size_t fold = 0; fold < m_num_folds; ++fold)
    {
      Split splitSet;

      //Randomly shuffle the indices.
      std::random_shuffle(exampleIndices.begin(), exampleIndices.end());
      splitSet.first.insert(splitSet.first.begin(), exampleIndices.begin(), exampleIndices.begin() + firstSetSize);
      splitSet.second.insert(splitSet.second.begin(), exampleIndices.begin() + firstSetSize + 1, exampleIndices.end());

#if 1
      std::cout << "First: \n" << tvgutil::make_limited_container(splitSet.first, 20) << "\n";
      std::cout << "Second: \n" << tvgutil::make_limited_container(splitSet.second, 20) << "\n\n";
#endif

      m_splits.push_back(splitSet);
    }
  }
};

} //end namespace rafl

#endif

