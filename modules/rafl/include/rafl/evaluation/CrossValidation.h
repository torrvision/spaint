/**
 * rafl: CrossValidation.h
 */

#ifndef H_RAFL_CROSSVALIDATION
#define H_RAFL_CROSSVALIDATION

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
 * \brief An instance of this class finds the Result of running an Algorithm on a set of Examples via n-fold
 * cross-validation.
 */
template <typename Algorithm, typename Result, typename Label>
class CrossValidation
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
  CrossValidation(size_t num_folds, unsigned int seed)
  : m_num_folds(num_folds), m_rng(seed)
  {
    assert(m_num_folds > 1);
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Runs the cross-validation with a particular Algorithm and set of examples.
   *
   * \param algorithm  A general object whose output is of type Result.
   * \param examples   The vector of examples making up the data on which to evaluate the Algorithm.
   *
   * \return The Result quantifying the average performance of the algorithm over the folds. 
   */
  Result run(Algorithm_Ptr algorithm, const std::vector<Example_CPtr>& examples)
  {
    initialise_splits(examples.size());

    std::vector<Result> results;
    for(size_t i = 0; i < m_num_folds; ++i)
    {
      results.push_back(algorithm->output(examples, m_splits[i]));
    }

    return std::accumulate(results.begin(), results.end(), 0.0)/results.size();
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
   * \brief Initialise the splits of the examples by randomly assigning each example to a particular fold.
   *
   * \param size  The number of examples.
   */
  void initialise_splits(size_t size)
  {
    assert(m_num_folds <= size);

    std::vector<unsigned int> splitLabel(size);
    for(size_t i = 0; i < size; ++i)
    {
      splitLabel[i] = m_rng.generate_int_from_uniform(0, m_num_folds - 1);
    }

#if 1
    std::cout << "splitLabel: \n" << tvgutil::make_limited_container(splitLabel, 20) << "\n";
#endif

    int testFold = 0;
    for(size_t fold = 0; fold < m_num_folds; ++fold)
    {
      Split splitSet;
      for(size_t index = 0; index < size; ++index)
      {
        if(splitLabel[index] != testFold)
        {
          splitSet.first.push_back(index);
        }
        else
        {
          splitSet.second.push_back(index);
        }
      }

      //Make sure the indices are randomly shuffled!
      //TODO pass custom generator?
      std::random_shuffle(splitSet.first.begin(), splitSet.first.end());
      std::random_shuffle(splitSet.second.begin(), splitSet.second.end());

#if 1
      std::cout << "Fold: " << fold << "\n";
      std::cout << "First: \n" << tvgutil::make_limited_container(splitSet.first, 20) << "\n";
      std::cout << "Second: \n" << tvgutil::make_limited_container(splitSet.second, 20) << "\n\n";
#endif

      m_splits.push_back(splitSet);
      ++testFold;
    }
  }
};

}

#endif

