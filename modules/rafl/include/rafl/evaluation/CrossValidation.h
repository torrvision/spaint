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
 * \brief An instance of this class finds the QuantitativePerformance of running an Algorithm on a set of Examples via n-fold
 * cross-validation.
 */
template <typename Algorithm, typename QuantitativePerformance, typename Label>
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
  size_t m_foldCount;

  /** A random number generator. */
  tvgutil::RandomNumberGenerator m_rng;

  /** Splits of the data. */
  Splits m_splits;

  //#################### CONSTRUCTOR ####################
public:
  /**
   * \brief Creates a general cross-validation object.
   *
   * \param foldCount  The number of folds with which to split the dataset of examples.
   * \param seed       The seed of the random number generator.
   */
  CrossValidation(size_t foldCount, unsigned int seed)
  : m_foldCount(foldCount), m_rng(seed)
  {
    assert(m_foldCount > 1);
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Runs the cross-validation with a particular Algorithm and set of examples.
   *
   * \param algorithm  A general object whose output is of type QuantitativePerformance.
   * \param examples   The vector of examples making up the data on which to evaluate the Algorithm.
   *
   * \return           The QuantitativePerformance quantifying the average performance of the algorithm over the folds.
   */
  QuantitativePerformance run(Algorithm_Ptr algorithm, const std::vector<Example_CPtr>& examples)
  {
    initialise_splits(examples.size());

    std::vector<QuantitativePerformance> performance;
    for(size_t i = 0; i < m_foldCount; ++i)
    {
      performance.push_back(algorithm->cross_validation_offline_output(examples, m_splits[i]));
    }

    QuantitativePerformance averagePerformance = QuantitativePerformance::average(performance);

    return averagePerformance;
  }

  /**
   * \brief Gets the number of folds.
   *
   * \return The number of folds.
   */
  size_t foldCount() const
  {
    return m_foldCount;
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Initialise the splits of the examples by randomly assigning each example to a particular fold.
   *
   * \param size  The number of examples.
   */
  void initialise_splits(size_t size)
  {
    assert(m_foldCount <= size);

    //Generate a vector which will be used to assign an example to a random fold.
    std::vector<unsigned int> splitLabel(size);
    for(size_t i = 0; i < size; ++i)
    {
      splitLabel[i] = m_rng.generate_int_from_uniform(0, m_foldCount - 1);
    }

#if 1
    std::cout << "splitLabel: \n" << tvgutil::make_limited_container(splitLabel, 20) << "\n";
#endif

    //For each fold, split the data in two, the first set contains all examples except those indicated by validationFold.
    int validationFold = 0;
    for(size_t fold = 0; fold < m_foldCount; ++fold)
    {
      Split splitSet;
      for(size_t index = 0; index < size; ++index)
      {
        if(splitLabel[index] != validationFold)
        {
          splitSet.first.push_back(index);
        }
        else
        {
          splitSet.second.push_back(index);
        }
      }

      //Make sure the indices are randomly shuffled!
      //FIXME pass custom generator?
      std::random_shuffle(splitSet.first.begin(), splitSet.first.end());
      std::random_shuffle(splitSet.second.begin(), splitSet.second.end());

#if 1
      std::cout << "Fold: " << fold << "\n";
      std::cout << "First: \n" << tvgutil::make_limited_container(splitSet.first, 20) << "\n";
      std::cout << "Second: \n" << tvgutil::make_limited_container(splitSet.second, 20) << "\n\n";
#endif

      m_splits.push_back(splitSet);
      ++validationFold;
    }
  }
};

}

#endif

