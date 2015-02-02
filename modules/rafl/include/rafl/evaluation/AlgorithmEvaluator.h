/**
 * rafl: AlgorithmEvaluator.h
 */

#ifndef H_RAFL_ALGORITHMEVALUATOR
#define H_RAFL_ALGORITHMEVALUATOR

#include "SplitGenerator.h"

namespace rafl {

/**
 * \brief An instance of a class deriving from an instantiation of this class template can be used to evaluate a learning algorithm.
 */
template <typename Example, typename Result>
class AlgorithmEvaluator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The split generator to use. */
  mutable SplitGenerator_Ptr m_splitGenerator;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs an algorithm evaluator.
   *
   * \param splitGenerator  The split generator to use.
   */
  explicit AlgorithmEvaluator(const SplitGenerator_Ptr& splitGenerator)
  : m_splitGenerator(splitGenerator)
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the algorithm evaluator.
   */
  virtual ~AlgorithmEvaluator() {}

  //#################### PROTECTED ABSTRACT MEMBER FUNCTIONS ####################
protected:
  /**
   * \brief Averages the individual results from the various splits.
   *
   * \param results The results from the various splits.
   * \return        The average of the results.
   */
  virtual Result average_results(const std::vector<Result>& results) const = 0;

  /**
   * \brief Evaluates the algorithm on the specified split of examples.
   *
   * \param examples  The examples on which to evaluate the algorithm.
   * \param split     The way in which the examples should be split into training and validation sets.
   * \return          The result of evaluating the algorithm on the specified split.
   */
  virtual Result evaluate_on_split(const std::vector<Example_CPtr>& examples, const SplitGenerator::Split& split) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Evaluates the algorithm on the specified set of examples.
   *
   * \param examples  The examples on which to evaluate the algorithm.
   * \return          The result of the evaluation process.
   */
  Result evaluate(const std::vector<Example_CPtr>& examples) const
  {
    std::vector<Result> results;

    std::vector<SplitGenerator::Split> splits = m_splitGenerator->generate_splits();
    for(size_t i = 0, size = splits.size(); i < size; ++i)
    {
      results.push_back(evaluate_on_split(examples, splits[i]));
    }

    return average_results(results);
  }
};

}

#endif
