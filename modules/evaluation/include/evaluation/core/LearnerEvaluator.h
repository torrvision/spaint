/**
 * evaluation: LearnerEvaluator.h
 */

#ifndef H_EVALUATION_LEARNEREVALUATOR
#define H_EVALUATION_LEARNEREVALUATOR

#include "../splitgenerators/SplitGenerator.h"

namespace evaluation {

/**
 * \brief An instance of a class deriving from an instantiation of this class template can be used to
 *        evaluate a learner (e.g. a random forest) using approaches based on example set splitting.
 */
template <typename Example, typename Result>
class LearnerEvaluator
{
  //#################### TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<const Example> Example_CPtr;
  typedef Result ResultType;

  //#################### PRIVATE VARIABLES ####################
private:
  /** The generator to use to split the example set. */
  mutable SplitGenerator_Ptr m_splitGenerator;

  //#################### CONSTRUCTORS ####################
protected:
  /**
   * \brief Constructs a learner evaluator.
   *
   * \param splitGenerator  The generator to use to split the example set.
   */
  explicit LearnerEvaluator(const SplitGenerator_Ptr& splitGenerator)
  : m_splitGenerator(splitGenerator)
  {}

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the learner evaluator.
   */
  virtual ~LearnerEvaluator() {}

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
   * \brief Evaluates the learner on the specified split of examples.
   *
   * \param examples  The examples on which to evaluate the learner.
   * \param split     The way in which the examples should be split into training and validation sets.
   * \return          The result of evaluating the learner on the specified split.
   */
  virtual Result evaluate_on_split(const std::vector<Example_CPtr>& examples, const SplitGenerator::Split& split) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Evaluates the learner on the specified set of examples.
   *
   * \param examples  The examples on which to evaluate the learner.
   * \return          The result of the evaluation process.
   */
  Result evaluate(const std::vector<Example_CPtr>& examples) const
  {
    std::vector<Result> results;

    std::vector<SplitGenerator::Split> splits = m_splitGenerator->generate_splits(examples.size());
    for(size_t i = 0, size = splits.size(); i < size; ++i)
    {
      results.push_back(evaluate_on_split(examples, splits[i]));
    }

    return average_results(results);
  }
};

}

#endif
