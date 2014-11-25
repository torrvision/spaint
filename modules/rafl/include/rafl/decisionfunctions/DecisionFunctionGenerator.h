/**
 * rafl: DecisionFunctionGenerator.h
 */

#ifndef H_RAFL_DECISIONFUNCTIONGENERATOR
#define H_RAFL_DECISIONFUNCTIONGENERATOR

#include <utility>

#include "../base/ProbabilityMassFunction.h"
#include "../examples/ExampleReservoir.h"
#include "../examples/ExampleUtil.h"
#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of a class template deriving from this one can be used to pick an appropriate decision function to split a set of examples.
 */
template <typename Label>
class DecisionFunctionGenerator
{
  //#################### PROTECTED TYPEDEFS ####################
protected:
  typedef boost::shared_ptr<const Example<Label> > Example_CPtr;

  //#################### NESTED TYPES ####################
public:
  /**
   * \brief An instance of this struct represents a split of a set of examples into two subsets,
   *        based on their classification against a decision function.
   */
  struct Split
  {
    /** The decision function that induced the split. */
    DecisionFunction_Ptr m_decisionFunction;

    /** The examples that were sent left by the decision function. */
    std::vector<Example_CPtr> m_leftExamples;

    /** The examples that were sent right by the decision function. */
    std::vector<Example_CPtr> m_rightExamples;
  };

  //#################### PUBLIC TYPEDEFS ####################
public:
  typedef boost::shared_ptr<Split> Split_Ptr;
  typedef boost::shared_ptr<const Split> Split_CPtr;

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the generator.
   */
  virtual ~DecisionFunctionGenerator() {}

  //#################### PRIVATE ABSTRACT MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Generates a candidate decision function to split the specified set of examples.
   *
   * \param examples  The examples to split.
   * \return          The candidate decision function.
   */
  virtual DecisionFunction_Ptr generate_candidate_decision_function(const std::vector<Example_CPtr>& examples) const = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Tries to pick an appropriate way in which to split the specified reservoir of examples.
   *
   * \param reservoir       The reservoir of examples to split.
   * \param candidateCount  The number of candidates to evaluate.
   * \param gainThreshold   The minimum information gain that must be obtained from a split to make it worthwhile.
   * \return                The chosen split, if one was suitable, or NULL otherwise.
   */
  Split_CPtr split_examples(const ExampleReservoir<Label>& reservoir, int candidateCount, float gainThreshold) const
  {
    float initialEntropy = ExampleUtil::calculate_entropy(*reservoir.get_histogram());
    std::multimap<float,Split_Ptr,std::greater<float> > gainToCandidateMap;

#if 0
    std::cout << "\nP: " << *reservoir.get_histogram() << ' ' << initialEntropy << '\n';
#endif

    std::vector<Example_CPtr> examples = reservoir.get_examples();
    for(int i = 0; i < candidateCount; ++i)
    {
      Split_Ptr splitCandidate(new Split);

      // Generate a decision function for the split candidate.
      splitCandidate->m_decisionFunction = generate_candidate_decision_function(examples);

#if 0
      std::cout << *splitCandidate->m_decisionFunction << '\n';
#endif

      // Partition the examples using the decision function.
      for(size_t j = 0, size = examples.size(); j < size; ++j)
      {
        if(splitCandidate->m_decisionFunction->classify_descriptor(*examples[j]->get_descriptor()) == DecisionFunction::DC_LEFT)
        {
          splitCandidate->m_leftExamples.push_back(examples[j]);
        }
        else
        {
          splitCandidate->m_rightExamples.push_back(examples[j]);
        }
      }

      // Calculate the information gain we would obtain from this split.
      float gain = calculate_information_gain(reservoir, initialEntropy, splitCandidate->m_leftExamples, splitCandidate->m_rightExamples);
      if(gain < FLT_MIN || gain < gainThreshold) splitCandidate.reset();

      // Add the result to the gain -> candidate map so as to allow us to find a split with maximum gain.
      gainToCandidateMap.insert(std::make_pair(gain, splitCandidate));
    }

    // Return a split candidate that had maximum gain (note that this may be NULL if no split had a high enough gain).
    return gainToCandidateMap.begin()->second;
  }

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the information gain that results from splitting an example reservoir in a particular way.
   *
   * \param reservoir       The reservoir.
   * \param initialEntropy  The entropy of the example set before the split.
   * \param leftExamples    The examples that end up in the left half of the split.
   * \param rightExamples   The examples that end up in the right half of the split.
   * \return                The information gain resulting from the split.
   */
  static float calculate_information_gain(const ExampleReservoir<Label>& reservoir, float initialEntropy, const std::vector<Example_CPtr>& leftExamples, const std::vector<Example_CPtr>& rightExamples)
  {
    float exampleCount = static_cast<float>(reservoir.current_size());
    float leftEntropy = ExampleUtil::calculate_entropy<Label>(leftExamples, reservoir.get_class_multipliers());
    float rightEntropy = ExampleUtil::calculate_entropy<Label>(rightExamples, reservoir.get_class_multipliers());
    float leftWeight = leftExamples.size() / exampleCount;
    float rightWeight = rightExamples.size() / exampleCount;

#if 0
    std::cout << "L: " << ExampleUtil::make_histogram(leftExamples) << ' ' << leftEntropy << '\n';
    std::cout << "R: " << ExampleUtil::make_histogram(rightExamples) << ' ' << rightEntropy << '\n';
#endif

    float gain = initialEntropy - (leftWeight * leftEntropy + rightWeight * rightEntropy);

#if 0
    std::cout << "Gain: " << gain << '\n';
#endif

    return gain;
  }
};

}

#endif
