/**
 * rafl: PairwiseOperationAndThresholdingFunctionGenerator.h
 */

#ifndef H_RAFL_PAIRWISEOPERATIONANSTHRESHOLDINGFUNCTIONGENERATOR
#define H_RAFL_PAIRWISEOPERATIONANSTHRESHOLDINGFUNCTIONGENERATOR

#include <cassert>

#include <tvgutil/RandomNumberGenerator.h>

#include "DecisionFunctionGenerator.h"
#include "PairwiseOperationAndThresholdingFunctionGenerator.h"

namespace rafl {

/**
 * \brief TODO.
 */
template <typename Label>
class PairwiseOperationAndThresholdingFunctionGenerator : public DecisionFunctionGenerator<Label>
{
  //#################### USINGS ####################
protected:
  using typename DecisionFunctionGenerator<Label>::Example_CPtr;

  //#################### PRIVATE VARIABLES #################### 
private:
  /** A random number generator. */
  tvgutil::RandomNumberGenerator_Ptr m_randomNumberGenerator;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief TODO.
   */
  explicit PairwiseOperationAndThresholdingFunctionGenerator(const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator)
  : m_randomNumberGenerator(randomNumberGenerator)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /**
   * \brief Gets the type of the decision function generator.
   *
   * \return The type of the decision function generator.
   */
  static std::string get_static_type()
  {
    return "PairwiseOperationAndThresholding";
  }

  /** Override */
  virtual std::string get_type() const
  {
    return get_static_type();
  }

  //#################### PRIVATE MEMBER FUNCTIONS #################### 
private:
  /** Override */
  virtual DecisionFunction_Ptr generate_candidate_decision_function(const std::vector<Example_CPtr>& examples) const
  {
    assert(!examples.empty());

    int descriptorSize = static_cast<int>(examples[0]->get_descriptor()->size());

    //Pick the first random feature in the descriptor.
    int firstFeatureIndex = m_randomNumberGenerator->generate_int_from_uniform(0, descriptorSize - 1);

    //Pick the second random feature int he descriptor.
    int secondFeatureIndex = m_randomNumberGenerator->generate_int_from_uniform(0, descriptorSize - 1);

    //Pick the pairwise operation (0 = subtraction, 1 = addition).
    bool pairwiseOperation = m_randomNumberGenerator->generate_int_from_uniform(0, 1);

    // Select an appropriate threshold by picking a random example and using
    // the value of the chosen feature form that example as the threshold.
    int exampleIndex = m_randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(examples.size()) - 1);
    float threshold = (*examples[exampleIndex]->get_descriptor())[featureIndex];

    return DecisionFunction_Ptr(new PairwiseOperationAndThresholdingFunctionGenerator(firstFeatureIndex, secondFeatureIndex, pairwiseOperation == 0 ? OP_SUBTRACT : OP_ADD, threshold));
  }
};

}

#endif
