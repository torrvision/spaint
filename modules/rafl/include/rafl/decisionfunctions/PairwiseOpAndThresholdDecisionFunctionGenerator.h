/**
 * rafl: PairwiseOpAndThresholdDecisionFunctionGenerator.h
 */

#ifndef H_RAFL_PAIRWISEOPANDTHRESHOLDDECISIONFUNCTIONGENERATOR
#define H_RAFL_PAIRWISEOPANDTHRESHOLDDECISIONFUNCTIONGENERATOR

#include <cassert>

#include <tvgutil/RandomNumberGenerator.h>

#include "DecisionFunctionGenerator.h"
#include "PairwiseOpAndThresholdDecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to generate a pairwise operation and thresholding decision function
 *        with which to split a set of examples.
 */
template <typename Label>
class PairwiseOpAndThresholdDecisionFunctionGenerator : public DecisionFunctionGenerator<Label>
{
  //#################### TYPEDEFS AND USINGS ####################
private:
  using typename DecisionFunctionGenerator<Label>::Example_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a decision function generator that can randomly generate pairwise operation and thresholding decision functions.
   */
  PairwiseOpAndThresholdDecisionFunctionGenerator() {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the type of the decision function generator.
   *
   * \return The type of the decision function generator.
   */
  static std::string get_static_type()
  {
    return "PairwiseOpAndThreshold";
  }

  /** Override */
  virtual std::string get_type() const
  {
    return get_static_type();
  }

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual DecisionFunction_Ptr generate_candidate_decision_function(const std::vector<Example_CPtr>& examples, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator) const
  {
    assert(!examples.empty());

    int descriptorSize = static_cast<int>(examples[0]->get_descriptor()->size());

    // Pick the first random feature in the descriptor.
    int firstFeatureIndex = randomNumberGenerator->generate_int_from_uniform(0, descriptorSize - 1);

    // Pick the second random feature in the descriptor.
    int secondFeatureIndex = randomNumberGenerator->generate_int_from_uniform(0, descriptorSize - 1);

    // Pick the pairwise operation.
    int opIndex = randomNumberGenerator->generate_int_from_uniform(0, PairwiseOpAndThresholdDecisionFunction::PO_COUNT - 1);
    PairwiseOpAndThresholdDecisionFunction::Op op = static_cast<PairwiseOpAndThresholdDecisionFunction::Op>(opIndex);

    // Select an appropriate threshold by picking a random example and using
    // the result of applying the pairwise operation to the chosen features
    // from that example as the threshold.
    int exampleIndex = randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(examples.size()) - 1);
    const Descriptor& descriptor = (*examples[exampleIndex]->get_descriptor());
    float threshold = PairwiseOpAndThresholdDecisionFunction::apply_op(op, descriptor[firstFeatureIndex], descriptor[secondFeatureIndex]);

    return DecisionFunction_Ptr(new PairwiseOpAndThresholdDecisionFunction(
      firstFeatureIndex,
      secondFeatureIndex,
      op,
      threshold
    ));
  }
};

}

#endif
