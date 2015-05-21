/**
 * rafl: FeatureThresholdingDecisionFunctionGenerator.h
 */

#ifndef H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTIONGENERATOR
#define H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTIONGENERATOR

#include <cassert>

#include <tvgutil/RandomNumberGenerator.h>

#include "FeatureBasedDecisionFunctionGenerator.h"
#include "FeatureThresholdingDecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to generate a feature thresholding decision function
 *        with which split a set of examples.
 */
template <typename Label>
class FeatureThresholdingDecisionFunctionGenerator : public FeatureBasedDecisionFunctionGenerator<Label>
{
  //#################### USINGS ####################
protected:
  using typename DecisionFunctionGenerator<Label>::Example_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a decision function generator that can randomly generate feature thresholding decision functions.
   *
   * \param featureIndexRange An optional range of indices specifying the features that should be considered when generating decision functions.
   */
  FeatureThresholdingDecisionFunctionGenerator(const boost::optional<std::pair<int,int> >& featureIndexRange = boost::none)
  : FeatureBasedDecisionFunctionGenerator(featureIndexRange)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual DecisionFunction_Ptr generate_candidate_decision_function(const std::vector<Example_CPtr>& examples, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator) const
  {
    assert(!examples.empty());

    int descriptorSize = static_cast<int>(examples[0]->get_descriptor()->size());

    // Pick a random feature in the descriptor to threshold.
    std::pair<int,int> featureIndexRange = get_feature_index_range(descriptorSize);
    int featureIndex = randomNumberGenerator->generate_int_from_uniform(featureIndexRange.first, featureIndexRange.second);

    // Select an appropriate threshold by picking a random example and using
    // the value of the chosen feature from that example as the threshold.
    int exampleIndex = randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(examples.size()) - 1);
    float threshold = (*examples[exampleIndex]->get_descriptor())[featureIndex];

    return DecisionFunction_Ptr(new FeatureThresholdingDecisionFunction(featureIndex, threshold));
  }

  /**
   * \brief Gets the type of the decision function generator.
   *
   * \return  The type of the decision function generator.
   */
  static std::string get_static_type()
  {
    return "FeatureThresholding";
  }

  /** Override */
  virtual std::string get_type() const
  {
    return get_static_type();
  }
};

}

#endif
