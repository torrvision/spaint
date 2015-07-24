/**
 * rafl: FeatureThresholdingDecisionFunctionGenerator.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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
  //#################### TYPEDEFS AND USINGS ####################
protected:
  typedef boost::shared_ptr<DecisionFunctionGenerator<Label> > DecisionFunctionGenerator_Ptr;
  using typename DecisionFunctionGenerator<Label>::Example_CPtr;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a decision function generator that can randomly generate feature thresholding decision functions.
   *
   * \param featureIndexRange An optional range of indices specifying the features that should be considered when generating decision functions.
   */
  FeatureThresholdingDecisionFunctionGenerator(const boost::optional<std::pair<int,int> >& featureIndexRange = boost::none)
  : FeatureBasedDecisionFunctionGenerator<Label>(featureIndexRange)
  {}

  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the type of the decision function generator.
   *
   * \return  The type of the decision function generator.
   */
  static std::string get_static_type()
  {
    return "FeatureThresholding";
  }

  /**
   * \brief Makes a feature thresholding decision function generator.
   *
   * \param params  The parameters to the decision function generator.
   * \return        The decision function generator.
   */
  static DecisionFunctionGenerator_Ptr maker(const std::string& params)
  {
    boost::optional<std::pair<int,int> > parsedParams = FeatureBasedDecisionFunctionGenerator<Label>::parse_params(params);
    return DecisionFunctionGenerator_Ptr(new FeatureThresholdingDecisionFunctionGenerator<Label>(parsedParams));
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual DecisionFunction_Ptr generate_candidate_decision_function(const std::vector<Example_CPtr>& examples, const tvgutil::RandomNumberGenerator_Ptr& randomNumberGenerator) const
  {
    assert(!examples.empty());

    int descriptorSize = static_cast<int>(examples[0]->get_descriptor()->size());

    // Pick a random feature in the descriptor to threshold.
    std::pair<int,int> featureIndexRange = this->get_feature_index_range(descriptorSize);
    int featureIndex = randomNumberGenerator->generate_int_from_uniform(featureIndexRange.first, featureIndexRange.second);

    // Select an appropriate threshold by picking a random example and using
    // the value of the chosen feature from that example as the threshold.
    int exampleIndex = randomNumberGenerator->generate_int_from_uniform(0, static_cast<int>(examples.size()) - 1);
    float threshold = (*examples[exampleIndex]->get_descriptor())[featureIndex];

    return DecisionFunction_Ptr(new FeatureThresholdingDecisionFunction(featureIndex, threshold));
  }

  /** Override */
  virtual std::string get_type() const
  {
    return get_static_type();
  }
};

}

#endif
