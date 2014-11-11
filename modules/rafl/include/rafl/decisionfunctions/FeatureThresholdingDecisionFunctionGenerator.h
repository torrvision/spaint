/**
 * rafl: FeatureThresholdingDecisionFunctionGenerator.h
 */

#ifndef H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTIONGENERATOR
#define H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTIONGENERATOR

#include "DecisionFunctionGenerator.h"
#include "FeatureThresholdingDecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template can be used to pick an appropriate decision function to split a set of examples.
 */
template <typename Label>
class FeatureThresholdingDecisionFunctionGenerator : public DecisionFunctionGenerator<Label>
{
  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /** Override */
  virtual DecisionFunction_Ptr generate_candidate(const std::vector<Example_CPtr>& examples) const
  {
    // TODO
    return DecisionFunction_Ptr();
  }
};

}

#endif
