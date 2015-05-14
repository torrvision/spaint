/**
 * rafl: FeatureComparisonAndThresholdingDecisionFunction.h
 */

#ifndef H_RAFL_FEATURECOMPARISONANDTHRESHOLDINGDECISIONFUNCTION
#define H_RAFL_FEATURECOMPARISONANDTHRESHOLDINGDECISIONFUNCTION

#include <boost/serialization/base_object.hpp>

#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief TODO.
 */
class FeatureComparisonAndThresholdingDecisionFunction : public DecisionFunction
{
  //#################### PRIVATE VARIABLES #################### 
private:
  /** The index of the first feature in a feature descriptor. */
  size_t m_firstFeatureIndex;

  /** The pairwise operation (false = addition), (true = subtraction) to apply to the features. */
  bool m_pairwiseOperation;

  /** The index of the second feature in a feature descriptor. */
  size_t m_secondFeatureIndex;

  /** The threshold against which to compare the result of the operation. */
  float m_threshold;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief TODO.
   */
  FeatureComparisonAndThresholdingDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, bool pairwiseOperation, float threshold)
  {
  }

private:
  /**
   * \brief TODO.
   */
  FeatureComparisonAndThresholdingDecisionFunction() {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual DescriptorClassification classify_descriptor(const Descriptor& desctiptor) const;

  /** Override */
  virtual void output(std::ostream& os) const;

  //#################### SERIALIZATION #################### 
private:
  /**
   * TODO.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<DecisionFunction>(*this);
    ar & m_firstFeatureIndex;
    ar & m_pairwiseOperation;
    ar & m_secondFeatureIndex;
    ar & m_threshold;
  }

  friend class boost::serialization::access;
};

}

#endif
