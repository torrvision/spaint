/**
 * rafl: FeatureThresholdingDecisionFunction.h
 */

#ifndef H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTION
#define H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTION

#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of this class represents a decision function that tests an individual feature against a threshold.
 */
class FeatureThresholdingDecisionFunction : public DecisionFunction
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the feature in a feature descriptor that should be compared to the threshold. */
  size_t m_featureIndex;

  /** The threshold against which to compare it. */
  float m_threshold;

  //#################### CONSTRUCTORS ####################
public:
  FeatureThresholdingDecisionFunction(){};

  /**
   * \brief Constructs a feature thresholding decision function.
   *
   * \param featureIndex  The index of the feature in a feature descriptor that should be compared to the threshold.
   * \param threshold     The threshold against which to compare it.
   */
  FeatureThresholdingDecisionFunction(size_t featureIndex, float threshold);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual DescriptorClassification classify_descriptor(const Descriptor& descriptor) const;

  /** Override */
  virtual void output(std::ostream& os) const;

  //#################### SERIALIZATION #################### 
private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<DecisionFunction>(*this);
    ar & m_featureIndex;
    ar & m_threshold;
  }
};

}

#endif
