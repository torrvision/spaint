/**
 * rafl: FeatureThresholdingDecisionFunction.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTION
#define H_RAFL_FEATURETHRESHOLDINGDECISIONFUNCTION

#include <boost/serialization/base_object.hpp>

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
  /**
   * \brief Constructs a feature thresholding decision function.
   *
   * \param featureIndex  The index of the feature in a feature descriptor that should be compared to the threshold.
   * \param threshold     The threshold against which to compare it.
   */
  FeatureThresholdingDecisionFunction(size_t featureIndex, float threshold);

private:
  /**
   * \brief Constructs a feature thresholding decision function.
   *
   * Note: This constructor is needed for serialization and should not be used otherwise.
   */
  FeatureThresholdingDecisionFunction() {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual DescriptorClassification classify_descriptor(const Descriptor& descriptor) const;

  /** Override */
  virtual void output(std::ostream& os) const;

  //#################### SERIALIZATION #################### 
private:
  /**
   * \brief Serializes the decision function to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<DecisionFunction>(*this);
    ar & m_featureIndex;
    ar & m_threshold;
  }

  friend class boost::serialization::access;
};

}

#endif
