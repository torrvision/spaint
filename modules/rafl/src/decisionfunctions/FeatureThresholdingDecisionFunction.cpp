/**
 * rafl: FeatureThresholdingDecisionFunction.cpp
 */

#include "decisionfunctions/FeatureThresholdingDecisionFunction.h"

namespace rafl {

//#################### CONSTRUCTORS ####################

FeatureThresholdingDecisionFunction::FeatureThresholdingDecisionFunction(size_t featureIndex, float threshold)
: m_featureIndex(featureIndex), m_threshold(threshold)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

DecisionFunction::DescriptorClassification FeatureThresholdingDecisionFunction::classify_descriptor(const Descriptor& descriptor) const
{
  return descriptor[m_featureIndex] < m_threshold ? DC_LEFT : DC_RIGHT;
}

}
