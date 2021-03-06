/**
 * rafl: FeatureThresholdingDecisionFunction.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "decisionfunctions/FeatureThresholdingDecisionFunction.h"

#include <iostream>

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

void FeatureThresholdingDecisionFunction::output(std::ostream& os) const
{
  os << "Feature " << m_featureIndex << " < " << m_threshold;
}

}

BOOST_CLASS_EXPORT(rafl::FeatureThresholdingDecisionFunction)
