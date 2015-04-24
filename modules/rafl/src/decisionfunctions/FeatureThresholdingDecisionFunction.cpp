/**
 * rafl: FeatureThresholdingDecisionFunction.cpp
 */

#include "decisionfunctions/FeatureThresholdingDecisionFunction.h"

#include <iostream>

// Note: It is CRUCIALLY IMPORTANT that the archive headers are included at the point where we invoke BOOST_CLASS_EXPORT, or the export won't work.
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/export.hpp>

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
