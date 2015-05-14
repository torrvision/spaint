/**
 * rafl: PairwiseComparisonAndThresholdingDecisionFunction.cpp
 */

#include "decisionfunctions/PairwiseComparisonAndThresholdingDecisionFunction.h"

#include <iostream>

// Note: It is CRUCIALLY IMPORTANT that the archive headers are included before the point at which we invoke BOOST_CLASS_EXPORT, or the export won't work.
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/export.hpp>

namespace rafl {

//#################### CONSTRUCTORS #################### 

PairwiseComparisonAndThresholdingDecisionFunction::PairwiseComparisonAndThresholdingDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, bool pairwiseOperation, float threshold)
: m_firstFeatureIndex(firstFeatureIndex), m_secondFeatureIndex(secondFeatureIndex), m_pairwiseOperation(pairwiseOperation), m_threshold(threshold)
{}

//#################### PUBLIC MEMBER FUNCTIONS #################### 

DecisionFunction::DescriptorClassification PairwiseComparisonAndThresholdingDecisionFunction::classify_descriptor(const Descriptor& descriptor) const
{
  //FIXME Implement this.
  return true;
}

void PairwiseComparisonAndThresholdingDecisionFunction::output(std::ostream& os) const
{
}
