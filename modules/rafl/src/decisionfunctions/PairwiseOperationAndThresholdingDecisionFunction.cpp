/**
 * rafl: PairwiseOperationAndThresholdingDecisionFunction.cpp
 */

#include "decisionfunctions/PairwiseOperationAndThresholdingDecisionFunction.h"

#include <iostream>

// Note: It is CRUCIALLY IMPORTANT that the archive headers are included before the point at which we invoke BOOST_CLASS_EXPORT, or the export won't work.
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/export.hpp>

namespace rafl {

//#################### CONSTRUCTORS #################### 

PairwiseOperationAndThresholdingDecisionFunction::PairwiseOperationAndThresholdingDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, PairwiseOperation pairwiseOperation, float threshold)
: m_firstFeatureIndex(firstFeatureIndex), m_pairwiseOperation(pairwiseOperation), m_secondFeatureIndex(secondFeatureIndex), m_threshold(threshold)
{}

//#################### PUBLIC MEMBER FUNCTIONS #################### 

DecisionFunction::DescriptorClassification PairwiseOperationAndThresholdingDecisionFunction::classify_descriptor(const Descriptor& descriptor) const
{
  float pairwiseFeature;
  switch (m_pairwiseOperation)
  {
    case PO_ADD:
      pairwiseFeature = descriptor[m_firstFeatureIndex] + descriptor[m_secondFeatureIndex];
      break;
    case PO_SUBTRACT:
      pairwiseFeature = descriptor[m_firstFeatureIndex] - descriptor[m_secondFeatureIndex];
      break;
    default:
      throw std::runtime_error("The pairwise operation you selectted is not recognised");
      break;
  }

  return pairwiseFeature < m_threshold ? DC_LEFT : DC_RIGHT;
}

std::string PairwiseOperationAndThresholdingDecisionFunction::get_name(const PairwiseOperation& pairwiseOperation) const
{
  std::string pairwiseOperationName;
  switch (pairwiseOperation)
  {
    case PO_ADD:
      pairwiseOperationName = "+";
      break;
    case PO_SUBTRACT:
      pairwiseOperationName = "-";
      break;
    default:
      throw std::runtime_error("The pairwise operation you selected is not recognised");
      break;
  }
  return pairwiseOperationName;
}

void PairwiseOperationAndThresholdingDecisionFunction::output(std::ostream& os) const
{
  os << "First Feature " << m_firstFeatureIndex << " " << get_name(m_pairwiseOperation) << " Second Feature " << m_secondFeatureIndex << " < " << m_threshold;
}

}

BOOST_CLASS_EXPORT(rafl::PairwiseOperationAndThresholdingDecisionFunction)
