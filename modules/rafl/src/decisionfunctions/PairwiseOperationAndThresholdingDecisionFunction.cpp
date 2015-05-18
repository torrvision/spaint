/**
 * rafl: PairwiseOperationAndThresholdingDecisionFunction.cpp
 */

#include "decisionfunctions/PairwiseOperationAndThresholdingDecisionFunction.h"

#include <iostream>

namespace rafl {

//#################### CONSTRUCTORS #################### 

PairwiseOperationAndThresholdingDecisionFunction::PairwiseOperationAndThresholdingDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex,
                                                                                                   PairwiseOperation pairwiseOperation, float threshold)
: m_firstFeatureIndex(firstFeatureIndex),
  m_pairwiseOperation(pairwiseOperation),
  m_secondFeatureIndex(secondFeatureIndex),
  m_threshold(threshold)
{}

PairwiseOperationAndThresholdingDecisionFunction::PairwiseOperationAndThresholdingDecisionFunction()
{}

//#################### PUBLIC MEMBER FUNCTIONS #################### 

DecisionFunction::DescriptorClassification PairwiseOperationAndThresholdingDecisionFunction::classify_descriptor(const Descriptor& descriptor) const
{
  const float feature1 = descriptor[m_firstFeatureIndex];
  const float feature2 = descriptor[m_secondFeatureIndex];

  float pairwiseFeature;
  switch(m_pairwiseOperation)
  {
    case PO_ADD:
      pairwiseFeature = feature1 + feature2;
      break;
    case PO_SUBTRACT:
      pairwiseFeature = feature1 - feature2;
      break;
    default:
      // This should never happen.
      throw std::runtime_error("Unknown pairwise operation");
  }

  return pairwiseFeature < m_threshold ? DC_LEFT : DC_RIGHT;
}

void PairwiseOperationAndThresholdingDecisionFunction::output(std::ostream& os) const
{
  os << "First Feature " << m_firstFeatureIndex << ' '
     << to_string(m_pairwiseOperation)
     << " Second Feature " << m_secondFeatureIndex
     << " < " << m_threshold;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

std::string PairwiseOperationAndThresholdingDecisionFunction::to_string(PairwiseOperation op)
{
  switch(op)
  {
    case PO_ADD:
      return "+";
    case PO_SUBTRACT:
      return "-";
    default:
      // This should never happen.
      throw std::runtime_error("Unknown pairwise operation");
  }
}

}

BOOST_CLASS_EXPORT(rafl::PairwiseOperationAndThresholdingDecisionFunction)
