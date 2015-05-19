/**
 * rafl: PairwiseOpAndThresholdDecisionFunction.cpp
 */

#include "decisionfunctions/PairwiseOpAndThresholdDecisionFunction.h"

#include <iostream>

namespace rafl {

//#################### CONSTRUCTORS #################### 

PairwiseOpAndThresholdDecisionFunction::PairwiseOpAndThresholdDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, Op op, float threshold)
: m_firstFeatureIndex(firstFeatureIndex),
  m_op(op),
  m_secondFeatureIndex(secondFeatureIndex),
  m_threshold(threshold)
{}

PairwiseOpAndThresholdDecisionFunction::PairwiseOpAndThresholdDecisionFunction()
{}

//#################### PUBLIC MEMBER FUNCTIONS #################### 

DecisionFunction::DescriptorClassification PairwiseOpAndThresholdDecisionFunction::classify_descriptor(const Descriptor& descriptor) const
{
  const float feature1 = descriptor[m_firstFeatureIndex];
  const float feature2 = descriptor[m_secondFeatureIndex];

  float pairwiseFeature;
  switch(m_op)
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

void PairwiseOpAndThresholdDecisionFunction::output(std::ostream& os) const
{
  os << "First Feature " << m_firstFeatureIndex << ' '
     << to_string(m_op)
     << " Second Feature " << m_secondFeatureIndex
     << " < " << m_threshold;
}

//#################### PRIVATE STATIC MEMBER FUNCTIONS ####################

std::string PairwiseOpAndThresholdDecisionFunction::to_string(Op op)
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

BOOST_CLASS_EXPORT(rafl::PairwiseOpAndThresholdDecisionFunction)
