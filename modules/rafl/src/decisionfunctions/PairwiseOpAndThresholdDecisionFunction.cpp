/**
 * rafl: PairwiseOpAndThresholdDecisionFunction.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
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
  float result = apply_op(m_op, descriptor[m_firstFeatureIndex], descriptor[m_secondFeatureIndex]);
  return result < m_threshold ? DC_LEFT : DC_RIGHT;
}

void PairwiseOpAndThresholdDecisionFunction::output(std::ostream& os) const
{
  os << "First Feature " << m_firstFeatureIndex << ' '
     << to_string(m_op)
     << " Second Feature " << m_secondFeatureIndex
     << " < " << m_threshold;
}

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

float PairwiseOpAndThresholdDecisionFunction::apply_op(Op op, float a, float b)
{
  switch(op)
  {
    case PO_ADD:
      return a + b;
    case PO_SUBTRACT:
      return a - b;
    default:
      // This should never happen.
      throw std::runtime_error("Unknown pairwise operation");
  }
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
