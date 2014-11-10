/**
 * rafl: DecisionFunction.cpp
 */

#include "base/DecisionFunction.h"

namespace rafl {

//#################### CONSTRUCTORS ####################

DecisionFunction::DecisionFunction(size_t featureIndex, float threshold)
: m_featureIndex(featureIndex), m_threshold(threshold)
{}

//#################### PUBLIC OPERATORS ####################

bool DecisionFunction::operator()(const Descriptor& descriptor) const
{
  return descriptor[m_featureIndex] < m_threshold;
}

}
