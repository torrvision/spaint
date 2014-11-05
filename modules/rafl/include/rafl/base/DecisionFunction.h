/**
 * rafl: DecisionFunction.h
 */

#ifndef H_RAFL_DECISIONFUNCTION
#define H_RAFL_DECISIONFUNCTION

#include "Descriptor.h"

namespace rafl {

/**
 * \brief An instance of this class represents the decision function of a node in a random forest.
 *
 * Decision functions (as currently implemented) test whether an individual feature is less than a threshold.
 */
class DecisionFunction
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
   * \brief Constructs a decision function.
   *
   * \param featureIndex  The index of the feature in the feature descriptor that should be compared to the threshold.
   * \param threshold     The threshold against which to compare it.
   */
  DecisionFunction(size_t featureIndex, float threshold)
  : m_featureIndex(featureIndex), m_threshold(threshold)
  {}

  //#################### PUBLIC OPERATORS ####################
public:
  /**
   * \brief Evaluates the decision function for the specified feature descriptor.
   *
   * \param descriptor  The feature descriptor for which to evaluate the decision function.
   * \return            true, if the feature being tested is less than the threshold, or false otherwise.
   */
  bool operator()(const Descriptor& descriptor) const
  {
    return descriptor[m_featureIndex] < m_threshold;
  }
};

//#################### TYPEDEFS ####################

typedef tvgutil::shared_ptr<DecisionFunction> DecisionFunction_Ptr;

}

#endif
