/**
 * rafl: PairwiseOperationAndThresholdingDecisionFunction.h
 */

#ifndef H_RAFL_PAIRWISEOPERATIONANDTHRESHOLDINGDECISIONFUNCTION
#define H_RAFL_PAIRWISEOPERATIONANDTHRESHOLDINGDECISIONFUNCTION

#include <boost/serialization/base_object.hpp>

#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief TODO.
 */
class PairwiseOperationAndThresholdingDecisionFunction : public DecisionFunction
{
  //#################### PUBLIC ENUMS #################### 
public:
  /**
   * \brief TODO.
   */
  enum PairwiseOperation {
    PO_ADD,
    PO_SUBTRACT
  };

  //#################### PRIVATE VARIABLES #################### 
private:
  /** The index of the first feature in a feature descriptor. */
  size_t m_firstFeatureIndex;

  /** The pairwise operation to apply to the features. */
  PairwiseOperation m_pairwiseOperation;

  /** The index of the second feature in a feature descriptor. */
  size_t m_secondFeatureIndex;

  /** The threshold against which to compare the result of the operation. */
  float m_threshold;

  //#################### CONSTRUCTORS #################### 
public:
  /**
   * \brief TODO.
   */
  PairwiseOperationAndThresholdingDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, PairwiseOperation pairwiseOperation, float threshold);

private:
  /**
   * \brief TODO.
   */
  PairwiseOperationAndThresholdingDecisionFunction() {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual DescriptorClassification classify_descriptor(const Descriptor& desctiptor) const;

  /**
   * \brief TODO>
   */
  std::string get_name(const PairwiseOperation& pairwiseOperation) const;

  /** Override */
  virtual void output(std::ostream& os) const;

  //#################### SERIALIZATION #################### 
private:
  /**
   * TODO.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & boost::serialization::base_object<DecisionFunction>(*this);
    ar & m_firstFeatureIndex;
    ar & m_pairwiseOperation;
    ar & m_secondFeatureIndex;
    ar & m_threshold;
  }

  friend class boost::serialization::access;
};

}

#endif
