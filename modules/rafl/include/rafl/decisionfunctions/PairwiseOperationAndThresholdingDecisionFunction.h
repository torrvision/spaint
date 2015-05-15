/**
 * rafl: PairwiseOperationAndThresholdingDecisionFunction.h
 */

#ifndef H_RAFL_PAIRWISEOPERATIONANDTHRESHOLDINGDECISIONFUNCTION
#define H_RAFL_PAIRWISEOPERATIONANDTHRESHOLDINGDECISIONFUNCTION

#include <boost/serialization/base_object.hpp>

#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of this class represents a decision function that combines two features with an operation and tests the result against a threshold.
 */
class PairwiseOperationAndThresholdingDecisionFunction : public DecisionFunction
{
  //#################### PUBLIC ENUMS #################### 
public:
  /**
   * \brief An enumeration specifying the different types of pairwise operations that are supported.
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
   * \brief Constructs a pairwise operation and thresholding decision function.
   *
   * \param firstFeatureIndex   The index of the first feature in a descriptor.
   * \param secondFeatureIndex  The index of the second feature in a descriptor.
   * \param pairwiseOperation   The specified pairwise operation.
   * \param threshold           The threshold against which to compare the result of combining the two features with the specified pairwise operation.
   */
  PairwiseOperationAndThresholdingDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, PairwiseOperation pairwiseOperation, float threshold);

private:
  /**
   * \brief Constructs a pairwise operation and thresholding decision function.
   *
   * Note: This constructor is needed for serialization and should not be used otherwise.
   */
  PairwiseOperationAndThresholdingDecisionFunction() {}

  //#################### PUBLIC MEMBER FUNCTIONS #################### 
public:
  /** Override */
  virtual DescriptorClassification classify_descriptor(const Descriptor& desctiptor) const;

  /**
   * \brief Get a string associated with each pairwise operation.
   *
   * \param pairwiseOperation  The specified pairwise operation.
   * \return                   The string representing the pairwise operation.
   */
  std::string get_name(const PairwiseOperation& pairwiseOperation) const;

  /** Override */
  virtual void output(std::ostream& os) const;

  //#################### SERIALIZATION #################### 
private:
  /**
   * \brief Serializes the decision function to/from an archive.
   *
   * \param ar       The archive.
   * \param version  The file format version number.
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
