/**
 * rafl: PairwiseOpAndThresholdDecisionFunction.h
 */

#ifndef H_RAFL_PAIRWISEOPANDTHRESHOLDDECISIONFUNCTION
#define H_RAFL_PAIRWISEOPANDTHRESHOLDDECISIONFUNCTION

#include <boost/serialization/base_object.hpp>

#include "DecisionFunction.h"

namespace rafl {

/**
 * \brief An instance of this class represents a decision function that combines two features
 *        using a pairwise operation and tests the result against a threshold.
 */
class PairwiseOpAndThresholdDecisionFunction : public DecisionFunction
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration specifying the different types of pairwise operations that are supported.
   */
  enum Op
  {
    /** The two features should be added together. */
    PO_ADD,

    /** The second feature should be subtracted from the first feature. */
    PO_SUBTRACT,

    /** A dummy value denoting the number of possible pairwise operations. */
    PO_COUNT
  };

  //#################### PRIVATE VARIABLES ####################
private:
  /** The index of the first feature in a feature descriptor. */
  size_t m_firstFeatureIndex;

  /** The pairwise operation to apply to the features. */
  Op m_op;

  /** The index of the second feature in a feature descriptor. */
  size_t m_secondFeatureIndex;

  /** The threshold against which to compare the result of the operation. */
  float m_threshold;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a pairwise operation and thresholding decision function.
   *
   * \param firstFeatureIndex   The index of the first feature in a feature descriptor.
   * \param secondFeatureIndex  The index of the second feature in a feature descriptor.
   * \param op                  The pairwise operation.
   * \param threshold           The threshold against which to compare the result of the operation.
   */
  PairwiseOpAndThresholdDecisionFunction(size_t firstFeatureIndex, size_t secondFeatureIndex, Op op, float threshold);

private:
  /**
   * \brief Constructs a pairwise operation and thresholding decision function.
   *
   * Note: This constructor is needed for serialization and should not be used otherwise.
   */
  PairwiseOpAndThresholdDecisionFunction();

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual DescriptorClassification classify_descriptor(const Descriptor& descriptor) const;

  /** Override */
  virtual void output(std::ostream& os) const;

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets a string representing the specified pairwise operation.
   *
   * \param op  A pairwise operation.
   * \return    A string representing the pairwise operation.
   */
  static std::string to_string(Op op);

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
    ar & m_op;
    ar & m_secondFeatureIndex;
    ar & m_threshold;
  }

  friend class boost::serialization::access;
};

}

#endif
