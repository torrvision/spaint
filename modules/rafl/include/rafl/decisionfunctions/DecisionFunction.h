/**
 * rafl: DecisionFunction.h
 */

#ifndef H_RAFL_DECISIONFUNCTION
#define H_RAFL_DECISIONFUNCTION

#include <iosfwd>

#include <boost/serialization/serialization.hpp>

#include "../base/Descriptor.h"

namespace rafl {

/**
 * \brief An instance of a class derived from this one represents a decision function for a random forest node
 *        that can be used to classify descriptors into either the left or right subtree of the node.
 */
class DecisionFunction
{
  //#################### ENUMERATIONS ####################
public:
  /**
   * \brief An enumeration containing the possible classifications that the decision function can output.
   */
  enum DescriptorClassification
  {
    DC_LEFT,
    DC_RIGHT
  };

  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the decision function.
   */
  virtual ~DecisionFunction() {}

  //#################### PUBLIC ABSTRACT MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Classifies the specified descriptor using the decision function.
   *
   * \param descriptor  The descriptor to classify.
   * \return            DC_LEFT, if the descriptor should be sent down the left subtree of the node, or DC_RIGHT otherwise.
   */
  virtual DescriptorClassification classify_descriptor(const Descriptor& descriptor) const = 0;

  /**
   * \brief Outputs the decision function to the specified stream.
   *
   * \param os  The stream.
   */
  virtual void output(std::ostream& os) const = 0;

  //#################### SERIALIZATION #################### 
private:
  /**
   * \brief Serializes the decision function to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    // No-op
  }

  friend class boost::serialization::access;
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs the specified decision function to a stream.
 *
 * \param os  The stream.
 * \param rhs The decision function.
 * \return    The stream.
 */
inline std::ostream& operator<<(std::ostream& os, const DecisionFunction& rhs)
{
  rhs.output(os);
  return os;
}

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<DecisionFunction> DecisionFunction_Ptr;

}

#endif
