/**
 * rafl: DecisionFunction.h
 */

#ifndef H_RAFL_DECISIONFUNCTION
#define H_RAFL_DECISIONFUNCTION

#include <iosfwd>

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
   * \brief Outputs a string representation of the decision function to the specified stream.
   *
   * \param os  The stream.
   */
  virtual void output(std::ostream& os) const = 0;
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a string representation of the specified decision function to a stream.
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
