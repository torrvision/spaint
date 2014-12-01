/**
 * rafl: Example.h
 */

#ifndef H_RAFL_EXAMPLE
#define H_RAFL_EXAMPLE

#include <ostream>

#include <tvgutil/LimitedContainer.h>

#include "../base/Descriptor.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a training example for a random forest.
 */
template <typename Label>
class Example
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The feature descriptor for the example. */
  Descriptor_CPtr m_descriptor;

  /** The label for the example. */
  Label m_label;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an example.
   *
   * \param descriptor  The feature descriptor for the example.
   * \param label       The label for the example.
   */
  Example(const Descriptor_CPtr& descriptor, const Label& label)
  : m_descriptor(descriptor), m_label(label)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Gets the feature descriptor for the example.
   *
   * \return  The feature descriptor for the example.
   */
  const Descriptor_CPtr& get_descriptor() const
  {
    return m_descriptor;
  }

  /**
   * \brief Gets the label for the example.
   *
   * \return  The label for the example.
   */
  const Label& get_label() const
  {
    return m_label;
  }
};

//#################### STREAM OPERATORS ####################

template <typename Label>
std::ostream& operator<<(std::ostream& os, const Example<Label>& rhs)
{
  os << tvgutil::make_limited_container(*rhs.get_descriptor(), 3) << ' ' << rhs.get_label();
  return os;
}

}

#endif
