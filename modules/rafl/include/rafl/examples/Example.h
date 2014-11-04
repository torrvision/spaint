/**
 * rafl: Example.h
 */

#ifndef H_RAFL_EXAMPLE
#define H_RAFL_EXAMPLE

#include "../base/Descriptor.h"

namespace rafl {

/**
 * \brief TODO
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
   * \brief TODO
   */
  Example(const Descriptor_CPtr& descriptor, const Label& label)
  : m_descriptor(descriptor), m_label(label)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  const Descriptor_CPtr& get_descriptor() const
  {
    return m_descriptor;
  }
};

}

#endif
