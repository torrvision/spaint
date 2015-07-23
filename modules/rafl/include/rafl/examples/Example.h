/**
 * rafl: Example.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFL_EXAMPLE
#define H_RAFL_EXAMPLE

#include <ostream>

#include <boost/serialization/serialization.hpp>

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

private:
  /**
   * \brief Constructs an example.
   *
   * Note: This constructor is needed for serialization and should not be used otherwise.
   */
  Example() {}

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

  //#################### SERIALIZATION ####################
private:
  /**
   * \brief Serializes the example to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & m_descriptor;
    ar & m_label;
  }

  friend class boost::serialization::access;
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs an example to a stream.
 *
 * \param os  The stream to which to output the example.
 * \param rhs The example to output.
 * \return    The stream.
 */
template <typename Label>
std::ostream& operator<<(std::ostream& os, const Example<Label>& rhs)
{
  const size_t ELEMENT_DISPLAY_LIMIT = 5;
  os << tvgutil::make_limited_container(*rhs.get_descriptor(), ELEMENT_DISPLAY_LIMIT) << ' ' << rhs.get_label();
  return os;
}

}

#endif
