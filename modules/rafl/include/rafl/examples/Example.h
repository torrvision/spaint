/**
 * rafl: Example.h
 */

#ifndef H_RAFL_EXAMPLE
#define H_RAFL_EXAMPLE

#include <ostream>

#include <boost/serialization/serialization.hpp>

#include <tvgutil/LimitedContainer.h>

#include "../base/Descriptor.h"

//#################### FORWARD DECLARATIONS ####################

namespace rafl {

template <typename Label> class Example;

}

namespace boost { namespace serialization {

template<typename Archive, typename LabelType> void load_construct_data(Archive&, rafl::Example<LabelType>*, const unsigned int);
template<typename Archive, typename LabelType> void save_construct_data(Archive&, const rafl::Example<LabelType>*, const unsigned int);

}}

//#################### MAIN CLASS TEMPLATE ####################

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

  //#################### SERIALIZATION ####################
private:
  /**
   * \brief Serializes the example to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    // No-op
  }

  friend class boost::serialization::access;

  template<typename Archive, typename LabelType>
  friend void boost::serialization::load_construct_data(Archive&, Example<LabelType>*, const unsigned int);

  template<typename Archive, typename LabelType>
  friend void boost::serialization::save_construct_data(Archive&, const Example<LabelType>*, const unsigned int);
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

//#################### SERIALIZATION ####################

namespace boost { namespace serialization {

/**
 * \brief Loads an example from an archive.
 *
 * \param ar      The archive.
 * \param rhs     A pointer to some memory into which to load the example.
 * \param version The file format version number.
 */
template<typename Archive, typename LabelType>
void load_construct_data(Archive& ar, rafl::Example<LabelType> *rhs, const unsigned int version)
{
  rafl::Descriptor_Ptr descriptor;
  int label;

  ar >> descriptor;
  ar >> label;

  ::new (rhs) rafl::Example<LabelType>(descriptor, label);
}

/**
 * \brief Saves an example to an archive.
 *
 * \param ar      The archive.
 * \param rhs     The example to save.
 * \param version The file format version number.
 */
template<typename Archive, typename LabelType>
void save_construct_data(Archive& ar, const rafl::Example<LabelType> *rhs, const unsigned int version)
{
  ar << rhs->m_descriptor;
  ar << rhs->m_label;
}

}}

#endif
