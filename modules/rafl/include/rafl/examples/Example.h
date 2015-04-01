/**
 * rafl: Example.h
 */

#ifndef H_RAFL_EXAMPLE
#define H_RAFL_EXAMPLE

#include <ostream>

#include <tvgutil/LimitedContainer.h>
#include <tvgutil/Serialization.h>

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

  //#################### SERIALIZATIONN ####################  
private:
  friend class boost::serialization::access;
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    // Intentionally left empty.
  }

  template<typename Archive, typename Dtype>
  friend void boost::serialization::save_construct_data(Archive& ar, const rafl::Example<Dtype> *example, const unsigned int file_version);

  template<typename Archive, typename Dtype>
  friend void boost::serialization::load_construct_data(Archive& ar, rafl::Example<Dtype> *example, const unsigned int file_version);
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

namespace boost { namespace serialization {
template<typename Archive, typename Label>
inline void save_construct_data(Archive& ar, const rafl::Example<Label> *example, const unsigned int file_version)
{
  ar << example->m_descriptor;
  ar << example->m_label;
}

template<typename Archive, typename Label>
inline void load_construct_data(Archive& ar, rafl::Example<Label> *example, const unsigned int file_version)
{
  //Retrieve data from archive required to construct new instance.
  rafl::Descriptor_Ptr descriptor;
  ar >> descriptor;

  int label;
  ar >> label;

  ::new(example)rafl::Example<Label>(descriptor, label);
}
}}

#endif
