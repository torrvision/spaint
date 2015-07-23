/**
 * rafl: Histogram.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_RAFL_HISTOGRAM
#define H_RAFL_HISTOGRAM

#include <map>
#include <stdexcept>

#include <boost/serialization/serialization.hpp>

#include <tvgutil/LimitedContainer.h>

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a histogram over the specified label type.
 */
template <typename Label>
class Histogram
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The bins that record the number of instances of each label that have been seen. */
  std::map<Label,size_t> m_bins;

  /** The total number of instances that are in the histogram. */
  size_t m_count;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an empty histogram.
   */
  Histogram()
  : m_count(0)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Adds an instance of the specified label to the histogram.
   *
   * \param label The label for which to add an instance.
   */
  void add(const Label& label)
  {
    ++m_bins[label];
    ++m_count;
  }

  /**
   * \brief Gets whether or not this is an empty histogram.
   *
   * \return  true, if the histogram is empty, or false otherwise.
   */
  bool empty() const
  {
    return get_count() == 0;
  }

  /**
   * \brief Gets the bins that record the number of instances of each label that have been seen.
   *
   * \return The bins that record the number of instances of each label that have been seen.
   */
  const std::map<Label,size_t>& get_bins() const
  {
    return m_bins;
  }

  /**
   * \brief Gets the total number of instances that are in the histogram.
   *
   * \return The total number of instances that are in the histogram.
   */
  size_t get_count() const
  {
    return m_count;
  }

  //#################### SERIALIZATION #################### 
private:
  /**
   * \brief Serializes the histogram to/from an archive.
   *
   * \param ar      The archive.
   * \param version The file format version number.
   */
  template<typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & m_bins;
    ar & m_count;
  }

  friend class boost::serialization::access;
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a histogram to the specified stream.
 *
 * \param os  The stream to which to output the histogram.
 * \param rhs The histogram to output.
 * \return    The stream.
 */
template <typename Label>
std::ostream& operator<<(std::ostream& os, const Histogram<Label>& rhs)
{
  const size_t ELEMENT_DISPLAY_LIMIT = 10;
  os << tvgutil::make_limited_container(rhs.get_bins(), ELEMENT_DISPLAY_LIMIT);
  return os;
}

}

#endif
