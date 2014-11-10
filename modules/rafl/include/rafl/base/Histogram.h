/**
 * rafl: Histogram.h
 */

#ifndef H_RAFL_HISTOGRAM
#define H_RAFL_HISTOGRAM

#include <map>

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
   * \brief TODO
   */
  void add(const Label& label)
  {
    ++m_bins[label];
    ++m_count;
  }

  /**
   * \brief TODO
   */
  const std::map<Label,size_t>& get_bins() const
  {
    return m_bins;
  }

  /**
   * \brief TODO
   */
  size_t get_count() const
  {
    return m_count;
  }
};

}

#endif
