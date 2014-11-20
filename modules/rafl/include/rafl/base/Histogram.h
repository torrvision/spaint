/**
 * rafl: Histogram.h
 */

#ifndef H_RAFL_HISTOGRAM
#define H_RAFL_HISTOGRAM

#include <map>
#include <stdexcept>

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
};

}

#endif
