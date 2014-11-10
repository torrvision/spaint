/**
 * rafl: ProbabilityMassFunction.h
 */

#ifndef H_RAFL_PROBABILITYMASSFUNCTION
#define H_RAFL_PROBABILITYMASSFUNCTION

#include <cassert>
#include <ostream>

#include <tvgutil/LimitedMap.h>
#include "Histogram.h"

namespace rafl {

/**
 * \brief An instance of an instantiation of this class template represents a probability mass function (PMF).
 */
template <typename Label>
class ProbabilityMassFunction
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The masses for the various labels. */
  std::map<Label,float> m_masses;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a probability mass function (PMF) as a normalised version of the specified histogram.
   *
   * \param histogram The histogram from which to construct our PMF.
   */
  explicit ProbabilityMassFunction(const Histogram<Label>& histogram)
  {
    const float SMALL_EPSILON = 1e-9f;

    // Determine the masses for the labels in the histogram by dividing the number of instances in each bin by the histogram count.
    const std::map<Label,size_t>& bins = histogram.get_bins();
    size_t count = histogram.get_count();
    for(typename std::map<Label,size_t>::const_iterator it = bins.begin(), iend = bins.end(); it != iend; ++it)
    {
      float mass = static_cast<float>(it->second) / count;

      // Our implementation is dependent on the masses never becoming too small. If this assumption turns out not to be ok,
      // we may need to change the implementation.
      assert(mass >= SMALL_EPSILON);

      m_masses.insert(std::make_pair(it->first, mass));
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief TODO
   */
  const std::map<Label,float>& get_masses() const
  {
    return m_masses;
  }
};

template <typename Label>
std::ostream& operator<<(std::ostream& os, const ProbabilityMassFunction<Label>& rhs)
{
  os << tvgutil::make_limited_map(rhs.get_masses(), 3);
  return os;
}

}

#endif
