/**
 * rafl: ProbabilityMassFunction.h
 */

#ifndef H_RAFL_PROBABILITYMASSFUNCTION
#define H_RAFL_PROBABILITYMASSFUNCTION

#include <cmath>

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
   * \brief Calculates the entropy of a probability mass function (PMF) as H(X) = -sum_{i} P(x_i) ln(P(x_i)).
   * 
   * \return The entropy of the PMF. When outcomes are equally likely, the entropy will be high; when the outcome is predictable, the entropy wil be low.
   */
  float calculate_entropy() const
  {
    float entropy = 0.0f;
    for(typename std::map<Label,float>::const_iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      float mass = it->second;
      //log from <cmath> == natural logarithm, log_{e}. The unit of entropy calculated with log_{e} is the "nat".
      //if P(x_i) == 0, the value of the corresponding sum 0*ln(0) is taken to be 0. lim{p->0+} p*log(p) = 0. (source: Wikipedia!)
      if(mass > 0) entropy += mass * log(mass);
    }
    return -entropy;
  }

  /**
   * \brief Returns a const reference to the probability mass function.
   */
  const std::map<Label,float>& get_masses() const
  {
    return m_masses;
  }
};

//#################### STREAM OPERATORS ####################

template <typename Label>
std::ostream& operator<<(std::ostream& os, const ProbabilityMassFunction<Label>& rhs)
{
  os << tvgutil::make_limited_map(rhs.get_masses(), 3);
  return os;
}

}

#endif
