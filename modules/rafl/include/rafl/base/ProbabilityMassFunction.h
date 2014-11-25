/**
 * rafl: ProbabilityMassFunction.h
 */

#ifndef H_RAFL_PROBABILITYMASSFUNCTION
#define H_RAFL_PROBABILITYMASSFUNCTION

#include <cassert>
#include <cmath>
#include <stdexcept>

#include <boost/optional.hpp>

#include <tvgutil/LimitedMap.h>
#include "Histogram.h"

namespace rafl {

//#################### GLOBAL CONSTANTS ####################

const float SMALL_EPSILON = 1e-9f;

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
   * \brief Constructs a probability mass function (PMF) by normalising a map from labels -> masses.
   *
   * \pre
   *   - !masses.empty()
   *   - Each mass >= 0
   *   - At least one mass > 0
   *
   * \param masses  The label -> masses map to normalise.
   */
  explicit ProbabilityMassFunction(const std::map<Label,float>& masses)
  : m_masses(masses)
  {
    assert(!masses.empty());

    // Calculate the sum of all the masses (we will divide by this to normalise the PMF).
    float sum = 0.0f;
    for(typename std::map<Label,float>::const_iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      assert(it->second >= 0.0f);
      sum += it->second;
    }

    if(fabs(sum) < SMALL_EPSILON) throw std::runtime_error("Cannot normalise the probability mass function: denominator too small");

    // Normalise the PMF by dividing each mass by the sum.
    for(typename std::map<Label,float>::iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      it->second /= sum;
    }
  }

  /**
   * \brief Constructs a probability mass function (PMF) as a normalised version of the specified histogram.
   *
   * \param histogram   The histogram from which to construct a PMF.
   * \param multipliers Optional per-class ratios that can be used to scale the probabilities for the different labels.
   */
  explicit ProbabilityMassFunction(const Histogram<Label>& histogram, const boost::optional<std::map<Label,float> >& multipliers = boost::none)
  {
    // Determine the masses for the labels in the histogram by dividing the number of instances in each bin by the histogram count.
    const std::map<Label,size_t>& bins = histogram.get_bins();
    size_t count = histogram.get_count();
    if(count == 0) throw std::runtime_error("Cannot make a probability mass function from an empty histogram");
    for(typename std::map<Label,size_t>::const_iterator it = bins.begin(), iend = bins.end(); it != iend; ++it)
    {
      float mass = static_cast<float>(it->second) / count;

      // Scale the mass by the relevant multiplier for the corresponding class (if supplied).
      if(multipliers)
      {
        typename std::map<Label,float>::const_iterator jt = multipliers->find(it->first);
        if(jt != multipliers->end()) mass *= jt->second;
      }

      // Our implementation is dependent on the masses never becoming too small. If this assumption turns out not to be ok,
      // we may need to change the implementation.
      assert(mass >= SMALL_EPSILON);

      m_masses.insert(std::make_pair(it->first, mass));
    }
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Returns a label in the PMF that has the highest mass.
   *
   * Note that there can be more than one label with the highest mass - this function returns one of them,
   * and the one returned will be the same each time the function is called (i.e. it's deterministic).
   *
   * \return  The calculated label.
   */
  Label calculate_best_label() const
  {
    Label bestLabel;
    float bestMass = 0.0f;
    for(typename std::map<Label,float>::const_iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      if(it->second > bestMass)
      {
        bestLabel = it->first;
        bestMass = it->second;
      }
    }
    assert(bestMass > 0.0f);  // note: there must be at least one label that has a non-zero mass
    return bestLabel;
  }

  /**
   * \brief Calculates the entropy of the PMF using the definition H(X) = -sum_{i} P(x_i) log2(P(x_i)).
   * 
   * \return The entropy of the PMF. When outcomes are equally likely, the entropy will be high; when the outcome is predictable, the entropy wil be low.
   */
  float calculate_entropy() const
  {
    float entropy = 0.0f;
    for(typename std::map<Label,float>::const_iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      float mass = it->second;
      if(mass > 0)
      {
        // Note: If P(x_i) = 0, the value of the corresponding sum 0*log2(0) is taken to be 0, since lim{p->0+} p*log2(p) = 0 (see Wikipedia!).
        entropy += mass * log2(mass);
      }
    }
    return -entropy;
  }

  /**
   * \brief Gets the masses for the various labels.
   *
   * \return The masses for the various labels.
   */
  const std::map<Label,float>& get_masses() const
  {
    return m_masses;
  }
};

//#################### STREAM OPERATORS ####################

/**
 * \brief Outputs a probability mass function (PMF) to the specified stream.
 *
 * \param os  The stream to which to output the PMF.
 * \param rhs The PMF to output.
 * \return    The stream.
 */
template <typename Label>
std::ostream& operator<<(std::ostream& os, const ProbabilityMassFunction<Label>& rhs)
{
  const size_t ELEMENT_DISPLAY_LIMIT = 3;
  os << tvgutil::make_limited_map(rhs.get_masses(), ELEMENT_DISPLAY_LIMIT);
  return os;
}

}

#endif
