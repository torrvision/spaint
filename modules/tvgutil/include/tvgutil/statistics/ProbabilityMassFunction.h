/**
 * tvgutil: ProbabilityMassFunction.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#ifndef H_TVGUTIL_PROBABILITYMASSFUNCTION
#define H_TVGUTIL_PROBABILITYMASSFUNCTION

#include <cassert>
#include <cmath>

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include "Histogram.h"
#include "../misc/ArgUtil.h"

namespace tvgutil {

//#################### GLOBAL CONSTANTS ####################

const float SMALL_EPSILON = 1e-9f;

/**
 * \brief An instance of an instantiation of this class template represents a probability mass function (PMF).
 *
 * Datatype Invariant: The masses in the PMF must sum to 1.
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
    normalise();
    ensure_invariant();
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

    if(multipliers) normalise();

    ensure_invariant();
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
    return ArgUtil::argmax(m_masses);
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

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Calculates the sum of the masses in the PMF.
   *
   * \return  The sum of the masses in the PMF.
   */
  float calculate_sum()
  {
    float sum = 0.0f;
    for(typename std::map<Label,float>::const_iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      assert(it->second >= 0.0f);
      sum += it->second;
    }
    return sum;
  }

  /**
   * \brief Ensures that the datatype invariant for the PMF is satisfied, i.e. that its masses sum to 1.
   */
  void ensure_invariant()
  {
    const float MAX_TOLERANCE = 1e-5f;
    float sum = calculate_sum();
    if(fabs(sum - 1.0f) >= MAX_TOLERANCE)
    {
      throw std::runtime_error("The masses in the PMF should sum to 1, but they sum to " + boost::lexical_cast<std::string>(sum));
    }
  }

  /**
   * \brief Normalises the PMF by dividing by the sum of its masses.
   */
  void normalise()
  {
    // Calculate the sum of the masses in the PMF.
    float sum = calculate_sum();
    if(fabs(sum) < SMALL_EPSILON) throw std::runtime_error("Cannot normalise the probability mass function: denominator too small");

    // Normalise the PMF by dividing each mass by the sum.
    for(typename std::map<Label,float>::iterator it = m_masses.begin(), iend = m_masses.end(); it != iend; ++it)
    {
      it->second /= sum;
    }
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
  os << make_limited_container(rhs.get_masses(), ELEMENT_DISPLAY_LIMIT);
  return os;
}

}

#endif
