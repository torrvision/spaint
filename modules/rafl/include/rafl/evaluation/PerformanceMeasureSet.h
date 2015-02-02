/**
 * rafl: PerformanceMeasureSet.h
 */

#ifndef H_RAFL_PERFORMANCEMEASURESET
#define H_RAFL_PERFORMANCEMEASURESET

#include <map>
#include <ostream>

#include "PerformanceMeasure.h"

namespace rafl {

/**
 * \brief An instance of this class represents a set of performance measures.
 */
class PerformanceMeasureSet
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The measures in the set. */
  std::map<std::string,PerformanceMeasure> m_measures;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a set of performance measures.
   *
   * \param measures  The measures in the set.
   */
  PerformanceMeasureSet(const std::map<std::string,PerformanceMeasure>& measures)
  : m_measures(measures)
  {}

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  static PerformanceMeasureSet average(const std::vector<PerformanceMeasureSet>& sets)
  {
    std::map<std::string,std::vector<PerformanceMeasure> > measureVector;

    for(std::vector<PerformanceMeasureSet>::const_iterator it = sets.begin(), iend = sets.end(); it != iend; ++it)
    {
      const std::map<std::string,PerformanceMeasure>& m = *it;
      for(std::map<std::string,PerformanceMeasure>::const_iterator jt = m.begin(), jend = m.end(); jt != jend; ++jt)
      {
        measureVector[jt->first].push_back(jt->second);
      }
    }

    std::map<std::string,PerformanceMeasure> averages;
    for(std::map<std::string,std::vector<PerformanceMeasure> >::const_iterator it = measureVector.begin(), iend = measureVector.end(); it != iend; ++it)
    {
      averages.insert(std::make_pair(it->first, PerformanceMeasure::average(it->second)));
    }

    return averages;
  }

  operator const std::map<std::string,PerformanceMeasure>&() const
  {
    return m_measures;
  }
};

//#################### STREAM OPERATORS ####################

std::ostream& operator<<(std::ostream& os, const PerformanceMeasureSet& rhs)
{
  // TODO: Something useful.
  return os;
}

}

#endif
