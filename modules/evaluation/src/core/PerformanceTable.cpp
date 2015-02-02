/**
 * evaluation: PerformanceTable.cpp
 */

#include "core/PerformanceTable.h"

#include <string>

#include <tvgutil/MapUtil.h>
using namespace tvgutil;

namespace evaluation {

//#################### CONSTRUCTORS ####################

PerformanceTable::PerformanceTable(const std::vector<std::string>& measureNames)
: m_measureNames(measureNames)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void PerformanceTable::output(std::ostream& os, const std::string& delimiter) const
{
  // Output the parameter names.
  const ParamSet& firstSet = m_results[0].first;
  for(ParamSet::const_iterator it = firstSet.begin(), iend = firstSet.end(); it != iend; ++it)
  {
    os << it->first << delimiter;
  }

  // Output the names of the performance measures.
  for(size_t i = 0, size = m_measureNames.size(); i < size; ++i)
  {
    os << m_measureNames[i] << "Mean" << delimiter << m_measureNames[i] << "StdDev";
  }

  os << '\n';

  // Output the results of running the algorithm with different sets of parameters (one set per row).
  for(size_t i = 0, size = m_results.size(); i < size; ++i)
  {
    // Output the parameters for the row.
    const ParamSet& params = m_results[i].first;
    for(ParamSet::const_iterator jt = params.begin(), jend = params.end(); jt != jend; ++jt)
    {
      os << jt->second << delimiter;
    }

    // Output the performance measures for the row.
    const std::map<std::string,PerformanceMeasure>& measures = m_results[i].second;
    for(size_t i = 0, size = m_measureNames.size(); i < size; ++i)
    {
      const PerformanceMeasure& measure = MapUtil::lookup(measures, m_measureNames[i]);
      os << measure.get_mean() << delimiter << measure.get_std_dev();
      if(i != size - 1) os << delimiter;
    }

    os << '\n';
  }
}

void PerformanceTable::record_performance(const ParamSet& params, const std::map<std::string,PerformanceMeasure>& measures)
{
  m_results.push_back(std::make_pair(params, measures));
}

}
