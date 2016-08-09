/**
 * evaluation: PerformanceTable.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "core/PerformanceTable.h"

#include <tvgutil/containers/MapUtil.h>
using namespace tvgutil;

namespace evaluation {

//#################### CONSTRUCTOR ####################

PerformanceTable::PerformanceTable(const std::vector<std::string>& measureNames)
: m_measureNames(measureNames)
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

const ParamSet& PerformanceTable::find_best_param_set(const std::string& measureName) const
{
  size_t bestRow = 0;
  float bestResult = static_cast<float>(INT_MIN);
  for(size_t i = 0, size = m_results.size(); i < size; ++i)
  {
    const PerformanceResult& result = m_results[i].second;
    const PerformanceMeasure& measure = MapUtil::lookup(result, measureName);

    float mean = measure.get_mean();
    if(mean > bestResult)
    {
      bestResult = mean;
      bestRow = i;
    }
  }

  return m_results[bestRow].first;
}

void PerformanceTable::output(std::ostream& os, const std::string& delimiter) const
{
  // Output the titles for the parameter columns.
  os << "ParameterString" << delimiter;
  const ParamSet& firstSet = m_results[0].first;
  for(ParamSet::const_iterator it = firstSet.begin(), iend = firstSet.end(); it != iend; ++it)
  {
    os << it->first << delimiter;
  }

  // Output the titles for the measure columns.
  os << "Measure" << delimiter << "Mean" << delimiter << "StdDev" << '\n';

  // Output the values of the measures for different sets of parameters (one parameter set / measure combination per row).
  for(size_t measureIndex = 0, measureCount = m_measureNames.size(); measureIndex < measureCount; ++measureIndex)
  {
    for(size_t resultIndex = 0, resultCount = m_results.size(); resultIndex < resultCount; ++resultIndex)
    {
      // Output the parameters associated with this set of results.
      const ParamSet& params = m_results[resultIndex].first;
      os << ParamSetUtil::param_set_to_string(params) << delimiter;
      for(ParamSet::const_iterator it = params.begin(), iend = params.end(); it != iend; ++it)
      {
        os << it->second << delimiter;
      }

      // Output the measure name.
      os << m_measureNames[measureIndex] << delimiter;

      // Output the corresponding measure mean and standard deviation.
      const PerformanceMeasure& measure = MapUtil::lookup(m_results[resultIndex].second, m_measureNames[measureIndex]);
      os << measure.get_mean() << delimiter << measure.get_std_dev() << '\n';
    }
  }
}

void PerformanceTable::record_performance(const ParamSet& params, const PerformanceResult& measures)
{
  m_results.push_back(std::make_pair(params, measures));
}

}
