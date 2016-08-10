/**
 * evaluation: PerformanceResultUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "util/PerformanceResultUtil.h"

namespace evaluation {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PerformanceResult PerformanceResultUtil::average_results(const std::vector<PerformanceResult>& results)
{
  std::map<std::string,std::vector<PerformanceMeasure> > groupedResults;

  // Group the measures.
  for(std::vector<PerformanceResult>::const_iterator it = results.begin(), iend = results.end(); it != iend; ++it)
  {
    const PerformanceResult& result = *it;
    for(PerformanceResult::const_iterator jt = result.begin(), jend = result.end(); jt != jend; ++jt)
    {
      groupedResults[jt->first].push_back(jt->second);
    }
  }

  // Average the measures in each group.
  PerformanceResult averagedResult;
  for(std::map<std::string,std::vector<PerformanceMeasure> >::const_iterator it = groupedResults.begin(), iend = groupedResults.end(); it != iend; ++it)
  {
    averagedResult.insert(std::make_pair(it->first, PerformanceMeasure::average(it->second)));
  }

  return averagedResult;
}

}
