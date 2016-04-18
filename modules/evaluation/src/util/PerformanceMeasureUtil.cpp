/**
 * evaluation: PerformanceMeasureUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2015. All rights reserved.
 */

#include "util/PerformanceMeasureUtil.h"

namespace evaluation {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

std::map<std::string,PerformanceMeasure> PerformanceMeasureUtil::average_results(const std::vector<std::map<std::string,PerformanceMeasure> >& results)
{
  std::map<std::string,std::vector<PerformanceMeasure> > groupedResults;

  // Group the results by measure.
  for(std::vector<std::map<std::string,PerformanceMeasure> >::const_iterator it = results.begin(), iend = results.end(); it != iend; ++it)
  {
    const std::map<std::string,PerformanceMeasure>& result = *it;
    for(std::map<std::string,PerformanceMeasure>::const_iterator jt = result.begin(), jend = result.end(); jt != jend; ++jt)
    {
      groupedResults[jt->first].push_back(jt->second);
    }
  }

  // Average the results for each measure.
  std::map<std::string,PerformanceMeasure> averagedResults;
  for(std::map<std::string,std::vector<PerformanceMeasure> >::const_iterator it = groupedResults.begin(), iend = groupedResults.end(); it != iend; ++it)
  {
    averagedResults.insert(std::make_pair(it->first, PerformanceMeasure::average(it->second)));
  }

  return averagedResults;
}

}
