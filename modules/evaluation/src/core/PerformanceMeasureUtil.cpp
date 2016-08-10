/**
 * evaluation: PerformanceMeasureUtil.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2016. All rights reserved.
 */

#include "core/PerformanceMeasureUtil.h"

#include <cmath>
#include <stdexcept>

namespace evaluation {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

PerformanceMeasure PerformanceMeasureUtil::average_measures(const std::vector<PerformanceMeasure>& measures)
{
  // Check that there are measures to average.
  if(measures.empty())
  {
    throw std::runtime_error("Cannot average an empty set of measures");
  }

  // Compute the mean of the combined measure.
  float combinedMean = 0.0f;
  size_t combinedSampleCount = 0;
  for(size_t i = 0, size = measures.size(); i < size; ++i)
  {
    size_t sampleCount = measures[i].get_sample_count();
    combinedMean += sampleCount * measures[i].get_mean();
    combinedSampleCount += sampleCount;
  }
  combinedMean /= combinedSampleCount;

  // Compute the variance of the combined measure.
  float combinedVariance = 0.0f;
  for(size_t i = 0, size = measures.size(); i < size; ++i)
  {
    float delta = measures[i].get_mean() - combinedMean;
    combinedVariance += measures[i].get_sample_count() * (measures[i].get_variance() + delta * delta);
  }
  combinedVariance /= combinedSampleCount;

  return PerformanceMeasure(combinedSampleCount, combinedMean, sqrt(combinedVariance));
}

PerformanceResult PerformanceMeasureUtil::average_results(const std::vector<PerformanceResult>& results)
{
  std::map<std::string,std::vector<PerformanceMeasure> > groupedMeasures;

  // Group the measures.
  for(std::vector<PerformanceResult>::const_iterator it = results.begin(), iend = results.end(); it != iend; ++it)
  {
    const PerformanceResult& result = *it;
    for(PerformanceResult::const_iterator jt = result.begin(), jend = result.end(); jt != jend; ++jt)
    {
      groupedMeasures[jt->first].push_back(jt->second);
    }
  }

  // Average the measures in each group.
  PerformanceResult averagedResult;
  for(std::map<std::string,std::vector<PerformanceMeasure> >::const_iterator it = groupedMeasures.begin(), iend = groupedMeasures.end(); it != iend; ++it)
  {
    averagedResult.insert(std::make_pair(it->first, average_measures(it->second)));
  }

  return averagedResult;
}

}
